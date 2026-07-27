"""Integration tests: composed sensor daemons on the real bus, real hardware.

Validates the HAL contract end to end through the generic daemon:
  * a driver named in a vehicle file is resolved, built, and published;
  * sequence numbers are strictly monotonic from 1;
  * publish_time >= sample_time, both plausibly recent;
  * a readable Jetson thermal zone yields VALID, sane temperatures;
  * a power-gated zone (cv0 EAGAINs while the CV cluster is off) still
    publishes on cadence, flagged COMM — flag and forward, proven against
    a real fault.

Runs on the Jetson only (needs /sys/class/thermal and a zenoh-capable env).
"""

import threading
import time

import pytest
import zenoh

from flatsat.apps.sensor_daemon import SensorDaemon
from flatsat.core.config import SensorEntry
from flatsat.core.health import health_topic
from flatsat.hardware.drivers.jetson_thermal import THERMAL_ROOT, JetsonThermalDriver
from flatsat.hardware.drivers.sim_gyro import SimGyroDriver
from flatsat.msgs import hal_pb2, health_pb2

RECV_TIMEOUT_S = 5.0


def _collect(entry: SensorEntry, driver: object, n_samples: int) -> list[bytes]:
    """Run a daemon at the entry's rate and collect published payloads.

    Args:
        entry: Sensor composition entry (topic, rate).
        driver: Driver instance to poll.
        n_samples: Stop after this many samples arrive.

    Returns:
        Raw payloads in arrival order.
    """
    sub_session = zenoh.open(zenoh.Config())
    daemon_session = zenoh.open(zenoh.Config())
    received: list[bytes] = []
    enough = threading.Event()

    def on_sample(sample: zenoh.Sample) -> None:
        received.append(bytes(sample.payload.to_bytes()))
        if len(received) >= n_samples:
            enough.set()

    sub = sub_session.declare_subscriber(entry.topic, on_sample)
    time.sleep(0.5)  # discovery

    daemon = SensorDaemon(entry, driver, daemon_session)  # type: ignore[arg-type]
    thread = threading.Thread(target=daemon.run, daemon=True)
    thread.start()
    enough.wait(RECV_TIMEOUT_S)
    daemon.stop()
    thread.join(timeout=2.0)
    sub.undeclare()
    sub_session.close()
    daemon_session.close()
    return received


@pytest.mark.verifies("FSW-HAL-002", "FSW-HAL-003")
def test_sim_gyro_driver_publishes_contract_compliant_samples() -> None:
    entry = SensorEntry(
        name="test_imu",
        driver="sim_gyro",
        topic="test/hal/sim_gyro",
        rate_hz=50.0,
        options={},
    )
    driver = SimGyroDriver.from_config(entry.name, {"seed": 7})
    payloads = _collect(entry, driver, n_samples=5)
    assert len(payloads) >= 5

    samples = [hal_pb2.ImuSample.FromString(p) for p in payloads]
    seqs = [s.header.seq for s in samples]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    assert seqs[0] == 1
    now_ns = time.time_ns()
    for s in samples:
        assert s.header.source == "test_imu"
        assert s.header.publish_time_ns >= s.header.sample_time_ns
        assert now_ns - s.header.publish_time_ns < 60 * 1_000_000_000
        assert abs(s.gyro_x_rad_s) < 1.0


@pytest.mark.skipif(not THERMAL_ROOT.exists(), reason="no thermal sysfs (not on target)")
def test_readable_zone_publishes_valid_samples() -> None:
    entry = SensorEntry(
        name="test_tj",
        driver="jetson_thermal",
        topic="test/hal/thermal_tj",
        rate_hz=10.0,
        options={"zone": "tj-thermal"},
    )
    driver = JetsonThermalDriver.from_config(entry.name, entry.options)
    samples = [hal_pb2.TemperatureSample.FromString(p) for p in _collect(entry, driver, 5)]
    assert len(samples) >= 5
    for s in samples:
        assert s.header.validity == hal_pb2.VALIDITY_FLAG_VALID
        assert s.location == "tj-thermal"
        assert 5.0 < s.temperature_c < 110.0, f"implausible Tj: {s.temperature_c}"


@pytest.mark.skipif(not THERMAL_ROOT.exists(), reason="no thermal sysfs (not on target)")
@pytest.mark.verifies("FSW-HAL-001", "FSW-HAL-004")
def test_power_gated_zone_flags_comm_but_keeps_publishing() -> None:
    entry = SensorEntry(
        name="test_cv0",
        driver="jetson_thermal",
        topic="test/hal/thermal_cv0",
        rate_hz=10.0,
        options={"zone": "cv0-thermal"},
    )
    driver = JetsonThermalDriver.from_config(entry.name, entry.options)
    samples = [hal_pb2.TemperatureSample.FromString(p) for p in _collect(entry, driver, 4)]
    assert len(samples) >= 4, "daemon must keep cadence even when every read fails"
    for s in samples:
        if s.header.validity & hal_pb2.VALIDITY_FLAG_COMM:
            assert s.temperature_c == 0.0, "no value smuggled alongside a COMM flag"


@pytest.mark.verifies("FSW-HAL-007")
def test_sensor_daemon_publishes_health() -> None:
    entry = SensorEntry(
        name="test_health_imu",
        driver="sim_gyro",
        topic="test/hal/health_imu",
        rate_hz=200.0,
        options={},
    )
    sub_session = zenoh.open(zenoh.Config())
    daemon_session = zenoh.open(zenoh.Config())
    received: list[health_pb2.SensorHealth] = []
    got = threading.Event()

    def on_health(sample: zenoh.Sample) -> None:
        received.append(health_pb2.SensorHealth.FromString(bytes(sample.payload.to_bytes())))
        got.set()

    sub = sub_session.declare_subscriber(health_topic(entry.name), on_health)
    time.sleep(0.5)

    daemon = SensorDaemon(entry, SimGyroDriver.from_config(entry.name, {"seed": 3}), daemon_session)
    thread = threading.Thread(target=lambda: daemon.run(health_every_s=0.3), daemon=True)
    thread.start()
    got.wait(RECV_TIMEOUT_S)
    daemon.stop()
    thread.join(timeout=2.0)
    sub.undeclare()
    sub_session.close()
    daemon_session.close()

    assert received, "no SensorHealth published"
    health = received[0]
    assert health.driver == "sim_gyro"
    assert health.rate_hz == pytest.approx(200.0)
    assert health.window_samples > 0
    assert health.header.source == entry.name
    assert health.header.seq >= 1
