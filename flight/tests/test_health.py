"""Health telemetry: the system's own vitals must be DATA, not printf."""

import threading
import time

import pytest
import zenoh

from flight.apps.control_loop import ControlLoop, LoopReport
from flight.apps.sensor_daemon import SensorDaemon
from flight.core.config import ControlEntry, SensorEntry
from flight.core.health import health_topic, percentiles
from flight.hal.drivers.sim_gyro import SimGyroDriver
from flight.msgs import health_pb2
from flight.registry import get_controller_class, get_guidance_class

RECV_TIMEOUT_S = 5.0


def test_percentiles_summarize_a_window() -> None:
    summary = percentiles([1.0, 2.0, 3.0, 4.0, 100.0])
    assert summary.count == 5
    assert summary.max == pytest.approx(100.0)
    assert summary.p50 == pytest.approx(3.0)


def test_percentiles_of_empty_window_are_zero() -> None:
    summary = percentiles([])
    assert summary.count == 0 and summary.max == 0.0


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


@pytest.mark.verifies("FSW-ADCS-010", "FSW-CFG-002")
def test_loop_health_carries_provenance_and_counters() -> None:
    entry = ControlEntry(
        strategy="rate_damping",
        objective="constant_rate",
        rate_hz=100.0,
        input_topic="test/health/in",
        output_topic="test/health/out",
        stale_after_s=0.05,
        options={"kp": 0.02, "kd": 0.005},
        objective_options={},
    )
    session = zenoh.open(zenoh.Config())
    loop = ControlLoop(
        session,
        entry,
        get_controller_class(entry.strategy).from_config(entry.options),
        get_guidance_class(entry.objective).from_config(entry.objective_options),
        vehicle_name="test-vehicle",
        config_checksum="deadbeef1234",
    )
    loop.scheduling = "SCHED_FIFO priority 80"
    loop.cpu_affinity = "3"

    report = LoopReport(
        cycles=6000,
        wakeup_lateness_us=[10.0, 20.0, 30.0],
        exec_time_us=[5.0, 6.0, 7.0],
        stale_cycles=12,
        saturated_cycles=3,
    )
    msg = report.to_proto(loop, loop.scheduling, loop.cpu_affinity)
    loop.close()
    session.close()

    # Provenance: a recorded window must trace to what produced it.
    assert msg.vehicle == "test-vehicle"
    assert msg.config_checksum == "deadbeef1234"
    assert msg.strategy == "rate_damping"
    assert msg.objective == "constant_rate"
    assert msg.scheduling == "SCHED_FIFO priority 80"
    assert msg.cpu_affinity == "3"
    # Counters and distributions survive serialization intact.
    back = health_pb2.LoopHealth.FromString(msg.SerializeToString())
    assert back.window_cycles == 6000
    assert back.stale_cycles == 12
    assert back.saturated_cycles == 3
    assert back.wakeup_lateness_us.max == pytest.approx(30.0)
    assert back.exec_time_us.count == 3
