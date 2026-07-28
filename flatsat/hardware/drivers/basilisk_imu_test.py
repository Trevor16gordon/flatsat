"""Basilisk IMU driver: truth in, device-corrupted samples out.

No Basilisk needed — the driver's contract is with the TRUTH TOPIC, so a
test publisher stands in for the bridge.
"""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.core.config import load_imu_spec
from flatsat.hardware.drivers.basilisk_imu import BasiliskImuDriver
from flatsat.msgs import hal_pb2, sim_pb2

TOPIC = "test/sim/truth_imu"
SPEC = load_imu_spec()


@pytest.fixture(name="truth_session")
def fixture_truth_session() -> Iterator[zenoh.Session]:
    """One zenoh session for publishing truth, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def _publish_truth(
    session: zenoh.Session, rates: tuple[float, float, float], topic: str = TOPIC
) -> None:
    msg = sim_pb2.TruthState()
    msg.omega_x_rad_s, msg.omega_y_rad_s, msg.omega_z_rad_s = rates
    session.put(topic, msg.SerializeToString())


def _wait_for_fresh(driver: BasiliskImuDriver, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        _, flags = driver.read()
        if not flags & hal_pb2.VALIDITY_FLAG_STALE:
            return True
        time.sleep(0.05)
    return False


@pytest.mark.verifies("FSW-SIM-004")
def test_no_truth_reads_stale_flagged_zeros() -> None:
    """No bridge running is the correct quiet state, not an error."""
    driver = BasiliskImuDriver(SPEC, truth_topic="test/sim/truth_none", stale_after_s=0.1)
    try:
        msg, flags = driver.read()
    finally:
        driver.close()
    assert flags & hal_pb2.VALIDITY_FLAG_STALE
    assert isinstance(msg, hal_pb2.ImuSample)
    assert msg.gyro_x_rad_s == 0.0, "no value smuggled alongside a STALE flag"


def test_fresh_truth_reads_valid_corrupted_sample(truth_session: zenoh.Session) -> None:
    driver = BasiliskImuDriver(SPEC, truth_topic=TOPIC, stale_after_s=5.0, seed=7)
    try:
        time.sleep(0.5)  # discovery
        _publish_truth(truth_session, (0.05, -0.02, 0.01))
        assert _wait_for_fresh(driver), "truth never reached the driver"
        msg, flags = driver.read()
        assert isinstance(msg, hal_pb2.ImuSample)
        assert flags == hal_pb2.VALIDITY_FLAG_VALID
        # Corruption comes from the SHARED model: value near truth, on the
        # device's quantization grid.
        assert msg.gyro_x_rad_s == pytest.approx(0.05, abs=6 * SPEC.gyro_noise_rad_s)
        steps = msg.gyro_x_rad_s / SPEC.gyro_lsb_rad_s
        assert steps == pytest.approx(round(steps), abs=1e-6)
    finally:
        driver.close()


def test_truth_beyond_full_scale_saturates_and_flags(truth_session: zenoh.Session) -> None:
    driver = BasiliskImuDriver(SPEC, truth_topic=TOPIC, stale_after_s=5.0, seed=7)
    try:
        time.sleep(0.5)
        _publish_truth(truth_session, (SPEC.gyro_full_scale_rad_s * 2.0, 0.0, 0.0))
        assert _wait_for_fresh(driver), "truth never reached the driver"
        msg, flags = driver.read()
        assert isinstance(msg, hal_pb2.ImuSample)
        assert flags & hal_pb2.VALIDITY_FLAG_SATURATED
        assert msg.gyro_x_rad_s == pytest.approx(
            SPEC.gyro_full_scale_rad_s, abs=SPEC.gyro_lsb_rad_s
        )
    finally:
        driver.close()


@pytest.mark.verifies("FSW-SIM-004")
def test_truth_going_stale_flags_again(truth_session: zenoh.Session) -> None:
    """A bridge that stops mid-run must degrade to STALE, not freeze values."""
    driver = BasiliskImuDriver(SPEC, truth_topic=TOPIC, stale_after_s=0.2, seed=7)
    try:
        time.sleep(0.5)
        _publish_truth(truth_session, (0.03, 0.0, 0.0))
        assert _wait_for_fresh(driver), "truth never reached the driver"
        time.sleep(0.35)  # exceed stale_after_s with no further truth
        msg, flags = driver.read()
        assert isinstance(msg, hal_pb2.ImuSample)
        assert flags & hal_pb2.VALIDITY_FLAG_STALE
        assert msg.gyro_x_rad_s == 0.0
    finally:
        driver.close()


def test_from_config_builds_from_vehicle_options() -> None:
    driver = BasiliskImuDriver.from_config(
        "imu_cfg_test",
        {"spec": "config/devices/imu0.toml", "truth_topic": "test/sim/truth_cfg", "seed": 1},
    )
    try:
        described = "\n".join(driver.describe())
    finally:
        driver.close()
    assert "test/sim/truth_cfg" in described
    assert "imu0" in described
