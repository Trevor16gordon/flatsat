"""Simulated IMU driver: read() contract, spec plumbing, reproducibility."""

import pytest

from flatsat.hardware.drivers.sim_gyro import SimGyroDriver
from flatsat.msgs import hal_pb2


def test_read_produces_imu_sample_within_amplitude() -> None:
    driver = SimGyroDriver.from_config("imu_test", {"seed": 11, "amplitude_rad_s": 0.05})
    msg, flags = driver.read()
    assert isinstance(msg, hal_pb2.ImuSample)
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    # amplitude plus generous noise margin; the model corrupts, it does not invent
    assert abs(msg.gyro_x_rad_s) < 0.05 + 0.1


def test_from_config_reads_spec_and_options() -> None:
    driver = SimGyroDriver.from_config(
        "imu_test", {"seed": 1, "amplitude_rad_s": 0.2, "period_s": 5.0}
    )
    described = "\n".join(driver.describe())
    assert "amplitude=0.2" in described
    assert "imu0" in described  # the shared spec's provenance is echoed


def test_seeded_drivers_read_identically() -> None:
    a = SimGyroDriver.from_config("imu_a", {"seed": 42})
    b = SimGyroDriver.from_config("imu_b", {"seed": 42})
    # Same time base cannot be guaranteed across constructions, so compare
    # the noise stream indirectly: same seed, same call count, same spec.
    a._t0 = b._t0  # align the analytic truth signal  # noqa: SLF001
    ra, fa = a.read()
    rb, fb = b.read()
    assert isinstance(ra, hal_pb2.ImuSample) and isinstance(rb, hal_pb2.ImuSample)
    assert fa == fb
    assert ra.gyro_x_rad_s == pytest.approx(rb.gyro_x_rad_s)
    assert ra.gyro_y_rad_s == pytest.approx(rb.gyro_y_rad_s)
