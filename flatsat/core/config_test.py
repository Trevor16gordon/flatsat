"""Configuration loading: typed, provenance-carrying, fail-loud."""

import pytest

from flatsat.core.config import load_imu_spec, load_vehicle


@pytest.mark.verifies("FSW-HAL-005", "FSW-CFG-001")
def test_specs_load_with_provenance() -> None:
    vehicle = load_vehicle()
    spec = load_imu_spec()
    assert vehicle.control.rate_hz > 0
    assert vehicle.provenance.checksum and len(vehicle.provenance.checksum) == 12
    assert vehicle.sensor("imu0").driver == "sim_gyro"
    assert spec.gyro_noise_rad_s > 0
    assert "imu0" in spec.provenance.path


def test_estimator_defaults_to_passthrough() -> None:
    """A vehicle file with no estimator key gets today's behavior explicitly."""
    vehicle = load_vehicle()
    assert vehicle.control.estimator == "passthrough"
    assert dict(vehicle.control.estimator_options) == {}


def test_unknown_sensor_fails_loud() -> None:
    vehicle = load_vehicle()
    with pytest.raises(KeyError):
        vehicle.sensor("no_such_device")
