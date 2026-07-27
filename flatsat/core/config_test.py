"""Configuration loading: typed, provenance-carrying, fail-loud."""

from pathlib import Path

import pytest

from flatsat.core.config import load_imu_spec, load_vehicle, load_wheel_spec


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


def test_actuators_load_with_body_and_mounting() -> None:
    """Commit 2's config surface: [body], [[actuators]], mounting."""
    vehicle = load_vehicle()
    wheel = vehicle.actuator("wheel0")
    assert wheel.driver == "sim_reaction_wheel"
    assert wheel.stale_zero_s > 0
    assert wheel.mounting.axis == (1.0, 0.0, 0.0)
    body = vehicle.require_body()
    assert body.mass_kg > 0
    assert len(body.inertia_kg_m2) == 3


def test_mounting_axis_is_normalized(tmp_path: Path) -> None:
    source = Path("config/vehicles/flatsat_v1.toml").read_text()
    tweaked = source.replace("axis = [1.0, 0.0, 0.0]", "axis = [2.0, 0.0, 0.0]")
    target = tmp_path / "vehicle.toml"
    target.write_text(tweaked)
    vehicle = load_vehicle(target)
    assert vehicle.actuator("wheel0").mounting.axis == pytest.approx((1.0, 0.0, 0.0))


def test_zero_mounting_axis_fails_loud(tmp_path: Path) -> None:
    source = Path("config/vehicles/flatsat_v1.toml").read_text()
    tweaked = source.replace("axis = [1.0, 0.0, 0.0]", "axis = [0.0, 0.0, 0.0]")
    target = tmp_path / "vehicle.toml"
    target.write_text(tweaked)
    with pytest.raises(ValueError, match="zero vector"):
        load_vehicle(target)


def test_wheel_spec_loads_with_provenance() -> None:
    spec = load_wheel_spec("config/devices/wheel0.toml")
    assert spec.max_torque_n_m > 0
    assert spec.max_momentum_n_m_s > 0
    assert spec.rotor_inertia_kg_m2 > 0
    assert len(spec.provenance.checksum) == 12
