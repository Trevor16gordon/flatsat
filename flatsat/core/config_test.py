"""Configuration loading: proto-schema'd, strict, provenance-carrying."""

from pathlib import Path

import pytest
from google.protobuf import text_format

from flatsat.core.config import load_imu_spec, load_vehicle, load_wheel_spec

VEHICLE = Path("config/vehicles/flatsat_v1.txtpb")


@pytest.mark.verifies("FSW-HAL-005", "FSW-CFG-001")
def test_specs_load_with_provenance() -> None:
    vehicle = load_vehicle()
    spec, prov = load_imu_spec()
    assert vehicle.control.rate_hz > 0
    assert vehicle.provenance.checksum and len(vehicle.provenance.checksum) == 12
    assert vehicle.sensor("imu0").driver == "basilisk_imu"
    assert spec.gyro_noise_rad_s > 0
    assert "imu0" in prov.path


@pytest.mark.verifies("FSW-CFG-004")
def test_misspelled_field_fails_loud(tmp_path: Path) -> None:
    """A typo'd config key is a startup failure, never a silent default."""
    tweaked = VEHICLE.read_text().replace("rate_hz: 100", "rate_hzz: 100")
    target = tmp_path / "vehicle.txtpb"
    target.write_text(tweaked)
    with pytest.raises(text_format.ParseError, match="rate_hzz"):
        load_vehicle(target)


def test_driver_selection_is_the_oneof_field() -> None:
    """The filled options block IS the driver — no separate key to disagree."""
    vehicle = load_vehicle()
    imu = vehicle.sensor("imu0")
    assert imu.driver == "basilisk_imu"
    assert imu.options.truth_topic == "sim/truth/state"
    thermal = vehicle.sensor("thermal_tj")
    assert thermal.driver == "jetson_thermal"
    assert thermal.options.zone == "tj-thermal"


def test_missing_options_block_fails_loud(tmp_path: Path) -> None:
    tweaked = VEHICLE.read_text().replace('jetson_thermal { zone: "tj-thermal" }', "")
    target = tmp_path / "vehicle.txtpb"
    target.write_text(tweaked)
    with pytest.raises(ValueError, match="no options selected"):
        load_vehicle(target)


def test_estimator_defaults_to_passthrough(tmp_path: Path) -> None:
    """A vehicle with no estimator block gets today's behavior explicitly."""
    tweaked = VEHICLE.read_text().replace("passthrough {}", "")
    target = tmp_path / "vehicle.txtpb"
    target.write_text(tweaked)
    vehicle = load_vehicle(target)
    assert vehicle.control.estimator == "passthrough"


def test_mode_and_telemetry_defaults_fill_absent_fields(tmp_path: Path) -> None:
    minimal = (
        'name: "min"\n'
        'control { rate_hz: 10 input_topic: "t/i" output_topic: "t/o" '
        "stale_after_s: 0.1 rate_damping { kp: 0.1 kd: 0.01 } }\n"
    )
    target = tmp_path / "vehicle.txtpb"
    target.write_text(minimal)
    vehicle = load_vehicle(target)
    assert vehicle.mode.ack_timeout_s == pytest.approx(2.0)
    assert vehicle.mode.clean_shutdown_marker
    assert "hal/**" in vehicle.telemetry.topics
    assert vehicle.telemetry.max_total_bytes > 0


def test_unknown_sensor_fails_loud() -> None:
    vehicle = load_vehicle()
    with pytest.raises(KeyError):
        vehicle.sensor("no_such_device")


def test_actuators_load_with_body_and_mounting() -> None:
    vehicle = load_vehicle()
    wheel = vehicle.actuator("wheel0")
    assert wheel.driver == "basilisk_reaction_wheel"
    assert wheel.stale_zero_s > 0
    assert wheel.mounting.axis == (1.0, 0.0, 0.0)
    body = vehicle.require_body()
    assert body.mass_kg > 0
    assert len(body.inertia_kg_m2) == 3


def test_mounting_axis_is_normalized(tmp_path: Path) -> None:
    tweaked = VEHICLE.read_text().replace("axis: [1.0, 0.0, 0.0]", "axis: [2.0, 0.0, 0.0]")
    target = tmp_path / "vehicle.txtpb"
    target.write_text(tweaked)
    vehicle = load_vehicle(target)
    assert vehicle.actuator("wheel0").mounting.axis == pytest.approx((1.0, 0.0, 0.0))


def test_zero_mounting_axis_fails_loud(tmp_path: Path) -> None:
    tweaked = VEHICLE.read_text().replace("axis: [1.0, 0.0, 0.0]", "axis: [0.0, 0.0, 0.0]")
    target = tmp_path / "vehicle.txtpb"
    target.write_text(tweaked)
    with pytest.raises(ValueError, match="zero vector"):
        load_vehicle(target)


def test_wheel_spec_loads_with_provenance() -> None:
    spec, prov = load_wheel_spec("config/devices/wheel0.txtpb")
    assert spec.max_torque_n_m > 0
    assert spec.max_momentum_n_m_s > 0
    assert spec.rotor_inertia_kg_m2 > 0
    assert len(prov.checksum) == 12
