"""Sensor model: noise, saturation, and quantization from the shared spec."""

import math
import random
import statistics

import pytest

from flatsat.core.config import load_imu_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.models.imu import apply_gyro_model, imu_temperature
from flatsat.msgs import hal_pb2

SPEC, _PROV = load_imu_spec()


def test_noise_matches_configured_sigma() -> None:
    rng = random.Random(1234)
    truth = 0.05
    samples = [apply_gyro_model((truth, 0.0, 0.0), SPEC, rng)[0][0] for _ in range(4000)]
    assert statistics.mean(samples) == pytest.approx(truth, abs=0.0004)
    assert statistics.stdev(samples) == pytest.approx(SPEC.gyro_noise_rad_s, rel=0.15)


def test_valid_flags_when_in_range() -> None:
    _, flags = apply_gyro_model((0.01, -0.01, 0.0), SPEC, random.Random(7))
    assert flags == hal_pb2.VALIDITY_FLAG_VALID


@pytest.mark.verifies("FSW-HAL-006")
def test_saturation_clips_and_flags() -> None:
    beyond = SPEC.gyro_full_scale_rad_s * 2.0
    rates, flags = apply_gyro_model((beyond, 0.0, 0.0), SPEC, random.Random(7))
    assert flags & hal_pb2.VALIDITY_FLAG_SATURATED
    assert rates[0] == pytest.approx(SPEC.gyro_full_scale_rad_s, abs=SPEC.gyro_lsb_rad_s)


def test_negative_saturation_flags_too() -> None:
    _, flags = apply_gyro_model((-SPEC.gyro_full_scale_rad_s * 2, 0.0, 0.0), SPEC, random.Random(7))
    assert flags & hal_pb2.VALIDITY_FLAG_SATURATED


def test_output_is_quantized_to_lsb() -> None:
    rates, _ = apply_gyro_model((0.037, -0.014, 0.002), SPEC, random.Random(99))
    for value in rates:
        steps = value / SPEC.gyro_lsb_rad_s
        assert steps == pytest.approx(round(steps), abs=1e-6)


def test_seeded_rng_is_reproducible() -> None:
    a = apply_gyro_model((0.02, 0.02, 0.02), SPEC, random.Random(5))
    b = apply_gyro_model((0.02, 0.02, 0.02), SPEC, random.Random(5))
    assert a == b


def test_temperature_relaxes_toward_the_eclipse_state() -> None:
    """The die walks toward the shadowed equilibrium, first-order."""
    spec = devices_pb2.ImuDevice(
        name="thermal_test",
        temperature_c=25.0,
        temperature_sunlit_c=38.0,
        temperature_eclipse_c=4.0,
        thermal_time_constant_s=100.0,
    )
    temp = spec.temperature_c
    for _ in range(100):  # 100 s of eclipse at 1 Hz
        temp = imu_temperature(temp, spec, in_eclipse=True, dt_s=1.0)
    # One time constant: ~63% of the way from 25 toward 4.
    expected = 4.0 + (25.0 - 4.0) * math.exp(-1.0)
    assert temp == pytest.approx(expected, rel=0.01)
    for _ in range(1000):  # long sunlit stretch converges to equilibrium
        temp = imu_temperature(temp, spec, in_eclipse=False, dt_s=1.0)
    assert temp == pytest.approx(38.0, abs=0.01)


def test_zero_time_constant_disables_the_thermal_model() -> None:
    """A bench part sees no orbit: temperature stays put."""
    spec = devices_pb2.ImuDevice(name="static_test", temperature_c=25.0)
    assert imu_temperature(25.0, spec, in_eclipse=True, dt_s=10.0) == 25.0
