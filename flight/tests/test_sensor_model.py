"""Tests for the shared sensor model and vehicle composition loading."""

import random
import statistics

import pytest

from flight.core.config import load_imu_spec, load_vehicle
from flight.hal.models.imu import apply_gyro_model
from flight.msgs import hal_pb2

SPEC = load_imu_spec()


@pytest.mark.verifies("FSW-HAL-005", "FSW-CFG-001")
def test_specs_load_with_provenance() -> None:
    vehicle = load_vehicle()
    assert vehicle.control.rate_hz > 0
    assert vehicle.provenance.checksum and len(vehicle.provenance.checksum) == 12
    assert vehicle.sensor("imu0").driver == "sim_gyro"
    assert SPEC.gyro_noise_rad_s > 0
    assert "imu0" in SPEC.provenance.path


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
