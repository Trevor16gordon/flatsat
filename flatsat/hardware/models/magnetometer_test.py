"""Magnetometer model: noise, rails, and quantization from the spec."""

import random

import pytest

from flatsat.hardware import devices_pb2
from flatsat.hardware.models.magnetometer import apply_mag_model
from flatsat.msgs import hal_pb2


def _spec(
    noise: float = 0.0, full_scale: float = 8e-4, lsb: float = 0.0
) -> devices_pb2.MagnetometerDevice:
    return devices_pb2.MagnetometerDevice(
        name="mag_test", rate_hz=50.0, noise_t=noise, full_scale_t=full_scale, lsb_t=lsb
    )


def test_noiseless_field_passes_through() -> None:
    measured, flags = apply_mag_model((2.6e-5, -1.0e-5, 0.0), _spec())
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert measured == pytest.approx((2.6e-5, -1.0e-5, 0.0))


def test_beyond_full_scale_rails_and_flags() -> None:
    measured, flags = apply_mag_model((1.0e-3, 0.0, -1.0e-3), _spec(full_scale=8e-4))
    assert flags & hal_pb2.VALIDITY_FLAG_SATURATED
    assert measured[0] == pytest.approx(8e-4)
    assert measured[2] == pytest.approx(-8e-4)


def test_quantization_snaps_to_the_lsb() -> None:
    measured, _ = apply_mag_model((2.55e-8, 0.0, 0.0), _spec(lsb=1e-8))
    assert measured[0] == pytest.approx(3e-8)


def test_noise_is_reproducible_with_a_seeded_rng() -> None:
    spec = _spec(noise=1.5e-8)
    first, _ = apply_mag_model((2.6e-5, 0.0, 0.0), spec, random.Random(7))
    second, _ = apply_mag_model((2.6e-5, 0.0, 0.0), spec, random.Random(7))
    assert first == second
    assert first[0] != pytest.approx(2.6e-5, abs=1e-12), "seeded noise must still be noise"
