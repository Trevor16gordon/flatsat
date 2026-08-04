"""Star tracker model: noise anisotropy, exclusion blinding, honesty."""

import math
import random

import numpy as np
import pytest

from flatsat.control.attitude.estimators.triad import mrp_to_dcm
from flatsat.hardware import devices_pb2
from flatsat.hardware.models.star_tracker import apply_star_model

SIGMA_TRUE = (0.1, -0.2, 0.15)
DARK: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _spec(
    cross: float = 0.0,
    roll: float = 0.0,
    sun_excl: float = 30.0,
    earth_excl: float = 25.0,
) -> devices_pb2.StarTrackerDevice:
    return devices_pb2.StarTrackerDevice(
        name="st_test",
        rate_hz=5.0,
        cross_noise_rad=cross,
        roll_noise_rad=roll,
        boresight=[0.0, 0.0, -1.0],
        sun_exclusion_deg=sun_excl,
        earth_exclusion_deg=earth_excl,
    )


def _error_deg(sigma_a: tuple[float, float, float], sigma_b: tuple[float, float, float]) -> float:
    a = np.array(mrp_to_dcm(sigma_a))
    b = np.array(mrp_to_dcm(sigma_b))
    cos_angle = (float(np.trace(a @ b.T)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


def test_noiseless_attitude_passes_through() -> None:
    measured, valid = apply_star_model(SIGMA_TRUE, DARK, DARK, _spec())
    assert valid
    assert measured == pytest.approx(SIGMA_TRUE)


def test_eclipse_is_the_trackers_best_friend() -> None:
    """No sun anywhere (zeros) must NOT blind — darkness means stars."""
    _measured, valid = apply_star_model(SIGMA_TRUE, DARK, DARK, _spec())
    assert valid


def test_sun_in_the_exclusion_cone_blinds() -> None:
    # Boresight -z; sun 10 degrees off it, inside the 30 degree cone.
    sun = (math.sin(math.radians(10.0)), 0.0, -math.cos(math.radians(10.0)))
    measured, valid = apply_star_model(SIGMA_TRUE, sun, DARK, _spec())
    assert not valid
    assert measured == (0.0, 0.0, 0.0)


def test_earth_in_the_exclusion_cone_blinds() -> None:
    nadir = (0.0, math.sin(math.radians(5.0)), -math.cos(math.radians(5.0)))
    _measured, valid = apply_star_model(SIGMA_TRUE, DARK, nadir, _spec())
    assert not valid


def test_sun_outside_the_cone_does_not_blind() -> None:
    sun = (1.0, 0.0, 0.0)  # 90 degrees off the -z boresight
    _measured, valid = apply_star_model(SIGMA_TRUE, sun, DARK, _spec())
    assert valid


def test_noise_scale_matches_the_spec() -> None:
    """Errors land at the arcsecond scale the spec asks for, not degrees."""
    rng = random.Random(5)
    spec = _spec(cross=0.00003, roll=0.0002)
    errors = [
        _error_deg(apply_star_model(SIGMA_TRUE, DARK, DARK, spec, rng)[0], SIGMA_TRUE)
        for _ in range(200)
    ]
    mean_deg = sum(errors) / len(errors)
    # Roll dominates at ~0.0115 deg 1-sigma; the mean total error must
    # sit near that scale — far below TRIAD's ~1 degree class.
    assert 0.001 < mean_deg < 0.1, f"mean error {mean_deg:.4f} deg off the spec scale"


def test_noise_is_reproducible_with_a_seeded_rng() -> None:
    spec = _spec(cross=0.00003, roll=0.0002)
    first, _ = apply_star_model(SIGMA_TRUE, DARK, DARK, spec, random.Random(7))
    second, _ = apply_star_model(SIGMA_TRUE, DARK, DARK, spec, random.Random(7))
    assert first == second
    assert first != pytest.approx(SIGMA_TRUE, abs=1e-9), "seeded noise must still be noise"
