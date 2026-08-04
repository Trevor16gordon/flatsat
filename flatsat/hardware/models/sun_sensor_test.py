"""Sun sensor model: angular noise, renormalization, honest darkness."""

import math
import random

import pytest

from flatsat.hardware import devices_pb2
from flatsat.hardware.models.sun_sensor import apply_sun_model

SUN = (0.6, 0.0, 0.8)  # already unit length


def _spec(noise_rad: float = 0.0) -> devices_pb2.SunSensorDevice:
    return devices_pb2.SunSensorDevice(name="css_test", rate_hz=10.0, noise_rad=noise_rad)


def test_noiseless_direction_passes_through() -> None:
    (x, y, z), visible = apply_sun_model(SUN, in_eclipse=False, spec=_spec())
    assert visible
    assert (x, y, z) == pytest.approx(SUN)


def test_eclipse_is_darkness_not_a_fault() -> None:
    vector, visible = apply_sun_model(SUN, in_eclipse=True, spec=_spec(noise_rad=0.02))
    assert not visible
    assert vector == (0.0, 0.0, 0.0)


def test_noisy_output_stays_a_unit_vector() -> None:
    (x, y, z), visible = apply_sun_model(SUN, False, _spec(noise_rad=0.1), random.Random(3))
    assert visible
    assert math.sqrt(x * x + y * y + z * z) == pytest.approx(1.0)


def test_noise_is_reproducible_with_a_seeded_rng() -> None:
    spec = _spec(noise_rad=0.02)
    first, _ = apply_sun_model(SUN, False, spec, random.Random(7))
    second, _ = apply_sun_model(SUN, False, spec, random.Random(7))
    assert first == second
    assert first != pytest.approx(SUN), "seeded noise must still be noise"


def test_noise_perturbs_angle_at_the_spec_scale() -> None:
    """0.02 rad of per-axis noise must land near 0.02 rad of cone angle."""
    rng = random.Random(11)
    spec = _spec(noise_rad=0.02)
    angles = []
    for _ in range(400):
        (x, y, z), _visible = apply_sun_model(SUN, False, spec, rng)
        dot = max(-1.0, min(1.0, x * SUN[0] + y * SUN[1] + z * SUN[2]))
        angles.append(math.acos(dot))
    mean = sum(angles) / len(angles)
    assert 0.005 < mean < 0.06, f"mean cone error {mean:.4f} rad off the 0.02 rad spec scale"
