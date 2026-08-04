"""Sun sensor model: the true sun direction -> what a coarse CSS reports.

PURE function driven by a device spec (``flatsat/hardware/devices.proto``
SunSensorDevice). The corruption is angular: perturb the true direction
by Gaussian noise per transverse axis and renormalize. Eclipse is not a
fault — the device honestly reports darkness (``sun_visible`` False),
because "no sun" is a real measurement a pointing law must respect, not
paper over.
"""

from __future__ import annotations

import math
import random

from flatsat.hardware import devices_pb2

Vec3 = tuple[float, float, float]


def apply_sun_model(
    truth_sun_body: Vec3,
    in_eclipse: bool,
    spec: devices_pb2.SunSensorDevice,
    rng: random.Random | None = None,
) -> tuple[Vec3, bool]:
    """Convert the true body-frame sun direction into a CSS reading.

    Args:
        truth_sun_body: True unit vector toward the sun, body frame.
        in_eclipse: Whether the Earth is blocking the sun.
        spec: Device specification supplying the angular noise.
        rng: Random source; defaults to the module-level generator.

    Returns:
        Tuple of ((x, y, z), sun_visible). In eclipse the vector is
        zeros and sun_visible is False.
    """
    if in_eclipse:
        return (0.0, 0.0, 0.0), False
    source = rng if rng is not None else random
    perturbed = [
        truth_sun_body[0] + source.gauss(0.0, spec.noise_rad),
        truth_sun_body[1] + source.gauss(0.0, spec.noise_rad),
        truth_sun_body[2] + source.gauss(0.0, spec.noise_rad),
    ]
    norm = math.sqrt(sum(v * v for v in perturbed))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0), False
    return (perturbed[0] / norm, perturbed[1] / norm, perturbed[2] / norm), True
