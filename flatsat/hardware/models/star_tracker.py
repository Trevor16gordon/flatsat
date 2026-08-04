"""Star tracker model: the true attitude -> what a real tracker reports.

PURE function driven by a device spec (``flatsat/hardware/devices.proto``
StarTrackerDevice), at the fidelity GNC stacks actually use: the vendor
owns photons-to-quaternion, so the model corrupts the ATTITUDE — small
rotation noise, anisotropic like real devices (roll about the boresight
is 5-10x worse than cross-boresight) — and enforces the blackout rules:
sun or Earth inside the exclusion cone means BLINDED, which is a real
operating condition (``star_valid`` False), never a device fault.
"""

from __future__ import annotations

import math
import random

import numpy as np

from flatsat.control.attitude.estimators.triad import dcm_to_mrp, mrp_to_dcm
from flatsat.hardware import devices_pb2

Vec3 = tuple[float, float, float]


def _unit(vector: Vec3) -> np.ndarray | None:
    """Normalize, or None for a zero vector.

    Args:
        vector: Any 3-vector.

    Returns:
        The unit vector, or None when there is nothing to normalize.
    """
    arr = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(arr))
    return arr / norm if norm > 0.0 else None


def apply_star_model(
    truth_sigma: Vec3,
    sun_body: Vec3,
    nadir_body: Vec3,
    spec: devices_pb2.StarTrackerDevice,
    rng: random.Random | None = None,
) -> tuple[Vec3, bool]:
    """Convert the true attitude into a star tracker reading.

    Args:
        truth_sigma: True attitude MRP (sigma_BN).
        sun_body: True body-frame sun direction; zeros = eclipse (a
            tracker LIKES the dark — no sun blinding then).
        nadir_body: True body-frame direction to Earth's center; zeros =
            no orbit (no Earth to blind on).
        spec: Device specification (noise, boresight, exclusion cones).
        rng: Random source; defaults to the module-level generator.

    Returns:
        Tuple of ((sigma_x, sigma_y, sigma_z), star_valid). A blinded
        tracker reports zeros and False — no stars, no attitude.
    """
    boresight = _unit(
        (spec.boresight[0], spec.boresight[1], spec.boresight[2])
        if len(spec.boresight) == 3
        else (0.0, 0.0, -1.0)
    )
    assert boresight is not None  # a zero boresight fails validation upstream
    for direction, cone_deg in (
        (_unit(sun_body), spec.sun_exclusion_deg),
        (_unit(nadir_body), spec.earth_exclusion_deg),
    ):
        if direction is None or cone_deg <= 0.0:
            continue
        if float(boresight @ direction) > math.cos(math.radians(cone_deg)):
            return (0.0, 0.0, 0.0), False

    source = rng if rng is not None else random
    # Anisotropic small-rotation error: roll about the boresight, cross
    # about two axes perpendicular to it.
    perp1 = _unit(tuple(np.cross(boresight, [1.0, 0.0, 0.0])))
    if perp1 is None or float(np.linalg.norm(perp1)) < 1e-9:
        perp1 = _unit(tuple(np.cross(boresight, [0.0, 1.0, 0.0])))
    assert perp1 is not None
    perp2 = np.cross(boresight, perp1)
    error_rotation = (
        source.gauss(0.0, spec.roll_noise_rad) * boresight
        + source.gauss(0.0, spec.cross_noise_rad) * perp1
        + source.gauss(0.0, spec.cross_noise_rad) * perp2
    )
    angle = float(np.linalg.norm(error_rotation))
    if angle == 0.0:
        return truth_sigma, True
    axis = error_rotation / angle
    # Rodrigues rotation for the small error DCM, applied to truth.
    tilde = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    error_dcm = np.eye(3) + math.sin(angle) * tilde + (1.0 - math.cos(angle)) * tilde @ tilde
    measured = dcm_to_mrp(error_dcm @ mrp_to_dcm(truth_sigma))
    return measured, True
