"""Magnetometer model: the true field -> what the device would report.

PURE function driven by a device spec (``flatsat/hardware/devices.proto``
MagnetometerDevice), mirroring the gyro model's order of operations:

  1. additive white noise (sigma from the spec)
  2. saturation at the device's full-scale range -> SATURATED flag
  3. quantization to the output LSB

Bias, scale-factor error, axis misalignment, and hard/soft-iron effects
are deliberately absent until there is a physical part to characterize —
an invented distortion model would be fiction a B-dot gain then "learns".
"""

from __future__ import annotations

import random

from flatsat.hardware import devices_pb2
from flatsat.msgs import hal_pb2

Vec3 = tuple[float, float, float]


def apply_mag_model(
    truth_field_t: Vec3,
    spec: devices_pb2.MagnetometerDevice,
    rng: random.Random | None = None,
) -> tuple[Vec3, int]:
    """Convert the true body-frame field into what the device would report.

    Args:
        truth_field_t: True field (x, y, z) in the body frame, tesla.
        spec: Device specification supplying noise, range, and resolution.
        rng: Random source; defaults to the module-level generator. Pass a
            seeded ``random.Random`` for reproducible runs.

    Returns:
        Tuple of (measured field, validity flags). The flags carry
        ``VALIDITY_FLAG_SATURATED`` when any axis railed.
    """
    source = rng if rng is not None else random
    limit = spec.full_scale_t
    lsb = spec.lsb_t
    flags: int = int(hal_pb2.VALIDITY_FLAG_VALID)
    measured: list[float] = []
    for truth in truth_field_t:
        value = truth + source.gauss(0.0, spec.noise_t)
        if value > limit:
            value = limit
            flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        elif value < -limit:
            value = -limit
            flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        if lsb > 0.0:
            value = round(value / lsb) * lsb
        measured.append(value)
    return (measured[0], measured[1], measured[2]), flags
