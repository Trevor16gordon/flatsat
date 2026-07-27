"""Sensor models: physics truth -> what a specific device would report.

PURE functions driven by a device spec (``flatsat.core.config.ImuSpec``), so the
simulation corrupts truth using the SAME numbers the flight-side driver
knows the hardware by. A HIL run is only meaningful if the simulated sensor
is the sensor that exists; a noise constant living in a sim script is not
that.

Modeled here, in the order a real part applies them:
  1. additive white noise (sigma from the spec)
  2. saturation at the device's full-scale range -> SATURATED flag
  3. quantization to the output LSB

Bias, scale-factor error, axis misalignment, and temperature drift are
deliberately absent until there is a physical part to characterize — an
invented bias model would be fiction that the controller then "learns".
"""

from __future__ import annotations

import random

from flatsat.core.config import ImuSpec
from flatsat.msgs import hal_pb2

Vec3 = tuple[float, float, float]


def apply_gyro_model(
    truth_rates: Vec3,
    spec: ImuSpec,
    rng: random.Random | None = None,
) -> tuple[Vec3, int]:
    """Convert true body rates into what the gyro would report.

    Args:
        truth_rates: True body rates (x, y, z) in rad/s.
        spec: Device specification supplying noise, range, and resolution.
        rng: Random source; defaults to the module-level generator. Pass a
            seeded ``random.Random`` for reproducible runs.

    Returns:
        Tuple of (measured rates, validity flags). The flags carry
        ``VALIDITY_FLAG_SATURATED`` when any axis railed — the honest signal
        that the returned value is a clipped bound, not a measurement.
    """
    source = rng if rng is not None else random
    limit = spec.gyro_full_scale_rad_s
    lsb = spec.gyro_lsb_rad_s
    flags: int = int(hal_pb2.VALIDITY_FLAG_VALID)
    measured: list[float] = []
    for truth in truth_rates:
        value = truth + source.gauss(0.0, spec.gyro_noise_rad_s)
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
