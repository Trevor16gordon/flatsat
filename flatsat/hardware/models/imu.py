"""Sensor models: physics truth -> what a specific device would report.

PURE functions driven by a device spec (``flatsat.hardware.devices.proto``), so the
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

import math
import random

from flatsat.hardware import devices_pb2
from flatsat.msgs import hal_pb2

Vec3 = tuple[float, float, float]


def imu_temperature(
    previous_c: float,
    spec: devices_pb2.ImuDevice,
    in_eclipse: bool,
    dt_s: float,
) -> float:
    """Advance the die temperature one step of the orbital thermal model.

    First-order relaxation toward a sunlit or eclipse equilibrium — the
    simplest model that makes a temperature BREATHE with the terminator
    instead of sitting at a constant, and honestly labeled as designed
    rather than characterized. A zero time constant disables it: the die
    stays at the nominal ``temperature_c`` (a bench part sees no orbit).

    Args:
        previous_c: Temperature at the previous step.
        spec: Device spec carrying the equilibria and time constant.
        in_eclipse: Whether the vehicle is currently shadowed.
        dt_s: Time since the previous step.

    Returns:
        The new die temperature in Celsius.
    """
    tau = spec.thermal_time_constant_s
    if tau <= 0.0 or dt_s <= 0.0:
        return previous_c
    target = spec.temperature_eclipse_c if in_eclipse else spec.temperature_sunlit_c
    alpha = 1.0 - math.exp(-dt_s / tau)
    return previous_c + (target - previous_c) * alpha


def apply_gyro_model(
    truth_rates: Vec3,
    spec: devices_pb2.ImuDevice,
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
