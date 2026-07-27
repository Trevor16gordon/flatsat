"""Attitude control laws — PURE functions, no I/O, no clock, no bus.

Everything here takes numbers and returns numbers. That is deliberate: a
control law with no side effects can be unit-tested numerically, swept in
batch Monte Carlo at thousands of times real time, and REPLAYED against
recorded flight telemetry to reproduce an anomaly exactly — none of which
is possible once bus calls and timestamps are welded into the math.

The loop process (``flight.adcs.loop``) is the imperative shell: it owns
subscriptions, timing, and publishing, and calls into this module for the
one thing that is actually control.
"""

from __future__ import annotations

from dataclasses import dataclass

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class RateDampingGains:
    """Gains for the PD rate-damping law.

    Attributes:
        kp: Proportional gain on body rate [N·m / (rad/s)].
        kd: Derivative gain on body rate [N·m / (rad/s²)].
    """

    kp: float
    kd: float


def rate_damping_torque(
    rates: Vec3,
    prev_rates: Vec3,
    dt_s: float,
    gains: RateDampingGains,
) -> Vec3:
    """Compute detumble torque from body rates (PD on rate).

    torque = -kp·ω - kd·dω/dt, per axis. The derivative term is skipped
    when ``dt_s`` is non-positive rather than dividing by zero — a control
    law must degrade, never raise, on a bad time step.

    Args:
        rates: Current body rates (x, y, z) in rad/s.
        prev_rates: Body rates from the previous cycle in rad/s.
        dt_s: Time between the two samples in seconds.
        gains: Proportional and derivative gains.

    Returns:
        Commanded torque (x, y, z) in N·m.
    """
    torque: list[float] = []
    for rate, prev in zip(rates, prev_rates, strict=True):
        derivative = (rate - prev) / dt_s if dt_s > 0.0 else 0.0
        torque.append(-gains.kp * rate - gains.kd * derivative)
    return (torque[0], torque[1], torque[2])
