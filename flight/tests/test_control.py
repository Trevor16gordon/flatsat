"""Numerical tests for the pure control law — no bus, no clock, no hardware."""

import math

import pytest

from flight.adcs.control import RateDampingGains, rate_damping_torque

GAINS = RateDampingGains(kp=0.02, kd=0.005)
DT = 0.01


def test_zero_rate_zero_torque() -> None:
    assert rate_damping_torque((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), DT, GAINS) == (0.0, 0.0, 0.0)


def test_torque_opposes_rotation() -> None:
    torque = rate_damping_torque((0.1, -0.2, 0.3), (0.1, -0.2, 0.3), DT, GAINS)
    assert torque[0] < 0 and torque[1] > 0 and torque[2] < 0


def test_proportional_term_exact() -> None:
    # Steady rate: derivative term vanishes, leaving exactly -kp * omega.
    torque = rate_damping_torque((0.5, 0.0, 0.0), (0.5, 0.0, 0.0), DT, GAINS)
    assert torque[0] == pytest.approx(-0.02 * 0.5)


def test_derivative_term_exact() -> None:
    # Rate stepped 0 -> 0.1 over dt: -kp*0.1 - kd*(0.1/dt).
    torque = rate_damping_torque((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), DT, GAINS)
    assert torque[0] == pytest.approx(-0.02 * 0.1 - 0.005 * (0.1 / DT))


def test_nonpositive_dt_degrades_instead_of_raising() -> None:
    # A bad time step must not divide by zero — the P term still applies.
    torque = rate_damping_torque((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, GAINS)
    assert torque[0] == pytest.approx(-0.02 * 0.1)


def test_scales_linearly_with_gain() -> None:
    weak = rate_damping_torque((0.3, 0.0, 0.0), (0.3, 0.0, 0.0), DT, RateDampingGains(0.01, 0.0))
    strong = rate_damping_torque((0.3, 0.0, 0.0), (0.3, 0.0, 0.0), DT, RateDampingGains(0.02, 0.0))
    assert strong[0] == pytest.approx(2.0 * weak[0])


def test_detumble_matches_first_order_decay() -> None:
    """Integrating a rigid axis under this law reproduces exp(-t/tau).

    With I·dω/dt = -kp·ω - kd·dω/dt the closed-form time constant is
    tau = (I + kd)/kp — here 45.25 s. This pins the law's dynamics
    numerically, so a gain or sign regression fails the suite instead of
    quietly changing how the spacecraft flies.
    """
    inertia = 0.9
    tau = (inertia + GAINS.kd) / GAINS.kp
    omega0 = 0.1
    omega = omega0
    prev = omega
    steps = 10_000  # 100 s at dt = 10 ms
    for _ in range(steps):
        torque = rate_damping_torque((omega, 0.0, 0.0), (prev, 0.0, 0.0), DT, GAINS)
        prev = omega
        omega += (torque[0] / inertia) * DT

    expected = omega0 * math.exp(-(steps * DT) / tau)
    assert omega == pytest.approx(expected, rel=0.05)
    assert 0.0 < omega < omega0
