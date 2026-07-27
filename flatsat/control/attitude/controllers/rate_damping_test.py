"""PD rate-damping law: numerical behavior pinned exactly."""

import math

import pytest

from flatsat.control.attitude.controller import (
    AttitudeReference,
    AttitudeState,
    ControlLimits,
)
from flatsat.control.attitude.controllers.rate_damping import RateDampingController

DT = 0.01
DETUMBLE = AttitudeReference()
GENEROUS = ControlLimits(max_torque_n_m=1e6)  # keep clipping out of the math tests


def pd() -> RateDampingController:
    return RateDampingController(kp=0.02, kd=0.005, limits=GENEROUS)


def state(rates: tuple[float, float, float]) -> AttitudeState:
    return AttitudeState(body_rates_rad_s=rates)


def test_zero_rate_zero_torque() -> None:
    assert pd().update(state((0.0, 0.0, 0.0)), DETUMBLE, DT).torque_n_m == (0.0, 0.0, 0.0)


def test_torque_opposes_rotation() -> None:
    torque = pd().update(state((0.1, -0.2, 0.3)), DETUMBLE, DT).torque_n_m
    assert torque[0] < 0 and torque[1] > 0 and torque[2] < 0


def test_proportional_term_exact() -> None:
    ctrl = pd()
    ctrl.update(state((0.5, 0.0, 0.0)), DETUMBLE, DT)  # prime the derivative history
    torque = ctrl.update(state((0.5, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
    assert torque[0] == pytest.approx(-0.02 * 0.5)


def test_derivative_term_exact() -> None:
    torque = pd().update(state((0.1, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
    assert torque[0] == pytest.approx(-0.02 * 0.1 - 0.005 * (0.1 / DT))


@pytest.mark.verifies("FSW-ADCS-008")
def test_reference_tracking_not_just_damping() -> None:
    """At the target rate the error is zero, so the command is zero."""
    reference = AttitudeReference(body_rates_rad_s=(0.05, 0.0, 0.0))
    ctrl = pd()
    ctrl.update(state((0.05, 0.0, 0.0)), reference, DT)
    torque = ctrl.update(state((0.05, 0.0, 0.0)), reference, DT).torque_n_m
    assert torque[0] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.verifies("FSW-ADCS-007")
def test_detumble_matches_first_order_decay() -> None:
    """Integrating a rigid axis under the PD law reproduces exp(-t/tau).

    tau = (I + kd)/kp — pins the dynamics numerically, so a gain or sign
    regression fails the suite instead of quietly changing how the
    spacecraft flies.
    """
    ctrl = pd()
    inertia = 0.9
    tau = (inertia + ctrl.kd) / ctrl.kp
    omega0 = 0.1
    omega = omega0
    steps = 10_000  # 100 s at dt = 10 ms
    for _ in range(steps):
        torque = ctrl.update(state((omega, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
        omega += (torque[0] / inertia) * DT
    assert omega == pytest.approx(omega0 * math.exp(-(steps * DT) / tau), rel=0.05)
