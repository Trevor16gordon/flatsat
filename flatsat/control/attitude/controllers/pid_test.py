"""PID rate law: integral action, anti-windup, and invalid-state discipline."""

import pytest

from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
)
from flatsat.control.attitude.controllers.pid import PidRateController
from flatsat.control.attitude.controllers.rate_damping import RateDampingController

DT = 0.01
DETUMBLE = AttitudeReference()
GENEROUS = ControlLimits(max_torque_n_m=1e6)


def state(rates: tuple[float, float, float], valid: bool = True) -> AttitudeState:
    return AttitudeState(body_rates_rad_s=rates, valid=valid)


def test_pid_integral_removes_standing_error() -> None:
    """Against a constant disturbance, PID reaches zero rate where PD cannot."""
    inertia = 0.9
    disturbance = 2e-4  # N·m, constant (gravity gradient, residual dipole)

    def settle(controller: AttitudeController) -> float:
        omega = 0.0
        for _ in range(60_000):  # 600 s
            torque = controller.update(state((omega, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
            omega += ((torque[0] + disturbance) / inertia) * DT
        return abs(omega)

    pd_error = settle(RateDampingController(kp=0.02, kd=0.005, limits=GENEROUS))
    pid_error = settle(PidRateController(kp=0.02, ki=0.01, kd=0.005, limits=GENEROUS))
    assert pd_error > 1e-3
    assert pid_error < pd_error / 10.0


def test_pid_integral_is_clamped() -> None:
    ctrl = PidRateController(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5, limits=GENEROUS)
    for _ in range(10_000):
        ctrl.update(state((1.0, 0.0, 0.0)), DETUMBLE, DT)
    assert abs(ctrl.update(state((1.0, 0.0, 0.0)), DETUMBLE, DT).torque_n_m[0]) <= 0.5 + 1e-9


@pytest.mark.verifies("FSW-ADCS-005")
def test_pid_does_not_integrate_invalid_state() -> None:
    """A stale estimate must not charge the integrator and fire later."""
    ctrl = PidRateController(kp=0.0, ki=1.0, kd=0.0, limits=GENEROUS)
    for _ in range(1000):
        ctrl.update(state((1.0, 0.0, 0.0), valid=False), DETUMBLE, DT)
    assert ctrl.update(state((0.0, 0.0, 0.0)), DETUMBLE, DT).torque_n_m[0] == pytest.approx(0.0)
