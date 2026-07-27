"""Control-strategy tests — no bus, no clock, no hardware.

Two layers on purpose: numerical tests pinning each strategy's behavior,
and interface tests that run against EVERY registered controller. The
latter is what keeps :class:`AttitudeController` an abstraction rather
than one implementation wearing a base class — a new strategy inherits
these contract checks automatically.
"""

import math

import pytest

from flight.adcs.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
)
from flight.adcs.controllers.pid import PidRateController
from flight.adcs.controllers.rate_damping import RateDampingController
from flight.adcs.guidance import ConstantRateReference
from flight.registry import CONTROLLERS, get_controller_class

DT = 0.01
DETUMBLE = AttitudeReference()
GENEROUS = ControlLimits(max_torque_n_m=1e6)  # keep clipping out of the math tests

# Config that satisfies every registered strategy, so the contract tests
# below stay valid as strategies are added.
UNIVERSAL_OPTIONS = {"kp": 0.02, "ki": 0.001, "kd": 0.005, "max_torque_n_m": 1e6}


def pd() -> RateDampingController:
    return RateDampingController(kp=0.02, kd=0.005, limits=GENEROUS)


def state(rates: tuple[float, float, float], valid: bool = True) -> AttitudeState:
    return AttitudeState(body_rates_rad_s=rates, valid=valid)


# --------------------------------------------------------------- PD law --


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


# ------------------------------------------------------------- PID law --


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


# ------------------------------------------- contract, every strategy --


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
def test_registry_builds_every_strategy(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS)
    assert isinstance(controller, AttitudeController)
    assert controller.describe()


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-001")
def test_every_strategy_is_quiet_at_the_reference(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS)
    output = controller.update(state((0.0, 0.0, 0.0)), DETUMBLE, DT)
    assert output.torque_n_m == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-002", "FSW-ADCS-003")
def test_every_strategy_opposes_error_and_respects_limits(name: str) -> None:
    options = {**UNIVERSAL_OPTIONS, "max_torque_n_m": 1e-4}
    controller = get_controller_class(name).from_config(options)
    output = controller.update(state((5.0, 0.0, 0.0)), DETUMBLE, DT)
    assert output.torque_n_m[0] == pytest.approx(-1e-4)
    assert output.saturated


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-004")
def test_every_strategy_survives_bad_timestep(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS)
    output = controller.update(state((0.1, 0.0, 0.0)), DETUMBLE, 0.0)
    assert all(math.isfinite(v) for v in output.torque_n_m)


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-006")
def test_reset_clears_history(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS)
    for _ in range(100):
        controller.update(state((0.3, 0.0, 0.0)), DETUMBLE, DT)
    controller.reset()
    fresh = get_controller_class(name).from_config(UNIVERSAL_OPTIONS)
    assert controller.update(state((0.1, 0.0, 0.0)), DETUMBLE, DT).torque_n_m == pytest.approx(
        fresh.update(state((0.1, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
    )


@pytest.mark.verifies("FSW-ADCS-008")
def test_guidance_supplies_the_reference() -> None:
    detumble = ConstantRateReference.from_config({})
    assert detumble.reference_at(0.0).body_rates_rad_s == (0.0, 0.0, 0.0)
    hold = ConstantRateReference.from_config({"target_rates_rad_s": [0.1, 0.0, -0.2]})
    assert hold.reference_at(123.0).body_rates_rad_s == (0.1, 0.0, -0.2)
