"""Momentum-dump law: wheels unchanged, rods bleed h, quiet when blind."""

import numpy as np
import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeReference, AttitudeState
from flatsat.control.attitude.controllers.momentum_dump import MomentumDumpController
from flatsat.control.attitude.controllers.rate_damping import RateDampingController

DT = 0.02
DETUMBLE = AttitudeReference()
FIELD = (0.0, 2.6e-5, 0.0)


def _controller(dump_gain: float = 0.15, max_dipole: float = 25.0) -> MomentumDumpController:
    return MomentumDumpController(
        kp=0.05, kd=0.005, max_torque_n_m=0.05, dump_gain=dump_gain, max_dipole_a_m2=max_dipole
    )


def _state(
    rates: tuple[float, float, float] = (0.0, 0.0, 0.0),
    field: tuple[float, float, float] | None = None,
    momentum: tuple[float, float, float] | None = None,
) -> AttitudeState:
    return AttitudeState(
        body_rates_rad_s=rates,
        mag_field_t=field,
        mag_fresh=field is not None,
        wheel_momentum_n_m_s=momentum,
    )


@pytest.mark.verifies("FSW-ADCS-012")
def test_torque_law_is_exactly_rate_damping() -> None:
    """Adding rods must not change how the wheels fly the vehicle."""
    combined = _controller()
    reference_law = RateDampingController(kp=0.05, kd=0.005, limits=combined.limits)
    for rates in ((0.1, -0.05, 0.02), (0.09, -0.04, 0.02)):
        ours = combined.update(_state(rates=rates), DETUMBLE, DT)
        theirs = reference_law.update(AttitudeState(body_rates_rad_s=rates), DETUMBLE, DT)
        assert ours.torque_n_m == pytest.approx(theirs.torque_n_m)


@pytest.mark.verifies("FSW-ADCS-012")
def test_dump_dipole_is_h_cross_b_over_b_squared() -> None:
    controller = _controller(dump_gain=0.15, max_dipole=1e9)
    momentum = (0.004, 0.0, 0.002)
    output = controller.update(_state(field=FIELD, momentum=momentum), DETUMBLE, DT)
    h = np.array(momentum)
    b = np.array(FIELD)
    expected = 0.15 * np.cross(h, b) / float(np.dot(b, b))
    assert output.dipole_a_m2 == pytest.approx(tuple(expected))


@pytest.mark.verifies("FSW-ADCS-012")
def test_dump_torque_opposes_the_perpendicular_momentum() -> None:
    """M x B must drain h_perp and never touch the along-B component."""
    controller = _controller(max_dipole=1e9)
    momentum = (0.004, 0.001, 0.002)  # includes a component along B (y)
    output = controller.update(_state(field=FIELD, momentum=momentum), DETUMBLE, DT)
    m = np.array(output.dipole_a_m2)
    b = np.array(FIELD)
    torque = np.cross(m, b)
    h = np.array(momentum)
    b_hat = b / np.linalg.norm(b)
    h_perp = h - np.dot(h, b_hat) * b_hat
    assert float(np.dot(torque, h_perp)) < 0.0, "external torque must oppose h_perp"
    assert float(np.dot(torque, b_hat)) == pytest.approx(0.0, abs=1e-18), (
        "m x B can never torque about the field line"
    )


@pytest.mark.verifies("FSW-ADCS-012")
def test_dump_goes_quiet_when_blind_but_torque_survives() -> None:
    """Momentum management degrades; attitude control must not."""
    controller = _controller()
    rates = (0.1, 0.0, 0.0)
    no_field = controller.update(_state(rates=rates, momentum=(0.004, 0.0, 0.0)), DETUMBLE, DT)
    assert no_field.dipole_a_m2 == (0.0, 0.0, 0.0)
    assert no_field.torque_n_m[0] < 0.0, "damping torque must survive a blind dump law"

    controller.reset()
    stale = AttitudeState(
        body_rates_rad_s=rates,
        mag_field_t=FIELD,
        mag_fresh=False,
        wheel_momentum_n_m_s=(0.004, 0.0, 0.0),
    )
    output = controller.update(stale, DETUMBLE, DT)
    assert output.dipole_a_m2 == (0.0, 0.0, 0.0)

    controller.reset()
    no_momentum = controller.update(_state(rates=rates, field=FIELD), DETUMBLE, DT)
    assert no_momentum.dipole_a_m2 == (0.0, 0.0, 0.0)


def test_dipole_clip_reports_saturation() -> None:
    controller = _controller(dump_gain=1e6, max_dipole=1.2)
    output = controller.update(_state(field=FIELD, momentum=(0.004, 0.0, 0.0)), DETUMBLE, DT)
    assert output.saturated
    assert max(abs(v) for v in output.dipole_a_m2) == pytest.approx(1.2)


def test_missing_gains_fail_loudly() -> None:
    with pytest.raises(ValueError, match="requires kp, kd and dump_gain"):
        MomentumDumpController.from_config(
            control_options_pb2.MomentumDumpOptions(kp=0.05, kd=0.005)
        )
