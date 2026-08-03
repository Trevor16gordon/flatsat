"""B-dot law: opposes the field's rotation, quiet without a field.

Numerical checks live here; the cross-strategy contract lives in
controller_test.py and runs for bdot automatically via the registry.
"""

import math

import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeReference, AttitudeState
from flatsat.control.attitude.controllers.bdot import BdotController

DT = 0.02
DETUMBLE = AttitudeReference()


def _controller(gain: float = 1.0e7, limit: float = 25.0, tau: float = 0.0) -> BdotController:
    return BdotController(gain=gain, max_dipole_a_m2=limit, filter_tau_s=tau)


def _state(field: tuple[float, float, float], fresh: bool = True) -> AttitudeState:
    return AttitudeState(mag_field_t=field, mag_fresh=fresh)


def _rotating_field(t_s: float, omega_rad_s: float = 0.1) -> tuple[float, float, float]:
    """The field a body spinning about z sees: B rotating in the x-y plane.

    Args:
        t_s: Time since start.
        omega_rad_s: Spin rate.

    Returns:
        Body-frame field, tesla.
    """
    magnitude = 2.6e-5
    return (
        magnitude * math.cos(-omega_rad_s * t_s),
        magnitude * math.sin(-omega_rad_s * t_s),
        0.0,
    )


@pytest.mark.verifies("FSW-ADCS-011")
def test_dipole_opposes_the_field_derivative() -> None:
    """M = -k dB/dt exactly, with the filter disabled (tau = 0)."""
    controller = _controller(gain=1.0e7, limit=1.0e9, tau=0.0)
    controller.update(_state((1.0e-5, 0.0, 0.0)), DETUMBLE, DT)  # prime
    output = controller.update(_state((1.2e-5, 0.0, 0.0)), DETUMBLE, DT)
    deriv = (1.2e-5 - 1.0e-5) / DT
    assert output.dipole_a_m2[0] == pytest.approx(-1.0e7 * deriv)
    assert output.torque_n_m == (0.0, 0.0, 0.0), "a magnetic law never speaks torque"


@pytest.mark.verifies("FSW-ADCS-011")
def test_commanded_torque_damps_the_spin() -> None:
    """Against a rotating field, m x B must oppose the spin axis.

    This is the property detumble stands on: the dipole reacts to the
    field's rotation, and the resulting torque projects negatively on
    the body rate that caused it.
    """
    omega_z = 0.1
    controller = _controller(tau=0.1)
    damped_steps = 0
    total = 200
    for step in range(total):
        field = _rotating_field(step * DT, omega_z)
        output = controller.update(_state(field), DETUMBLE, DT)
        m = output.dipole_a_m2
        torque_z = m[0] * field[1] - m[1] * field[0]
        if step > 20:  # after the filter settles
            damped_steps += int(torque_z * omega_z < 0.0)
    assert damped_steps > 0.95 * (total - 21), "m x B fails to oppose the spin"


def test_quiet_without_a_field() -> None:
    controller = _controller()
    output = controller.update(AttitudeState(), DETUMBLE, DT)
    assert output.dipole_a_m2 == (0.0, 0.0, 0.0)
    assert not output.saturated


@pytest.mark.verifies("FSW-ADCS-011")
def test_stale_field_goes_quiet_not_spiky() -> None:
    """A frozen measurement must not be differentiated into phantom rate."""
    controller = _controller(tau=0.0)
    controller.update(_state((1.0e-5, 0.0, 0.0)), DETUMBLE, DT)
    controller.update(_state((1.2e-5, 0.0, 0.0)), DETUMBLE, DT)
    stale = controller.update(_state((1.2e-5, 0.0, 0.0), fresh=False), DETUMBLE, DT)
    assert stale.dipole_a_m2 == (0.0, 0.0, 0.0)
    # Recovery re-primes rather than differencing across the gap.
    resumed = controller.update(_state((5.0e-5, 0.0, 0.0)), DETUMBLE, DT)
    assert resumed.dipole_a_m2 == (0.0, 0.0, 0.0), "cross-gap difference is a phantom dB/dt"


def test_clip_reports_saturation() -> None:
    controller = _controller(gain=1.0e12, limit=25.0, tau=0.0)
    controller.update(_state((1.0e-5, 0.0, 0.0)), DETUMBLE, DT)
    output = controller.update(_state((2.0e-5, 0.0, 0.0)), DETUMBLE, DT)
    assert output.saturated
    assert abs(output.dipole_a_m2[0]) == pytest.approx(25.0)


def test_missing_gain_fails_loudly() -> None:
    with pytest.raises(ValueError, match="requires gain"):
        BdotController.from_config(control_options_pb2.BdotOptions())
