"""Nadir-pointing law: computed target, knowledge gating, dump parity."""

import numpy as np
import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeReference, AttitudeState
from flatsat.control.attitude.controllers.nadir_point import NadirPointController
from flatsat.control.attitude.estimators.triad import dcm_to_mrp, mrp_to_dcm
from flatsat.sim import orbit
from flatsat.sim.orbit_config import load_orbit

ORBIT_FILE = "config/orbits/starlink_leo.txtpb"
DT = 0.01
DETUMBLE = AttitudeReference()


def _controller() -> NadirPointController:
    return NadirPointController.from_config(
        control_options_pb2.NadirPointOptions(
            orbit=ORBIT_FILE,
            point_axis=[0.0, 0.0, 1.0],
            k_align=0.001,
            kp=0.02,
            kd=0.0,
            max_torque_n_m=0.05,
            dump_gain=0.15,
            max_dipole_a_m2=1.0,
        )
    )


def _state(sigma: tuple[float, float, float] | None, valid: bool = True) -> AttitudeState:
    return AttitudeState(
        body_rates_rad_s=(0.0, 0.0, 0.0),
        valid=True,
        sigma_bn=sigma,
        attitude_valid=valid and sigma is not None,
    )


def _sigma_pointing_z_at_nadir(t_s: float) -> tuple[float, float, float]:
    """The attitude whose body +z sits exactly on nadir at time ``t_s``."""
    elements, _gmst, _solar = load_orbit(ORBIT_FILE)
    position, _velocity = orbit.propagate_eci(elements, t_s)
    nadir = -position / np.linalg.norm(position)
    # Build a body frame with +z on nadir: rows of [BN] are body axes in ECI.
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(nadir @ helper)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, nadir)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(nadir, x_axis)
    return dcm_to_mrp(np.vstack((x_axis, y_axis, nadir)))


def test_no_torque_when_already_pointing_at_earth() -> None:
    controller = _controller()
    sigma = _sigma_pointing_z_at_nadir(DT)  # onboard clock lands at DT
    output = controller.update(_state(sigma), DETUMBLE, DT)
    assert output.torque_n_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert controller.last_target_angle_deg == pytest.approx(0.0, abs=1e-6)


def test_alignment_torque_moves_the_axis_toward_nadir() -> None:
    """Misalign by a known rotation; the torque must be k_align * (a x n)."""
    controller = _controller()
    sigma_aligned = _sigma_pointing_z_at_nadir(DT)
    # Rotate the aligned attitude 90 degrees about body x: nadir moves
    # from body +z to body +y (RIGID rotation of the frame).
    quarter_x = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    sigma = dcm_to_mrp(quarter_x @ np.array(mrp_to_dcm(sigma_aligned)))
    output = controller.update(_state(sigma), DETUMBLE, DT)
    # a=+z, n=+y: a x n = -x, scaled by the gain.
    assert output.torque_n_m == pytest.approx((-0.001, 0.0, 0.0), abs=1e-9)
    assert controller.last_target_angle_deg == pytest.approx(90.0, abs=1e-6)


@pytest.mark.verifies("FSW-ADCS-015")
def test_no_attitude_knowledge_pauses_alignment() -> None:
    """Earth is computed, not sensed: no attitude means HOLD, not guess."""
    controller = _controller()
    blind = AttitudeState(
        body_rates_rad_s=(0.5, 0.0, 0.0),
        valid=True,
        sigma_bn=None,
        attitude_valid=False,
    )
    output = controller.update(blind, DETUMBLE, DT)
    assert output.torque_n_m[0] == pytest.approx(-0.02 * 0.5)  # pure rate damping
    assert output.torque_n_m[1] == pytest.approx(0.0)
    assert output.torque_n_m[2] == pytest.approx(0.0)
    assert controller.last_target_angle_deg is None


def test_dump_dipole_rides_along() -> None:
    """Same shared dump law as momentum_dump/sun_point."""
    controller = _controller()
    state = AttitudeState(
        body_rates_rad_s=(0.0, 0.0, 0.0),
        valid=True,
        mag_field_t=(0.0, 0.0, 2.0e-5),
        mag_fresh=True,
        wheel_momentum_n_m_s=(0.01, 0.0, 0.0),
    )
    output = controller.update(state, DETUMBLE, DT)
    assert output.dipole_a_m2 == pytest.approx((0.0, -1.0, 0.0))  # clipped rail
    assert output.dipole_saturated
    assert not output.torque_saturated


def test_from_config_requires_orbit_axis_and_gains() -> None:
    with pytest.raises(ValueError, match="orbit"):
        NadirPointController.from_config(
            control_options_pb2.NadirPointOptions(
                point_axis=[0.0, 0.0, 1.0], k_align=0.001, kp=0.02, kd=0.005, dump_gain=0.1
            )
        )
    with pytest.raises(ValueError, match="point_axis"):
        NadirPointController.from_config(
            control_options_pb2.NadirPointOptions(
                orbit=ORBIT_FILE, k_align=0.001, kp=0.02, kd=0.005, dump_gain=0.1
            )
        )
    with pytest.raises(ValueError, match="requires"):
        NadirPointController.from_config(
            control_options_pb2.NadirPointOptions(
                orbit=ORBIT_FILE, point_axis=[0.0, 0.0, 1.0], kp=0.02
            )
        )
