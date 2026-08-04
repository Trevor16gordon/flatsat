"""TRIAD estimator: exact recovery, honest degradation, MRP conversion."""

import math

import numpy as np
import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.estimators.triad import TriadEstimator, dcm_to_mrp, triad_dcm
from flatsat.msgs import hal_pb2
from flatsat.sim import orbit
from flatsat.sim.basilisk_hil import mrp_to_dcm
from flatsat.sim.orbit_config import load_orbit

ORBIT_FILE = "config/orbits/starlink_leo.txtpb"
DT = 5.0


def _imu(validity: int = hal_pb2.VALIDITY_FLAG_VALID) -> hal_pb2.ImuSample:
    msg = hal_pb2.ImuSample()
    msg.gyro_x_rad_s, msg.gyro_y_rad_s, msg.gyro_z_rad_s = 0.01, -0.02, 0.005
    msg.header.validity = validity
    return msg


def _vector_pair(
    sigma_true: tuple[float, float, float], t_s: float
) -> tuple[hal_pb2.MagnetometerSample, hal_pb2.SunSensorSample]:
    """Noiseless measurements consistent with the onboard models at ``t_s``."""
    elements, gmst, solar = load_orbit(ORBIT_FILE)
    position, _velocity = orbit.propagate_eci(elements, t_s)
    dcm = np.array(mrp_to_dcm(sigma_true))
    mag_body = dcm @ orbit.magnetic_field_eci(position, t_s, gmst)
    sun_body = dcm @ orbit.sun_direction_eci(t_s, solar)
    mag = hal_pb2.MagnetometerSample()
    mag.mag_x_t, mag.mag_y_t, mag.mag_z_t = mag_body
    sun = hal_pb2.SunSensorSample()
    sun.sun_x, sun.sun_y, sun.sun_z = sun_body
    sun.sun_visible = True
    return mag, sun


def _estimator() -> TriadEstimator:
    return TriadEstimator.from_config(control_options_pb2.TriadOptions(orbit=ORBIT_FILE))


@pytest.mark.parametrize(
    "sigma",
    [(0.0, 0.0, 0.0), (0.1, -0.2, 0.15), (0.4, 0.3, -0.5), (-0.05, 0.0, 0.7)],
)
def test_dcm_mrp_roundtrip(sigma: tuple[float, float, float]) -> None:
    recovered = dcm_to_mrp(np.array(mrp_to_dcm(sigma)))
    assert recovered == pytest.approx(sigma, abs=1e-9)


def test_triad_dcm_recovers_an_exact_rotation() -> None:
    dcm_true = np.array(mrp_to_dcm((0.2, -0.1, 0.3)))
    primary_i = np.array([0.0, 0.0, 1.0])
    secondary_i = np.array([1.0, 0.2, 0.0])
    dcm = triad_dcm(dcm_true @ primary_i, dcm_true @ secondary_i, primary_i, secondary_i)
    assert dcm is not None
    assert np.allclose(dcm, dcm_true, atol=1e-12)


def test_triad_dcm_refuses_a_degenerate_pair() -> None:
    v = np.array([0.0, 0.6, 0.8])
    assert triad_dcm(v, 2.0 * v, v, 2.0 * v) is None


@pytest.mark.verifies("FSW-ADCS-013")
def test_estimator_recovers_the_true_attitude() -> None:
    sigma_true = (0.1, -0.2, 0.15)
    mag, sun = _vector_pair(sigma_true, DT)  # onboard clock lands at DT after one update
    state = _estimator().update(_imu(), age_s=0.0, fresh=True, dt_s=DT, mag=mag, sun=sun)
    assert state.attitude_valid
    assert state.sigma_bn is not None
    est = np.array(mrp_to_dcm(state.sigma_bn))
    true = np.array(mrp_to_dcm(sigma_true))
    cos_angle = (float(np.trace(est @ true.T)) - 1.0) / 2.0
    error_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
    assert error_deg < 1e-6, f"noiseless TRIAD off by {error_deg:.2e} deg"
    assert state.body_rates_rad_s == pytest.approx((0.01, -0.02, 0.005))


@pytest.mark.verifies("FSW-ADCS-013")
def test_eclipse_invalidates_attitude_but_not_rates() -> None:
    mag, sun = _vector_pair((0.1, -0.2, 0.15), DT)
    sun.sun_visible = False
    sun.sun_x = sun.sun_y = sun.sun_z = 0.0
    state = _estimator().update(_imu(), age_s=0.0, fresh=True, dt_s=DT, mag=mag, sun=sun)
    assert not state.attitude_valid
    assert state.sigma_bn is None
    assert state.valid, "gyro rates must keep passing through in the dark"


def test_missing_magnetometer_invalidates_attitude() -> None:
    _mag, sun = _vector_pair((0.1, -0.2, 0.15), DT)
    state = _estimator().update(_imu(), age_s=0.0, fresh=True, dt_s=DT, mag=None, sun=sun)
    assert not state.attitude_valid


def test_from_config_requires_an_orbit() -> None:
    with pytest.raises(ValueError, match="orbit"):
        TriadEstimator.from_config(control_options_pb2.TriadOptions())
