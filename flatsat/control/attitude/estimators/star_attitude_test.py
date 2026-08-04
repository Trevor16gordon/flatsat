"""Star attitude estimator: the ladder degrades one honest rung at a time."""

import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.estimators.star_attitude import StarAttitudeEstimator
from flatsat.control.attitude.estimators.triad_test import DT, ORBIT_FILE, _imu, _vector_pair
from flatsat.msgs import hal_pb2


def _star(sigma: tuple[float, float, float], valid: bool = True) -> hal_pb2.StarTrackerSample:
    msg = hal_pb2.StarTrackerSample()
    msg.sigma_x, msg.sigma_y, msg.sigma_z = sigma
    msg.star_valid = valid
    return msg


def _estimator(orbit: str | None = ORBIT_FILE) -> StarAttitudeEstimator:
    options = control_options_pb2.StarAttitudeOptions()
    if orbit is not None:
        options.orbit = orbit
    return StarAttitudeEstimator.from_config(options)


def test_valid_tracker_attitude_wins() -> None:
    state = _estimator().update(
        _imu(), age_s=0.0, fresh=True, dt_s=DT, star=_star((0.1, -0.2, 0.15))
    )
    assert state.attitude_valid
    assert state.sigma_bn == pytest.approx((0.1, -0.2, 0.15))


def test_blinded_tracker_falls_back_to_triad() -> None:
    sigma_true = (0.1, -0.2, 0.15)
    mag, sun = _vector_pair(sigma_true, DT)
    state = _estimator().update(
        _imu(),
        age_s=0.0,
        fresh=True,
        dt_s=DT,
        mag=mag,
        sun=sun,
        star=_star((0.9, 0.9, 0.9), valid=False),
    )
    assert state.attitude_valid, "TRIAD fallback must carry a blinded tracker"
    assert state.sigma_bn is not None
    assert state.sigma_bn == pytest.approx(sigma_true, abs=1e-6)


def test_blinded_tracker_in_eclipse_is_rates_only() -> None:
    """Bottom of the ladder: nothing can produce an attitude, say so."""
    state = _estimator().update(
        _imu(), age_s=0.0, fresh=True, dt_s=DT, star=_star((0.0, 0.0, 0.0), valid=False)
    )
    assert not state.attitude_valid
    assert state.sigma_bn is None
    assert state.valid, "gyro rates keep passing through"


def test_no_fallback_configured_degrades_straight_to_rates() -> None:
    sigma_true = (0.1, -0.2, 0.15)
    mag, sun = _vector_pair(sigma_true, DT)
    state = _estimator(orbit=None).update(
        _imu(),
        age_s=0.0,
        fresh=True,
        dt_s=DT,
        mag=mag,
        sun=sun,
        star=_star((0.0, 0.0, 0.0), valid=False),
    )
    assert not state.attitude_valid, "no orbit = no TRIAD rung, even with sun+mag present"


def test_fallback_clock_keeps_time_while_the_tracker_leads() -> None:
    """TRIAD must stay accurate after N steps of star-tracker duty."""
    estimator = _estimator()
    sigma_true = (0.1, -0.2, 0.15)
    steps = 5
    for _ in range(steps - 1):
        estimator.update(_imu(), 0.0, True, DT, star=_star(sigma_true))
    # Step N: tracker blinded; TRIAD's onboard clock must be at N*DT.
    mag, sun = _vector_pair(sigma_true, steps * DT)
    state = estimator.update(
        _imu(), 0.0, True, DT, mag=mag, sun=sun, star=_star((0.0, 0.0, 0.0), valid=False)
    )
    assert state.attitude_valid
    assert state.sigma_bn == pytest.approx(sigma_true, abs=1e-6)
