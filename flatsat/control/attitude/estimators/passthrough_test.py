"""Passthrough estimator: the measurement IS the estimate — verify exactly that."""

import pytest

from flatsat.control.attitude.estimators.passthrough import PassthroughEstimator
from flatsat.msgs import hal_pb2


def _sample(rates: tuple[float, float, float], validity: int = 0) -> hal_pb2.ImuSample:
    msg = hal_pb2.ImuSample()
    msg.gyro_x_rad_s, msg.gyro_y_rad_s, msg.gyro_z_rad_s = rates
    msg.header.validity = validity
    return msg


def test_rates_pass_through_verbatim() -> None:
    estimator = PassthroughEstimator.from_config({})
    state = estimator.update(_sample((0.1, -0.2, 0.3)), age_s=0.002, fresh=True, dt_s=0.01)
    assert state.body_rates_rad_s == pytest.approx((0.1, -0.2, 0.3))
    assert state.valid


def test_stale_measurement_invalidates_the_estimate() -> None:
    estimator = PassthroughEstimator.from_config({})
    state = estimator.update(_sample((0.1, 0.0, 0.0)), age_s=0.5, fresh=False, dt_s=0.01)
    assert not state.valid
    assert state.body_rates_rad_s == pytest.approx((0.1, 0.0, 0.0))  # rates still visible


def test_flagged_measurement_invalidates_the_estimate() -> None:
    estimator = PassthroughEstimator.from_config({})
    flagged = _sample((0.1, 0.0, 0.0), validity=int(hal_pb2.VALIDITY_FLAG_SATURATED))
    state = estimator.update(flagged, age_s=0.001, fresh=True, dt_s=0.01)
    assert not state.valid
