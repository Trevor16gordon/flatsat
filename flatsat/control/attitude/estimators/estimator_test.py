"""Estimator contract tests — run against EVERY registered estimator."""

import pytest
from google.protobuf.message import Message

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeState
from flatsat.control.attitude.estimators.estimator import StateEstimator
from flatsat.core.registry import ESTIMATORS, get_estimator_class
from flatsat.msgs import hal_pb2

DT = 0.01

# One typed options instance per registered estimator.
ESTIMATOR_OPTIONS: dict[str, Message] = {
    "passthrough": control_options_pb2.PassthroughOptions(),
    "triad": control_options_pb2.TriadOptions(orbit="config/orbits/starlink_leo.txtpb"),
}


def _sample(rates: tuple[float, float, float], validity: int = 0) -> hal_pb2.ImuSample:
    msg = hal_pb2.ImuSample()
    msg.gyro_x_rad_s, msg.gyro_y_rad_s, msg.gyro_z_rad_s = rates
    msg.header.validity = validity
    return msg


@pytest.mark.parametrize("name", sorted(ESTIMATORS))
def test_registry_builds_every_estimator(name: str) -> None:
    estimator = get_estimator_class(name).from_config(ESTIMATOR_OPTIONS[name])
    assert isinstance(estimator, StateEstimator)
    assert estimator.describe()


@pytest.mark.parametrize("name", sorted(ESTIMATORS))
def test_every_estimator_returns_a_state(name: str) -> None:
    estimator = get_estimator_class(name).from_config(ESTIMATOR_OPTIONS[name])
    state = estimator.update(_sample((0.1, -0.2, 0.3)), age_s=0.001, fresh=True, dt_s=DT)
    assert isinstance(state, AttitudeState)
    assert state.age_s == pytest.approx(0.001)


@pytest.mark.parametrize("name", sorted(ESTIMATORS))
def test_no_estimator_trusts_a_stale_measurement(name: str) -> None:
    estimator = get_estimator_class(name).from_config(ESTIMATOR_OPTIONS[name])
    state = estimator.update(_sample((0.1, 0.0, 0.0)), age_s=1.0, fresh=False, dt_s=DT)
    assert not state.valid
