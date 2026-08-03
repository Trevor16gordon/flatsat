"""Wheel model: envelopes enforced exactly where the spec says."""

import pytest

from flatsat.core.config import load_wheel_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.models.wheel import WheelModel
from flatsat.msgs import hal_pb2


def _spec(max_torque: float = 0.05, max_momentum: float = 0.5) -> devices_pb2.WheelDevice:
    real, _prov = load_wheel_spec("config/devices/wheel0.txtpb")
    return devices_pb2.WheelDevice(
        name="test_wheel",
        max_torque_n_m=max_torque,
        max_momentum_n_m_s=max_momentum,
        rotor_inertia_kg_m2=real.rotor_inertia_kg_m2,
    )


@pytest.mark.verifies("FSW-ACT-005")
def test_torque_clips_and_flags_range() -> None:
    model = WheelModel(_spec(max_torque=0.05))
    flags = model.apply(1.0, dt_s=0.01)  # 20x the envelope
    assert flags & hal_pb2.VALIDITY_FLAG_RANGE
    assert abs(model.applied_torque_n_m) <= 0.05 + 1e-12


@pytest.mark.verifies("FSW-ACT-005")
def test_momentum_rails_and_flags_saturated() -> None:
    model = WheelModel(_spec(max_torque=10.0, max_momentum=0.01))
    for _ in range(100):
        model.apply(5.0, dt_s=0.01)
    flags = model.apply(5.0, dt_s=0.01)
    assert flags & hal_pb2.VALIDITY_FLAG_SATURATED
    assert model.saturated
    # +torque to the body stores NEGATIVE rotor momentum (reaction).
    assert model.momentum_n_m_s == pytest.approx(-0.01)
    assert model.applied_torque_n_m == 0.0, "no torque into the rail"


def test_torque_out_of_the_rail_is_allowed() -> None:
    model = WheelModel(_spec(max_torque=10.0, max_momentum=0.01))
    for _ in range(100):
        model.apply(5.0, dt_s=0.01)
    flags = model.apply(-1.0, dt_s=0.001)  # desaturating direction
    assert not flags & hal_pb2.VALIDITY_FLAG_SATURATED
    assert model.applied_torque_n_m == pytest.approx(-1.0)


def test_momentum_is_the_reaction_to_applied_torque() -> None:
    """The rotor stores MINUS the integral of body-applied torque.

    Getting this sign wrong once turned the momentum-dump law into
    positive feedback: the dump consumed this field as stored momentum
    and the wheels spun up exponentially instead of draining.
    """
    model = WheelModel(_spec(max_torque=1.0, max_momentum=100.0))
    for _ in range(10):
        model.apply(0.5, dt_s=0.01)
    assert model.momentum_n_m_s == pytest.approx(-0.5 * 0.1)


def test_state_message_reflects_the_model() -> None:
    spec = _spec(max_torque=1.0, max_momentum=100.0)
    model = WheelModel(spec)
    model.apply(0.5, dt_s=0.02)
    msg = model.state_message()
    assert msg.momentum_n_m_s == pytest.approx(-0.01)
    assert msg.speed_rad_s == pytest.approx(-0.01 / spec.rotor_inertia_kg_m2)
    assert msg.torque_n_m == pytest.approx(0.5)
    assert not msg.saturated
