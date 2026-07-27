"""Simulated reaction wheel: envelopes enforced exactly where the spec says."""

import time

import pytest

from flatsat.core.config import WheelSpec, load_wheel_spec
from flatsat.hardware.drivers.sim_reaction_wheel import SimReactionWheelDriver
from flatsat.msgs import hal_pb2


def _spec(max_torque: float = 0.05, max_momentum: float = 0.5) -> WheelSpec:
    real = load_wheel_spec("config/devices/wheel0.toml")
    return WheelSpec(
        name="test_wheel",
        max_torque_n_m=max_torque,
        max_momentum_n_m_s=max_momentum,
        rotor_inertia_kg_m2=real.rotor_inertia_kg_m2,
        provenance=real.provenance,
    )


def test_from_config_reads_device_spec() -> None:
    driver = SimReactionWheelDriver.from_config("wheel0", {"device": "config/devices/wheel0.toml"})
    assert "wheel0.toml" in "\n".join(driver.describe())


@pytest.mark.verifies("FSW-ACT-005")
def test_torque_clips_and_flags_range() -> None:
    driver = SimReactionWheelDriver(_spec(max_torque=0.05))
    flags = driver.apply(1.0)  # 20x the envelope
    assert flags & hal_pb2.VALIDITY_FLAG_RANGE
    msg, _ = driver.state()
    assert isinstance(msg, hal_pb2.WheelState)
    assert abs(msg.torque_n_m) <= 0.05 + 1e-12


@pytest.mark.verifies("FSW-ACT-005")
def test_momentum_rails_and_flags_saturated() -> None:
    driver = SimReactionWheelDriver(_spec(max_torque=10.0, max_momentum=0.01))
    driver.apply(5.0)  # prime the wall-clock integrator
    time.sleep(0.02)
    for _ in range(50):  # integrate hard into the rail
        driver.apply(5.0)
        time.sleep(0.001)
    flags = driver.apply(5.0)
    assert flags & hal_pb2.VALIDITY_FLAG_SATURATED
    msg, _ = driver.state()
    assert isinstance(msg, hal_pb2.WheelState)
    assert msg.saturated
    assert msg.momentum_n_m_s == pytest.approx(0.01)
    assert msg.torque_n_m == 0.0, "no torque into the rail"


def test_torque_out_of_the_rail_is_allowed() -> None:
    driver = SimReactionWheelDriver(_spec(max_torque=10.0, max_momentum=0.01))
    driver.apply(5.0)
    time.sleep(0.02)
    for _ in range(50):
        driver.apply(5.0)
        time.sleep(0.001)
    flags = driver.apply(-1.0)  # desaturating direction
    assert not flags & hal_pb2.VALIDITY_FLAG_SATURATED
    msg, _ = driver.state()
    assert isinstance(msg, hal_pb2.WheelState)
    assert msg.torque_n_m == pytest.approx(-1.0)


def test_momentum_integrates_torque_over_time() -> None:
    driver = SimReactionWheelDriver(_spec(max_torque=1.0, max_momentum=100.0))
    driver.apply(0.5)  # first call establishes the time base
    time.sleep(0.05)
    driver.apply(0.5)
    msg, _ = driver.state()
    assert isinstance(msg, hal_pb2.WheelState)
    # ~0.5 N·m for ~50 ms → ~0.025 N·m·s; generous margin for sleep jitter
    assert 0.01 < msg.momentum_n_m_s < 0.2
    assert msg.speed_rad_s == pytest.approx(
        msg.momentum_n_m_s / load_wheel_spec("config/devices/wheel0.toml").rotor_inertia_kg_m2
    )


def test_state_is_always_publishable() -> None:
    driver = SimReactionWheelDriver(_spec())
    msg, flags = driver.state()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert msg.SerializeToString() is not None
