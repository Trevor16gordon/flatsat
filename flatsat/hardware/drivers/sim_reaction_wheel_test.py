"""Local wheel fake: wall-clock integration over the shared model.

Envelope semantics (clip+RANGE, momentum rail+SATURATED) are pinned in
``flatsat/hardware/models/wheel_test.py`` — the shared model both wheel
drivers run. These tests cover what THIS driver adds: the wall-clock time
base and config plumbing.
"""

import time

import pytest

from flatsat.core.config import load_wheel_spec
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.drivers.sim_reaction_wheel import SimReactionWheelDriver
from flatsat.msgs import hal_pb2

OPTIONS = driver_options_pb2.SimReactionWheelOptions(device="config/devices/wheel0.txtpb")


def test_from_config_reads_device_spec() -> None:
    driver = SimReactionWheelDriver.from_config("wheel0", OPTIONS)
    assert "wheel0.txtpb" in "\n".join(driver.describe())


def test_momentum_integrates_torque_against_the_wall_clock() -> None:
    driver = SimReactionWheelDriver.from_config("wheel0", OPTIONS)
    driver.apply(0.04)  # first call establishes the time base
    time.sleep(0.05)
    driver.apply(0.04)
    msg, flags = driver.state()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert isinstance(msg, hal_pb2.WheelState)
    # ~0.04 N·m to the body for ~50 ms → the rotor stores the REACTION,
    # ~-2e-3 N·m·s; generous margin for sleep jitter.
    assert -2e-2 < msg.momentum_n_m_s < -5e-4
    spec, _prov = load_wheel_spec("config/devices/wheel0.txtpb")
    assert msg.speed_rad_s == pytest.approx(msg.momentum_n_m_s / spec.rotor_inertia_kg_m2)


def test_first_apply_moves_no_momentum() -> None:
    """With no time base yet, the first command integrates over zero time."""
    driver = SimReactionWheelDriver.from_config("wheel0", OPTIONS)
    driver.apply(0.05)
    msg, _ = driver.state()
    assert isinstance(msg, hal_pb2.WheelState)
    assert msg.momentum_n_m_s == 0.0


def test_state_is_always_publishable() -> None:
    driver = SimReactionWheelDriver.from_config("wheel0", OPTIONS)
    msg, flags = driver.state()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert msg.SerializeToString() is not None
