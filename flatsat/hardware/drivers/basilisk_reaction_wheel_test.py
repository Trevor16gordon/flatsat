"""Basilisk wheel driver: shared envelopes + applied-torque feedback.

No Basilisk needed — the driver's contract is with its feedback topic; a
test subscriber stands in for the bridge.
"""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.hardware.drivers.basilisk_reaction_wheel import (
    BasiliskReactionWheelDriver,
    wheel_torque_topic,
)
from flatsat.msgs import hal_pb2, sim_pb2

OPTIONS = {"device": "config/devices/wheel0.toml"}


@pytest.fixture(name="bridge_session")
def fixture_bridge_session() -> Iterator[zenoh.Session]:
    """One zenoh session standing in for the bridge, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def test_apply_publishes_applied_torque_for_the_bridge(bridge_session: zenoh.Session) -> None:
    received: list[sim_pb2.WheelAxisTorque] = []
    sub = bridge_session.declare_subscriber(
        wheel_torque_topic("wheel_fb_test"),
        lambda s: received.append(sim_pb2.WheelAxisTorque.FromString(bytes(s.payload.to_bytes()))),
    )
    driver = BasiliskReactionWheelDriver.from_config("wheel_fb_test", OPTIONS)
    try:
        time.sleep(0.5)  # discovery
        flags = driver.apply(0.02)
        assert flags == hal_pb2.VALIDITY_FLAG_VALID
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        driver.close()
        sub.undeclare()
    assert received, "applied torque never reached the bridge side"
    assert received[0].wheel == "wheel_fb_test"
    assert received[0].torque_n_m == pytest.approx(0.02)
    assert received[0].header.seq >= 1


def test_feedback_carries_post_envelope_torque(bridge_session: zenoh.Session) -> None:
    """The sim must feel what the device could do, not what was asked."""
    received: list[sim_pb2.WheelAxisTorque] = []
    sub = bridge_session.declare_subscriber(
        wheel_torque_topic("wheel_clip_test"),
        lambda s: received.append(sim_pb2.WheelAxisTorque.FromString(bytes(s.payload.to_bytes()))),
    )
    driver = BasiliskReactionWheelDriver.from_config("wheel_clip_test", OPTIONS)
    try:
        time.sleep(0.5)
        flags = driver.apply(1.0)  # 20x the wheel0 torque envelope
        assert flags & hal_pb2.VALIDITY_FLAG_RANGE
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        driver.close()
        sub.undeclare()
    assert received
    assert abs(received[0].torque_n_m) <= 0.05 + 1e-12, "feedback must be the CLIPPED torque"


def test_state_reports_the_shared_model() -> None:
    driver = BasiliskReactionWheelDriver.from_config("wheel_state_test", OPTIONS)
    try:
        driver.apply(0.03)
        time.sleep(0.02)
        driver.apply(0.03)
        msg, flags = driver.state()
    finally:
        driver.close()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert isinstance(msg, hal_pb2.WheelState)
    assert msg.torque_n_m == pytest.approx(0.03)
    assert msg.momentum_n_m_s > 0.0
