"""Basilisk magnetorquer driver: envelope + applied-dipole feedback.

No Basilisk needed — the driver's contract is with its feedback topic; a
test subscriber stands in for the plant.
"""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.drivers.basilisk_magnetorquer import (
    BasiliskMagnetorquerDriver,
    magnetorquer_dipole_topic,
)
from flatsat.msgs import hal_pb2, sim_pb2

OPTIONS = driver_options_pb2.BasiliskMagnetorquerOptions(device="config/devices/mtq0.txtpb")


@pytest.fixture(name="plant_session")
def fixture_plant_session() -> Iterator[zenoh.Session]:
    """One zenoh session standing in for the plant, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def test_driver_declares_the_dipole_command_kind() -> None:
    """The daemon's parser dispatch hangs on this attribute."""
    assert BasiliskMagnetorquerDriver.command_kind == "dipole"


def test_apply_publishes_applied_dipole_for_the_plant(plant_session: zenoh.Session) -> None:
    received: list[sim_pb2.MagnetorquerDipole] = []
    sub = plant_session.declare_subscriber(
        magnetorquer_dipole_topic("mtq_fb_test"),
        lambda s: received.append(
            sim_pb2.MagnetorquerDipole.FromString(bytes(s.payload.to_bytes()))
        ),
    )
    driver = BasiliskMagnetorquerDriver.from_config("mtq_fb_test", OPTIONS)
    try:
        time.sleep(0.5)  # discovery
        flags = driver.apply(0.8)
        assert flags == hal_pb2.VALIDITY_FLAG_VALID
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        driver.close()
        sub.undeclare()
    assert received, "applied dipole never reached the plant side"
    assert received[0].rod == "mtq_fb_test"
    assert received[0].dipole_a_m2 == pytest.approx(0.8)


@pytest.mark.verifies("FSW-ACT-007")
def test_feedback_carries_post_envelope_dipole(plant_session: zenoh.Session) -> None:
    """The plant must feel what the rod could drive, not what was asked."""
    received: list[sim_pb2.MagnetorquerDipole] = []
    sub = plant_session.declare_subscriber(
        magnetorquer_dipole_topic("mtq_clip_test"),
        lambda s: received.append(
            sim_pb2.MagnetorquerDipole.FromString(bytes(s.payload.to_bytes()))
        ),
    )
    driver = BasiliskMagnetorquerDriver.from_config("mtq_clip_test", OPTIONS)
    try:
        time.sleep(0.5)
        flags = driver.apply(50.0)  # far beyond the mtq0 dipole envelope
        assert flags & hal_pb2.VALIDITY_FLAG_RANGE
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        driver.close()
        sub.undeclare()
    assert received
    assert abs(received[0].dipole_a_m2) <= 1.2 + 1e-12, "feedback must be the CLIPPED dipole"


def test_state_reports_the_shared_model() -> None:
    driver = BasiliskMagnetorquerDriver.from_config("mtq_state_test", OPTIONS)
    try:
        driver.apply(0.5)
        msg, flags = driver.state()
    finally:
        driver.close()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert isinstance(msg, hal_pb2.MagnetorquerState)
    assert msg.dipole_a_m2 == pytest.approx(0.5)


def test_missing_device_file_fails_loudly() -> None:
    with pytest.raises(ValueError, match="requires a device file"):
        BasiliskMagnetorquerDriver.from_config(
            "mtq_bad", driver_options_pb2.BasiliskMagnetorquerOptions()
        )
