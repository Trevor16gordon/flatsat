"""Link service over the real bus: telemetry down, commands up, link down.

Two services on one loopback channel stand in for the spacecraft and the
ground mirror. The decisive test is the last one: with the link dead,
the spacecraft keeps working — just with stale ground data.
"""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.apps.link_service import LinkService, build_link
from flatsat.comms.phy.loopback import reset_channels
from flatsat.core.config import load_vehicle
from flatsat.msgs import hal_pb2

VEHICLE = "config/vehicles/test_comms.txtpb"
RECV_TIMEOUT_S = 5.0


@pytest.fixture(autouse=True)
def _isolate_channels() -> Iterator[None]:
    """Every test gets fresh loopback channels."""
    reset_channels()
    yield
    reset_channels()


@pytest.fixture(name="pair")
def fixture_pair() -> Iterator[tuple[LinkService, LinkService, zenoh.Session]]:
    """A flight service and a ground mirror sharing one loopback channel.

    Yields:
        Tuple of (flight service, ground service, observer session).
    """
    vehicle = load_vehicle(VEHICLE)
    flight_session = zenoh.open(zenoh.Config())
    ground_session = zenoh.open(zenoh.Config())
    topics = list(vehicle.comms.downlink_topics)
    flight = LinkService(build_link(vehicle, "flight"), flight_session, topics, app_name="link")
    ground = LinkService(
        build_link(vehicle, "ground"),
        ground_session,
        downlink_topics=[],
        republish_prefix="test/ground",
        app_name="ground_link",
    )
    time.sleep(0.5)  # discovery
    yield flight, ground, ground_session
    flight.close()
    ground.close()
    flight_session.close()
    ground_session.close()


def _sample(value: float) -> bytes:
    msg = hal_pb2.ImuSample()
    msg.header.source = "imu0"
    msg.gyro_x_rad_s = value
    payload: bytes = msg.SerializeToString()
    return payload


def _wait(predicate: object, timeout_s: float = RECV_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():  # type: ignore[operator]
            return True
        time.sleep(0.05)
    return False


@pytest.mark.verifies("FSW-LINK-006")
def test_telemetry_crosses_the_link_and_is_republished(
    pair: tuple[LinkService, LinkService, zenoh.Session],
) -> None:
    """A bus message on the spacecraft reappears on the ground bus."""
    flight, ground, ground_session = pair
    received: list[hal_pb2.ImuSample] = []
    sub = ground_session.declare_subscriber(
        "test/ground/test/comms/hal/imu0/sample",
        lambda s: received.append(hal_pb2.ImuSample.FromString(bytes(s.payload.to_bytes()))),
    )
    time.sleep(0.3)

    ground_session.put("test/comms/hal/imu0/sample", _sample(0.125))
    assert _wait(lambda: flight._link.queued > 0)  # noqa: SLF001 — asserting the queue
    flight.pump()  # transmit
    assert _wait(lambda: bool(ground.pump()) or bool(received))
    assert _wait(lambda: bool(received)), "telemetry never reached the ground bus"
    sub.undeclare()
    assert received[0].gyro_x_rad_s == pytest.approx(0.125)


@pytest.mark.verifies("FSW-LINK-003")
def test_only_allowlisted_topics_are_carried(
    pair: tuple[LinkService, LinkService, zenoh.Session],
) -> None:
    """The link carries NAMED traffic — never the whole bus."""
    flight, _ground, ground_session = pair
    ground_session.put("test/comms/not/carried", b"should stay home")
    time.sleep(0.5)
    assert flight._link.queued == 0, "unlisted topic must not enter the link"  # noqa: SLF001


@pytest.mark.verifies("FSW-LINK-007")
def test_the_spacecraft_keeps_working_with_the_link_dead(
    pair: tuple[LinkService, LinkService, zenoh.Session],
) -> None:
    """The correctness test from PLAN §6, executed.

    With the ground end closed (the link is dead), publishing continues,
    the service keeps pumping without error, and the data waits — the
    spacecraft is unaffected, the ground is merely stale.
    """
    flight, ground, ground_session = pair
    ground.close()
    ground._link._modem.close()  # noqa: SLF001 — the far end goes away entirely

    for index in range(5):
        ground_session.put("test/comms/hal/imu0/sample", _sample(index * 0.01))
    assert _wait(lambda: flight._link.queued >= 5)  # noqa: SLF001

    for _ in range(10):
        flight.pump()  # must not raise with nobody listening
    health = flight.publish_health()
    assert health.frames_sent > 0, "the flight side transmitted regardless"
    assert health.messages_delivered == 0, "and heard nothing back"


def test_health_reports_contact_and_queue(
    pair: tuple[LinkService, LinkService, zenoh.Session],
) -> None:
    flight, _ground, ground_session = pair
    ground_session.put("test/comms/hal/imu0/sample", _sample(0.5))
    assert _wait(lambda: flight._link.queued > 0)  # noqa: SLF001
    health = flight.publish_health()
    assert health.in_contact  # the test vehicle declares continuous contact
    assert health.queued_messages >= 1
