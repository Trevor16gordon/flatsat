"""C1 end to end: a file crosses the link, stages, activates, rolls back.

The whole deployment path in one process pair — ground publishes an
artifact, it is segmented, framed, carried by the (lossy-capable) PHY,
reassembled, verified, staged, and only then activated by an explicitly
authorized command. This is the test that says "we can reprogram this
spacecraft over the radio, safely".
"""

import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import zenoh

from flatsat.apps.link_service import LinkService, build_link
from flatsat.apps.uplink_send import send_artifact, send_control
from flatsat.apps.uplink_service import UplinkService
from flatsat.comms.phy.loopback import reset_channels
from flatsat.comms.slots import SlotManager
from flatsat.comms.uplink import UplinkReceiver
from flatsat.core.config import load_vehicle
from flatsat.msgs import uplink_pb2

VEHICLE = "config/vehicles/test_uplink.txtpb"
ARTIFACT = bytes(range(256)) * 30  # ~7.7 KiB "model"
RECV_TIMEOUT_S = 10.0


@pytest.fixture(autouse=True)
def _isolate_channels() -> Iterator[None]:
    """Every test gets fresh loopback channels."""
    reset_channels()
    yield
    reset_channels()


@pytest.fixture(name="rig")
def fixture_rig(tmp_path: Path) -> Iterator[tuple[UplinkService, zenoh.Session, object]]:
    """Ground bus -> link -> flight bus -> uplink service.

    Yields:
        Tuple of (uplink service, ground session, pump callable).
    """
    vehicle = load_vehicle(VEHICLE)
    ground_session = zenoh.open(zenoh.Config())
    flight_session = zenoh.open(zenoh.Config())

    # Ground link carries the artifact topics up; flight link republishes
    # them onto the flight bus, where the uplink service is listening.
    ground_link = LinkService(
        build_link(vehicle, "ground"),
        ground_session,
        downlink_topics=list(vehicle.comms.downlink_topics),
        app_name="ground_link",
    )
    flight_link = LinkService(
        build_link(vehicle, "flight"),
        flight_session,
        downlink_topics=[],
        republish_prefix="flight",  # arrival over the air lands here, and only here
        app_name="flight_link",
    )
    # The flight service listens ONLY under the "flight/" prefix, which
    # nothing but the flight link's republication ever writes. Both
    # sessions are peers on this host, so without that separation the
    # ground's publications would reach the service directly and the
    # test would prove nothing about the link.
    service = UplinkService(
        UplinkReceiver(tmp_path / "staging"),
        SlotManager(tmp_path / "slots"),
        flight_session,
        topic_prefix="flight",
    )
    time.sleep(0.5)  # discovery

    def pump(seconds: float = 2.0) -> None:
        """Run both link ends for a while (the 'pass')."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ground_link.pump()
            flight_link.pump()
            time.sleep(0.02)

    yield service, ground_session, pump
    service.close()
    ground_link.close()
    flight_link.close()
    ground_session.close()
    flight_session.close()


@pytest.mark.verifies("FSW-UPL-001", "FSW-UPL-007")
def test_artifact_crosses_the_link_and_stages(
    rig: tuple[UplinkService, zenoh.Session, object],
) -> None:
    """A model uplinked over the link arrives byte-exact — and inert."""
    service, ground_session, pump = rig
    send_artifact(
        ground_session,
        "ml_policy",
        "2026-07-28a",
        ARTIFACT,
        uplink_pb2.ARTIFACT_KIND_MODEL,
        chunk_bytes=1024,
    )
    pump(3.0)  # type: ignore[operator]

    status = service.publish_status()
    assert "ml_policy@2026-07-28a" in list(status.staged), service.last_event
    staged = service._receiver.staged_path("ml_policy", "2026-07-28a")  # noqa: SLF001
    assert staged is not None and staged.read_bytes() == ARTIFACT
    assert list(status.slots) == [], "arrival must NOT activate anything"


@pytest.mark.verifies("FSW-UPL-003", "FSW-UPL-006")
def test_activate_then_rollback_over_the_link(
    rig: tuple[UplinkService, zenoh.Session, object],
) -> None:
    """The full C1 traversal: two versions up, activate, regress, revert."""
    service, ground_session, pump = rig
    for version, payload in (("v1", b"known good weights"), ("v2", b"regressing weights")):
        send_artifact(
            ground_session,
            "ml_policy",
            version,
            payload,
            uplink_pb2.ARTIFACT_KIND_MODEL,
            chunk_bytes=512,
        )
        pump(1.5)  # type: ignore[operator]
    assert len(service.publish_status().staged) == 2, service.last_event

    for version in ("v1", "v2"):
        send_control(
            ground_session,
            uplink_pb2.ArtifactControl.ACTION_ACTIVATE,
            "ml_policy",
            version,
            ground_authority=True,
            reason="test activation",
        )
        pump(1.5)  # type: ignore[operator]
    slots = service._slots  # noqa: SLF001
    live = slots.active_path("ml_policy")
    assert live is not None and live.read_bytes() == b"regressing weights"

    send_control(
        ground_session,
        uplink_pb2.ArtifactControl.ACTION_ROLLBACK,
        "ml_policy",
        "",
        ground_authority=False,  # rollback needs none
        reason="regression detected",
    )
    pump(1.5)  # type: ignore[operator]
    live = slots.active_path("ml_policy")
    assert live is not None and live.read_bytes() == b"known good weights"


@pytest.mark.verifies("FSW-UPL-003")
def test_unauthorized_activation_is_refused_over_the_link(
    rig: tuple[UplinkService, zenoh.Session, object],
) -> None:
    """Bytes arriving over RF do not deploy themselves."""
    service, ground_session, pump = rig
    send_artifact(ground_session, "ml_policy", "v1", b"weights", uplink_pb2.ARTIFACT_KIND_MODEL)
    pump(1.5)  # type: ignore[operator]
    send_control(
        ground_session,
        uplink_pb2.ArtifactControl.ACTION_ACTIVATE,
        "ml_policy",
        "v1",
        ground_authority=False,
        reason="sneaky",
    )
    pump(1.5)  # type: ignore[operator]

    status = service.publish_status()
    assert status.refused_activations >= 1, service.last_event
    assert service._slots.active_path("ml_policy") is None  # noqa: SLF001
    assert list(status.slots) == [], "nothing may go live without authority"
