"""Mode manager + client over the real bus: distribution and boot policy."""

import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import zenoh

from flatsat.mode import mode_config_pb2
from flatsat.mode.client import ModeClient
from flatsat.mode.manager import ModeManager, request_topic
from flatsat.msgs import mode_pb2

RECV_TIMEOUT_S = 5.0
# Test-only key base: the production sys/mode would read (and command!)
# a live mode manager on this box.
BASE = "test/sys/mode"


def _entry(
    tmp_path: Path, apps: tuple[str, ...] = (), ack_timeout_s: float = 0.5
) -> mode_config_pb2.ModeConfig:
    return mode_config_pb2.ModeConfig(
        apps=list(apps),
        ack_timeout_s=ack_timeout_s,
        min_dwell_s=0.05,
        clean_shutdown_marker=str(tmp_path / "clean-shutdown"),
    )


def _wait_for(predicate: object, timeout_s: float = RECV_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():  # type: ignore[operator]
            return True
        time.sleep(0.02)
    return False


@pytest.fixture(name="sessions")
def fixture_sessions() -> Iterator[tuple[zenoh.Session, zenoh.Session]]:
    """Two zenoh sessions (manager side, client side), closed on teardown.

    Yields:
        Tuple of (manager session, client session).
    """
    a = zenoh.open(zenoh.Config())
    b = zenoh.open(zenoh.Config())
    yield a, b
    a.close()
    b.close()


@pytest.mark.verifies("FSW-MODE-007")
def test_unexpected_reset_boots_into_safe(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, _ = sessions
    manager = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)  # no marker exists
    state = manager.state()
    manager.close()
    assert state.mode == mode_pb2.SYSTEM_MODE_SAFE
    assert "unexpected reset" in state.reason


@pytest.mark.verifies("FSW-MODE-007")
def test_clean_shutdown_marker_allows_nominal_boot(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, _ = sessions
    first = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)
    first.shutdown_clean()
    first.close()

    second = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)
    state = second.state()
    second.close()
    assert state.mode == mode_pb2.SYSTEM_MODE_NOMINAL
    # The marker was consumed: a crash now means the NEXT boot is Safe.
    assert not (tmp_path / "clean-shutdown").exists()


@pytest.mark.verifies("FSW-MODE-001")
def test_late_joiner_learns_mode_by_query(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, client_session = sessions
    manager = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)
    time.sleep(0.5)  # discovery

    client = ModeClient(client_session, "late_app", base_topic=BASE)
    got = _wait_for(lambda: client.current is not None)
    client.close()
    manager.close()
    assert got, "late joiner never learned the mode"
    assert client.current is not None
    assert client.current.mode == mode_pb2.SYSTEM_MODE_SAFE


@pytest.mark.verifies("FSW-MODE-005")
def test_transitions_broadcast_with_monotonic_seq(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, client_session = sessions
    manager = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)
    time.sleep(0.5)

    seen: list[mode_pb2.ModeState] = []
    client = ModeClient(client_session, "watcher", on_mode=seen.append, base_topic=BASE)
    _wait_for(lambda: seen)  # late-join answer
    baseline = len(seen)

    time.sleep(0.1)  # dwell
    decision = manager.request(
        mode_pb2.ModeRequest(
            source="ground",
            requested=mode_pb2.SYSTEM_MODE_RECOVERY,
            reason="checkout",
            ground_authority=True,
        )
    )
    assert decision.accepted
    assert _wait_for(lambda: len(seen) > baseline), "transition never broadcast"
    client.close()
    manager.close()

    assert seen[-1].mode == mode_pb2.SYSTEM_MODE_RECOVERY
    assert seen[-1].mode_seq == seen[0].mode_seq + 1
    assert "ground" in seen[-1].reason


@pytest.mark.verifies("FSW-MODE-003")
def test_bus_requests_respect_ground_authority(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, client_session = sessions
    manager = ModeManager(_entry(tmp_path), manager_session, base_topic=BASE)
    time.sleep(0.5)

    pub = client_session.declare_publisher(request_topic(BASE))
    unauthorized = mode_pb2.ModeRequest(
        source="rogue", requested=mode_pb2.SYSTEM_MODE_RECOVERY, reason="no"
    )
    pub.put(unauthorized.SerializeToString())
    time.sleep(0.5)
    assert manager.state().mode == mode_pb2.SYSTEM_MODE_SAFE, (
        "away-from-safety honored without ground authority"
    )

    authorized = mode_pb2.ModeRequest(
        source="ground",
        requested=mode_pb2.SYSTEM_MODE_RECOVERY,
        reason="checkout",
        ground_authority=True,
    )
    pub.put(authorized.SerializeToString())
    got = _wait_for(lambda: manager.state().mode == mode_pb2.SYSTEM_MODE_RECOVERY)
    manager.close()
    assert got, "ground-authorized request never honored"


@pytest.mark.verifies("FSW-MODE-008")
def test_missing_ack_is_surfaced_as_a_fault(
    sessions: tuple[zenoh.Session, zenoh.Session], tmp_path: Path
) -> None:
    manager_session, client_session = sessions
    entry = _entry(tmp_path, apps=("acking_app", "dead_app"), ack_timeout_s=0.4)
    manager = ModeManager(entry, manager_session, base_topic=BASE)
    time.sleep(0.5)

    client = ModeClient(client_session, "acking_app", base_topic=BASE)  # dead_app never shows up
    time.sleep(0.1)
    manager.request(
        mode_pb2.ModeRequest(
            source="ground",
            requested=mode_pb2.SYSTEM_MODE_RECOVERY,
            reason="checkout",
            ground_authority=True,
        )
    )
    assert _wait_for(lambda: client.current is not None and client.current.mode_seq >= 2)
    time.sleep(entry.ack_timeout_s + 0.2)  # let the ack window expire

    missing = manager.missing_acks()
    health = manager.publish_health()
    client.close()
    manager.close()

    assert missing == ["dead_app"], f"expected only dead_app missing, got {missing}"
    assert list(health.missing_acks) == ["dead_app"]
    assert health.mode == int(mode_pb2.SYSTEM_MODE_RECOVERY)
