"""Ground-commanding CLI internals against a live manager on the bus."""

import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import zenoh

from flatsat.apps.mode_request import current_state, send_request
from flatsat.mode import mode_config_pb2
from flatsat.mode.manager import ModeManager
from flatsat.msgs import mode_pb2

# Test-only key base: the production sys/mode has a LIVE manager on this box.
BASE = "test/cli/sys/mode"


@pytest.fixture(name="manager")
def fixture_manager(tmp_path: Path) -> Iterator[ModeManager]:
    """A mode manager on a test-only base topic, closed on teardown.

    Yields:
        The manager (booted SAFE — no clean-shutdown marker).
    """
    session = zenoh.open(zenoh.Config())
    entry = mode_config_pb2.ModeConfig(
        apps=[],
        ack_timeout_s=0.5,
        min_dwell_s=0.05,
        clean_shutdown_marker=str(tmp_path / "marker"),
    )
    manager = ModeManager(entry, session, base_topic=BASE)
    time.sleep(0.5)  # discovery
    yield manager
    manager.close()
    session.close()


def test_status_reads_the_latched_mode(manager: ModeManager) -> None:
    session = zenoh.open(zenoh.Config())
    time.sleep(0.5)  # let the fresh session discover the manager's queryable
    state = current_state(session, BASE)
    session.close()
    assert state is not None
    assert state.mode == mode_pb2.SYSTEM_MODE_SAFE


def test_ground_request_leaves_safe(manager: ModeManager) -> None:
    session = zenoh.open(zenoh.Config())
    time.sleep(0.1)  # dwell
    before, after = send_request(session, BASE, "RECOVERY", "checkout", ground=True)
    session.close()
    assert before is not None and after is not None
    assert after.mode == mode_pb2.SYSTEM_MODE_RECOVERY
    assert after.mode_seq == before.mode_seq + 1


def test_unauthorized_request_is_refused(manager: ModeManager) -> None:
    session = zenoh.open(zenoh.Config())
    time.sleep(0.1)
    before, after = send_request(session, BASE, "RECOVERY", "sneaky", ground=False)
    session.close()
    assert before is not None and after is not None
    assert after.mode == mode_pb2.SYSTEM_MODE_SAFE
    assert after.mode_seq == before.mode_seq, "refused request must not advance the seq"


def test_unknown_mode_fails_loud(manager: ModeManager) -> None:
    session = zenoh.open(zenoh.Config())
    try:
        with pytest.raises(ValueError, match="unknown mode"):
            send_request(session, BASE, "WARP", "no", ground=True)
    finally:
        session.close()
