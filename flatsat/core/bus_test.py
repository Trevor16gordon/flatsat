"""Zenoh bus smoke tests: the two patterns the architecture depends on.

1. Pub/sub delivery of a serialized protobuf sensor message between two
   sessions (the HAL daemon -> consumer path, PLAN §4).
2. The latched-mode pattern (PLAN §7): a mode manager holds ModeState and
   answers queries on ``sys/mode``, so a LATE-JOINING process learns the
   current mode immediately via get() instead of waiting for a broadcast.

These run two zenoh peer sessions inside one process — enough to prove the
bus mechanics and the serialization seam without orchestration.
"""

import json
import time
from collections.abc import Callable, Iterator

import pytest
import zenoh

from flatsat.msgs import hal_pb2, mode_pb2

RECV_TIMEOUT_S = 5.0


def _wait_for(predicate: Callable[[], object], timeout_s: float = RECV_TIMEOUT_S) -> bool:
    """Poll ``predicate`` until it returns truthy or the timeout expires.

    Args:
        predicate: Zero-arg callable returning truthiness.
        timeout_s: Maximum seconds to wait.

    Returns:
        True if the predicate became truthy in time.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture(name="two_sessions")
def fixture_two_sessions() -> Iterator[tuple[zenoh.Session, zenoh.Session]]:
    """Open two independent zenoh peer sessions (publisher and consumer).

    Yields:
        Tuple of (session_a, session_b), closed on teardown.
    """
    a = zenoh.open(zenoh.Config())
    b = zenoh.open(zenoh.Config())
    yield a, b
    a.close()
    b.close()


def test_pubsub_delivers_imu_sample(two_sessions: tuple[zenoh.Session, zenoh.Session]) -> None:
    pub_session, sub_session = two_sessions
    received: list[bytes] = []
    # Test-only topic: production key expressions must never be used here, or
    # a live daemon (or a HIL sim on another machine) publishes into the test
    # and the assertions read someone else's data.
    topic = "test/bus/imu_sample"
    sub = sub_session.declare_subscriber(
        topic, lambda sample: received.append(bytes(sample.payload.to_bytes()))
    )
    time.sleep(0.5)  # let peer discovery + subscription propagate

    msg = hal_pb2.ImuSample()
    msg.header.source = "imu0"
    msg.header.seq = 1
    msg.header.validity = hal_pb2.VALIDITY_FLAG_VALID
    msg.gyro_z_rad_s = 0.02
    pub_session.put(topic, msg.SerializeToString())

    assert _wait_for(lambda: received), "no message arrived over the bus"
    back = hal_pb2.ImuSample.FromString(received[0])
    assert back.header.source == "imu0"
    assert back.header.seq == 1
    assert back.gyro_z_rad_s == pytest.approx(0.02)
    sub.undeclare()


@pytest.mark.verifies("FSW-BUS-002")
def test_late_joiner_learns_mode_via_query(
    two_sessions: tuple[zenoh.Session, zenoh.Session],
) -> None:
    mgr_session, late_session = two_sessions

    current = mode_pb2.ModeState(
        mode=mode_pb2.SYSTEM_MODE_SAFE,
        mode_seq=7,
        reason="test: boot into safe",
    )

    def answer(query: zenoh.Query) -> None:
        """Reply to a sys/mode query with the current latched state.

        Args:
            query: The incoming zenoh query.
        """
        query.reply(topic, current.SerializeToString())

    # Test-only key: the production sys/mode now has a LIVE mode manager
    # answering queries on this box.
    topic = "test/bus/sys_mode"
    queryable = mgr_session.declare_queryable(topic, answer)
    time.sleep(0.5)  # let peer discovery + queryable propagate

    # The "late joiner": queries AFTER the last broadcast happened.
    answers: list[bytes] = []
    for reply in late_session.get(topic, timeout=RECV_TIMEOUT_S):
        if reply.ok is not None:
            answers.append(bytes(reply.ok.payload.to_bytes()))

    assert answers, "late joiner got no answer from the queryable"
    state = mode_pb2.ModeState.FromString(answers[0])
    assert state.mode == mode_pb2.SYSTEM_MODE_SAFE
    assert state.mode_seq == 7
    queryable.undeclare()


def test_bus_config_defaults_to_multicast_scouting(monkeypatch: pytest.MonkeyPatch) -> None:
    """No endpoints configured: the LAN default, untouched."""
    from flatsat.core.bus import ENDPOINTS_ENV, bus_config

    monkeypatch.delenv(ENDPOINTS_ENV, raising=False)
    cfg = bus_config()
    assert json.loads(cfg.get_json("mode")) is None
    assert json.loads(cfg.get_json("connect/endpoints")) == []


def test_bus_config_endpoints_select_client_mode(monkeypatch: pytest.MonkeyPatch) -> None:
    """Router endpoints demand CLIENT mode — peers are not routed."""
    from flatsat.core.bus import ENDPOINTS_ENV, bus_config

    monkeypatch.setenv(ENDPOINTS_ENV, "tcp/100.65.0.120:7447, tcp/127.0.0.1:7447")
    cfg = bus_config()
    assert json.loads(cfg.get_json("mode")) == "client"
    assert json.loads(cfg.get_json("connect/endpoints")) == [
        "tcp/100.65.0.120:7447",
        "tcp/127.0.0.1:7447",
    ]
