"""Mode state machine: every §7 rule pinned, no bus, no sleeping."""

import pytest

from flatsat.mode.machine import BOOT_SOURCE, ModeStateMachine
from flatsat.msgs import mode_pb2

DWELL_NS = 1_000_000_000  # 1 s


def _machine(start_ns: int = 0) -> ModeStateMachine:
    return ModeStateMachine(min_dwell_ns=DWELL_NS, start_ns=start_ns)


def _request(
    mode: mode_pb2.SystemMode.ValueType,
    source: str = "test",
    ground: bool = False,
    reason: str = "r",
) -> mode_pb2.ModeRequest:
    return mode_pb2.ModeRequest(
        source=source, requested=mode, reason=reason, ground_authority=ground
    )


def _to_nominal(machine: ModeStateMachine, now_ns: int) -> None:
    """Drive a fresh machine INIT -> NOMINAL (clean boot)."""
    decision = machine.decide(_request(mode_pb2.SYSTEM_MODE_NOMINAL, source=BOOT_SOURCE), now_ns)
    assert decision.accepted


def test_starts_latched_in_init() -> None:
    machine = _machine()
    assert machine.mode == mode_pb2.SYSTEM_MODE_INIT
    assert machine.mode_seq == 0


@pytest.mark.verifies("FSW-MODE-002")
def test_toward_safety_needs_no_authority_from_any_mode() -> None:
    for path in (
        [mode_pb2.SYSTEM_MODE_SAFE],  # INIT -> SAFE
        [mode_pb2.SYSTEM_MODE_NOMINAL, mode_pb2.SYSTEM_MODE_SAFE],  # via boot
    ):
        machine = _machine()
        now = DWELL_NS * 10
        for target in path:
            source = BOOT_SOURCE if target == mode_pb2.SYSTEM_MODE_NOMINAL else "fdir"
            decision = machine.decide(_request(target, source=source), now)
            assert decision.accepted, decision.reason
            now += DWELL_NS * 10
        assert machine.mode == mode_pb2.SYSTEM_MODE_SAFE


@pytest.mark.verifies("FSW-MODE-002")
def test_toward_safety_ignores_dwell() -> None:
    """Safety must never wait out a timer."""
    machine = _machine()
    _to_nominal(machine, DWELL_NS * 10)
    # Immediately after the transition — dwell has NOT elapsed.
    decision = machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir"), DWELL_NS * 10 + 1)
    assert decision.accepted


@pytest.mark.verifies("FSW-MODE-003")
def test_away_from_safety_requires_ground_authority() -> None:
    machine = _machine()
    machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir"), 0)
    now = DWELL_NS * 10

    refused = machine.decide(_request(mode_pb2.SYSTEM_MODE_RECOVERY, source="fdir"), now)
    assert not refused.accepted
    assert "ground" in refused.reason

    allowed = machine.decide(
        _request(mode_pb2.SYSTEM_MODE_RECOVERY, source="ground", ground=True), now
    )
    assert allowed.accepted
    assert machine.mode == mode_pb2.SYSTEM_MODE_RECOVERY


@pytest.mark.verifies("FSW-MODE-004")
def test_safe_to_nominal_must_go_through_recovery() -> None:
    machine = _machine()
    machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir"), 0)
    decision = machine.decide(
        _request(mode_pb2.SYSTEM_MODE_NOMINAL, source="ground", ground=True), DWELL_NS * 10
    )
    assert not decision.accepted
    assert "no transition" in decision.reason


@pytest.mark.verifies("FSW-MODE-004")
def test_nonexistent_transitions_are_rejected() -> None:
    machine = _machine()
    _to_nominal(machine, 0)
    decision = machine.decide(
        _request(mode_pb2.SYSTEM_MODE_RECOVERY, source="ground", ground=True), DWELL_NS * 10
    )
    assert not decision.accepted  # Nominal -> Recovery does not exist


def test_same_mode_request_is_rejected_not_a_transition() -> None:
    machine = _machine()
    before = machine.mode_seq
    decision = machine.decide(_request(mode_pb2.SYSTEM_MODE_INIT), 0)
    assert not decision.accepted
    assert machine.mode_seq == before


@pytest.mark.verifies("FSW-MODE-006")
def test_dwell_blocks_rapid_departure_from_safety() -> None:
    machine = _machine()
    machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir"), 0)

    too_soon = machine.decide(
        _request(mode_pb2.SYSTEM_MODE_RECOVERY, source="ground", ground=True),
        DWELL_NS // 2,
    )
    assert not too_soon.accepted
    assert "dwell" in too_soon.reason

    after_dwell = machine.decide(
        _request(mode_pb2.SYSTEM_MODE_RECOVERY, source="ground", ground=True),
        DWELL_NS * 2,
    )
    assert after_dwell.accepted


@pytest.mark.verifies("FSW-MODE-005")
def test_sequence_is_monotonic_and_reason_carries_source() -> None:
    machine = _machine()
    _to_nominal(machine, 0)
    assert machine.mode_seq == 1
    machine.decide(
        _request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir", reason="gyro flatline"),
        DWELL_NS * 10,
    )
    assert machine.mode_seq == 2
    assert "fdir" in machine.reason and "gyro flatline" in machine.reason


def test_safe_entries_counted_for_flap_escalation() -> None:
    machine = _machine()
    now = 0
    for _ in range(3):
        _to_nominal(machine, now) if machine.mode == mode_pb2.SYSTEM_MODE_INIT else None
        if machine.mode == mode_pb2.SYSTEM_MODE_SAFE:
            now += DWELL_NS * 10
            machine.decide(
                _request(mode_pb2.SYSTEM_MODE_RECOVERY, source="ground", ground=True), now
            )
            now += DWELL_NS * 10
            machine.decide(
                _request(mode_pb2.SYSTEM_MODE_NOMINAL, source="ground", ground=True), now
            )
        now += DWELL_NS * 10
        machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir"), now)
    assert machine.safe_entries == 3


def test_state_message_reflects_the_latch() -> None:
    machine = _machine()
    machine.decide(_request(mode_pb2.SYSTEM_MODE_SAFE, source="fdir", reason="test"), 0)
    msg = machine.state_message(transition_time_ns=123)
    assert msg.mode == mode_pb2.SYSTEM_MODE_SAFE
    assert msg.mode_seq == 1
    assert msg.transition_time_ns == 123
