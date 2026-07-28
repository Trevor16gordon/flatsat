"""FDIR rule engine: every threshold pinned, clock injected, no sleeping."""

import pytest

from flatsat.control.health import fdir_config_pb2
from flatsat.control.health.rules import RuleMonitor, build_monitors

S = int(1e9)  # seconds -> ns


def _flag_rule(
    max_ratio: float = 0.5, window_s: float = 10.0, min_trip_s: float = 1.0
) -> fdir_config_pb2.FdirRule:
    return fdir_config_pb2.FdirRule(
        name="stale",
        min_trip_s=min_trip_s,
        flag_ratio=fdir_config_pb2.FlagRatioRule(
            topic="test/fdir/imu", flag_mask=8, window_s=window_s, max_ratio=max_ratio
        ),
    )


def _silence_rule(timeout_s: float = 2.0) -> fdir_config_pb2.FdirRule:
    return fdir_config_pb2.FdirRule(
        name="silent",
        min_trip_s=1.0,
        silence=fdir_config_pb2.SilenceRule(topic="test/fdir/imu", timeout_s=timeout_s),
    )


@pytest.mark.verifies("FSW-FDIR-001")
def test_rulebook_is_declarative_config() -> None:
    """Monitors are built from the proto rulebook — rules are data."""
    config = fdir_config_pb2.FdirConfig(rules=[_flag_rule(), _silence_rule()])
    monitors = build_monitors(config, start_ns=0)
    assert [m.name for m in monitors] == ["stale", "silent"]
    assert all(m.topic == "test/fdir/imu" for m in monitors)


def test_rule_without_a_check_fails_loud() -> None:
    bare = fdir_config_pb2.FdirRule(name="empty")
    with pytest.raises(ValueError, match="no check selected"):
        RuleMonitor(bare, start_ns=0)


@pytest.mark.verifies("FSW-FDIR-002")
def test_sustained_stale_ratio_trips() -> None:
    monitor = RuleMonitor(_flag_rule(max_ratio=0.5, window_s=10.0, min_trip_s=1.0), start_ns=0)
    # 100% flagged for 2 s: condition holds from the first sample; trip
    # requires the 1 s persistence.
    for tick in range(20):
        monitor.on_message(tick * S // 10, validity=8)
    assert not monitor.tripped(0), "no persistence yet"
    monitor.tripped(0)  # start the persistence clock at t=0 evaluation
    assert monitor.tripped(2 * S), "held 2 s beyond min_trip 1 s"


def test_brief_stall_does_not_trip() -> None:
    """The HIL census lesson: sub-second stalls are NORMAL, never faults."""
    monitor = RuleMonitor(_flag_rule(max_ratio=0.5, window_s=10.0, min_trip_s=1.0), start_ns=0)
    # 10 s of healthy 10 Hz traffic with ONE 0.7 s stall (7 flagged of 100).
    for tick in range(100):
        flagged = 8 if 50 <= tick < 57 else 0
        monitor.on_message(tick * S // 10, validity=flagged)
        assert not monitor.tripped(tick * S // 10)


def test_recovery_resets_persistence() -> None:
    monitor = RuleMonitor(_flag_rule(max_ratio=0.5, window_s=2.0, min_trip_s=1.0), start_ns=0)
    for tick in range(10):  # 1 s fully flagged — condition holds, not yet tripped
        monitor.on_message(tick * S // 10, validity=8)
    monitor.tripped(S)  # persistence clock running
    for tick in range(10, 40):  # 3 s of clean traffic flushes the window
        monitor.on_message(tick * S // 10, validity=0)
    assert not monitor.tripped(4 * S)
    # A NEW degradation must re-earn its persistence from zero.
    for tick in range(40, 60):
        monitor.on_message(tick * S // 10, validity=8)
    assert not monitor.tripped(int(4.05 * S)), "fresh condition, persistence not yet met"


@pytest.mark.verifies("FSW-FDIR-002")
def test_silence_trips_without_any_message() -> None:
    """A topic that NEVER speaks is the process-death signature."""
    monitor = RuleMonitor(_silence_rule(timeout_s=2.0), start_ns=0)
    assert not monitor.tripped(1 * S)
    monitor.tripped(int(2.5 * S))  # condition starts holding; persistence clock arms
    assert monitor.tripped(4 * S), "silent past timeout + persistence"


def test_sound_resets_silence() -> None:
    monitor = RuleMonitor(_silence_rule(timeout_s=2.0), start_ns=0)
    monitor.on_message(int(1.5 * S), validity=8)  # a FLAGGED message is still sound
    assert not monitor.tripped(3 * S)


def test_empty_window_is_not_a_flag_ratio_fault() -> None:
    """Silence is SilenceRule's fault — ratio over nothing is no ratio."""
    monitor = RuleMonitor(_flag_rule(window_s=1.0), start_ns=0)
    monitor.on_message(0, validity=8)
    assert not monitor.tripped(10 * S), "sample aged out of the window entirely"
