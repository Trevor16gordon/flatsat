"""Real-time hygiene helpers: report what IS, not what was requested."""

import os

import pytest

from flight.core.rt import describe_actual, pin_to_core, quiesce_gc, try_fifo, try_mlockall


@pytest.mark.verifies("FSW-RT-003")
def test_describe_actual_reports_live_values() -> None:
    """The report must read the kernel, not echo the request.

    A live service once reported "SCHED_OTHER (default)" while systemd had
    granted it FIFO 80 on core 3 — the strings described attempts. These
    read sched_getscheduler/getparam/getaffinity instead.
    """
    lines = describe_actual()
    assert len(lines) == 2
    policy_line, affinity_line = lines

    names = {os.SCHED_OTHER: "SCHED_OTHER", os.SCHED_FIFO: "SCHED_FIFO", os.SCHED_RR: "SCHED_RR"}
    expected_policy = names[os.sched_getscheduler(0)]
    assert expected_policy in policy_line
    assert str(os.sched_getparam(0).sched_priority) in policy_line

    for cpu in sorted(os.sched_getaffinity(0)):
        assert str(cpu) in affinity_line


def test_unprivileged_helpers_fail_soft() -> None:
    """Privileged operations must explain themselves, never raise."""
    assert "SCHED_OTHER" in try_fifo(None)
    assert isinstance(try_fifo(80), str)  # granted or explained, never an exception
    assert isinstance(try_mlockall(), str)
    assert "unrestricted" in pin_to_core(None)
    assert "gc" in quiesce_gc()


def test_pin_to_core_restricts_affinity() -> None:
    original = os.sched_getaffinity(0)
    try:
        message = pin_to_core(0)
        assert "core 0" in message
        assert os.sched_getaffinity(0) == {0}
    finally:
        os.sched_setaffinity(0, original)
