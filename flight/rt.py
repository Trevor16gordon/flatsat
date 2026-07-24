"""Real-time process hygiene helpers: scheduling, memory locking, GC, pinning.

Each helper ATTEMPTS its change and returns a human-readable description of
what actually happened — privileged operations (SCHED_FIFO, mlockall) fail
soft with an explanation instead of raising, so the same code runs in
unprivileged baseline mode and sudo'd RT mode. Callers print the returned
strings so every report states the conditions it was measured under.
"""

from __future__ import annotations

import ctypes
import gc
import os

_MCL_CURRENT = 1
_MCL_FUTURE = 2


def try_fifo(priority: int | None) -> str:
    """Attempt to switch the calling process to SCHED_FIFO.

    Args:
        priority: FIFO priority 1-99, or None to stay on the default policy.

    Returns:
        Description of the scheduling policy now in effect.
    """
    if priority is None:
        return "SCHED_OTHER (default; pass --fifo N under sudo for RT)"
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except PermissionError:
        return f"SCHED_OTHER (no permission for FIFO {priority}; run under sudo)"
    return f"SCHED_FIFO priority {priority}"


def try_mlockall() -> str:
    """Attempt to lock all current and future pages into RAM.

    Returns:
        Description of the memory-locking state.
    """
    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    if libc.mlockall(_MCL_CURRENT | _MCL_FUTURE) != 0:
        return "memory not locked (mlockall needs privilege; run under sudo)"
    return "mlockall: all pages locked"


def pin_to_core(core: int | None) -> str:
    """Pin the calling process to a single CPU core.

    Args:
        core: Core index, or None to leave affinity unrestricted.

    Returns:
        Description of the CPU affinity now in effect.
    """
    if core is None:
        return "affinity: unrestricted"
    os.sched_setaffinity(0, {core})
    return f"affinity: pinned to core {core}"


def describe_actual() -> list[str]:
    """Report the scheduling state ACTUALLY in effect (declare-then-verify).

    The try_* helpers describe what this process attempted; the deployment
    layer (systemd CPUSchedulingPolicy=/CPUAffinity=) may have granted more.
    Reports should print measured truth, not attempts.

    Returns:
        Verified policy/priority and CPU affinity strings.
    """
    names = {os.SCHED_OTHER: "SCHED_OTHER", os.SCHED_FIFO: "SCHED_FIFO", os.SCHED_RR: "SCHED_RR"}
    policy = names.get(os.sched_getscheduler(0), "unknown-policy")
    priority = os.sched_getparam(0).sched_priority
    cpus = ",".join(str(c) for c in sorted(os.sched_getaffinity(0)))
    return [f"verified: {policy} priority {priority}", f"verified: affinity cpus {cpus}"]


def quiesce_gc() -> str:
    """Freeze existing objects out of GC scanning and disable collection.

    The control loop must not allocate in steady state; disabling the
    collector removes its pauses from the jitter budget. Long-running
    processes that DO allocate should re-enable and collect at a safe point.

    Returns:
        Description of the GC state.
    """
    gc.freeze()
    gc.disable()
    return "gc: frozen + disabled"
