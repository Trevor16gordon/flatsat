#!/usr/bin/env python3
"""Bus round-trip latency benchmark: measure what PREEMPT_RT cannot promise.

cyclictest bounds the SCHEDULER (69 us worst-case on this box, 2026-07-24);
this tool bounds the TRANSPORT — the zenoh/TCP loopback path every bus
message rides. It answers, with percentiles instead of reasoning: can a
100-200 Hz control path afford to cross the bus? (PLAN §4 keeps the option
of consolidating into one pinned process if the answer is ever no.)

Method: an echo process republishes every payload from bench/ping onto
bench/pong untouched (no parse — minimal responder overhead). The pinger
sends paced protobuf ImuSamples carrying a monotonic send timestamp,
receives its own echo, and records round-trip time; one-way ~= RTT/2.
Warmup pings (discovery, JIT, cache) are discarded by sequence number.

Run under load for the numbers that matter (same torture as the N3 gate):
  stress-ng --cpu 4 --vm 2 --vm-bytes 1G &  +  the GPU matmul loop

Usage (from the repo root):
  python tools/bus_bench.py                     # spawn echo child + measure
  python tools/bus_bench.py --count 5000 --rate 500
  python tools/bus_bench.py --role echo         # responder only (manual pairing)
  sudo ... --fifo 80                            # try SCHED_FIFO on both ends

Exit code 0 = report produced; 4 = excessive loss (>1% pongs missing).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import zenoh

from flatsat.core.bus import bus_config

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))  # allow `python tools/bus_bench.py` from anywhere

from flatsat.msgs import hal_pb2  # noqa: E402 — needs the sys.path bootstrap above

PING_KEY = "bench/ping"
PONG_KEY = "bench/pong"


def try_fifo(priority: int | None) -> str:
    """Attempt to switch the calling process to SCHED_FIFO.

    Args:
        priority: FIFO priority 1-99, or None to stay on the default policy.

    Returns:
        Human-readable description of the scheduling policy in effect.
    """
    if priority is None:
        return "SCHED_OTHER (default; pass --fifo N under sudo for RT)"
    try:
        import os

        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except PermissionError:
        return f"SCHED_OTHER (no permission for FIFO {priority}; run under sudo)"
    return f"SCHED_FIFO priority {priority}"


def run_echo(fifo: int | None) -> int:
    """Run the echo responder until killed.

    Args:
        fifo: Optional SCHED_FIFO priority to attempt.

    Returns:
        0 on clean (KeyboardInterrupt) exit.
    """
    policy = try_fifo(fifo)
    session = zenoh.open(bus_config())
    pub = session.declare_publisher(PONG_KEY)

    def bounce(sample: zenoh.Sample) -> None:
        """Republish the ping payload on the pong key, unparsed.

        Args:
            sample: Incoming ping.
        """
        pub.put(sample.payload.to_bytes())

    sub = session.declare_subscriber(PING_KEY, bounce)
    print(f"echo: ready ({policy})", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    sub.undeclare()
    session.close()
    return 0


def run_ping(count: int, rate_hz: float, warmup: int, fifo: int | None) -> int:
    """Send paced pings, collect echoed RTTs, print the percentile report.

    Args:
        count: Measured pings (after warmup).
        rate_hz: Ping pacing rate.
        warmup: Leading pings discarded from statistics.
        fifo: Optional SCHED_FIFO priority to attempt.

    Returns:
        0 on success; 4 if more than 1% of measured pongs went missing.
    """
    policy = try_fifo(fifo)
    session = zenoh.open(bus_config())
    rtts_ns: list[int] = []
    lock = threading.Lock()

    def on_pong(sample: zenoh.Sample) -> None:
        """Record round-trip time for a returning ping.

        Args:
            sample: The echoed payload.
        """
        t_recv = time.monotonic_ns()
        msg = hal_pb2.ImuSample.FromString(bytes(sample.payload.to_bytes()))
        if msg.header.seq < warmup:
            return
        with lock:
            rtts_ns.append(t_recv - int(msg.header.sample_time_ns))

    sub = session.declare_subscriber(PONG_KEY, on_pong)
    pub = session.declare_publisher(PING_KEY)
    time.sleep(1.0)  # discovery

    period_ns = int(1_000_000_000 / rate_hz)
    msg = hal_pb2.ImuSample()
    msg.header.source = "bus_bench"
    payload_len = 0
    next_wake = time.monotonic_ns() + period_ns
    for seq in range(warmup + count):
        msg.header.seq = seq
        msg.header.sample_time_ns = time.monotonic_ns()
        wire = msg.SerializeToString()
        payload_len = len(wire)
        pub.put(wire)
        delta = next_wake - time.monotonic_ns()
        if delta > 0:
            time.sleep(delta / 1e9)
        next_wake += period_ns

    time.sleep(0.5)  # drain stragglers
    sub.undeclare()
    session.close()

    with lock:
        oneway_us = np.asarray(sorted(rtts_ns), dtype=np.float64) / 2.0 / 1000.0
    received = int(oneway_us.size)
    loss = count - received

    print("-" * 64)
    print(f"  pinger policy : {policy}")
    print(f"  payload       : {payload_len} B protobuf ImuSample, loopback TCP")
    print(f"  pings         : {count} at {rate_hz:g} Hz ({warmup} warmup discarded)")
    print(f"  received      : {received}  (lost {loss})")
    if received:
        print(f"  one-way  min  : {oneway_us[0]:9.1f} us")
        print(f"  one-way  p50  : {np.percentile(oneway_us, 50):9.1f} us")
        print(f"  one-way  p99  : {np.percentile(oneway_us, 99):9.1f} us")
        print(f"  one-way p99.9 : {np.percentile(oneway_us, 99.9):9.1f} us")
        print(f"  one-way  MAX  : {oneway_us[-1]:9.1f} us")
    print("-" * 64)
    return 4 if loss > 0.01 * count else 0


def main() -> int:
    """Entry point: run echo, pinger, or (default) both.

    Returns:
        Process exit code (see module docstring).
    """
    parser = argparse.ArgumentParser(description="Zenoh bus round-trip latency benchmark.")
    parser.add_argument("--role", choices=["both", "echo", "ping"], default="both")
    parser.add_argument("--count", type=int, default=2000, help="measured pings")
    parser.add_argument("--rate", type=float, default=200.0, help="ping rate [Hz]")
    parser.add_argument("--warmup", type=int, default=100, help="warmup pings discarded")
    parser.add_argument("--fifo", type=int, default=None, help="SCHED_FIFO priority to attempt")
    args = parser.parse_args()

    if args.role == "echo":
        return run_echo(args.fifo)
    if args.role == "ping":
        return run_ping(args.count, args.rate, args.warmup, args.fifo)

    echo_cmd = [sys.executable, str(Path(__file__).resolve()), "--role", "echo"]
    if args.fifo is not None:
        echo_cmd += ["--fifo", str(args.fifo)]
    echo = subprocess.Popen(echo_cmd, cwd=REPO_ROOT)
    try:
        time.sleep(1.5)  # let the responder come up
        return run_ping(args.count, args.rate, args.warmup, args.fifo)
    finally:
        echo.terminate()
        echo.wait(timeout=5)


if __name__ == "__main__":
    sys.exit(main())
