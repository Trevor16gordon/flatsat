#!/usr/bin/env python3
"""A1 mock ADCS control loop: the first closed loop over the bus.

Subscribes to an ImuSample topic, runs a PD rate-damping law at a fixed
rate under full RT hygiene (absolute-deadline cadence, GC quiesced, core
pinning, optional SCHED_FIFO/mlockall), publishes WheelTorqueCommand every
cycle, and instruments itself like cyclictest: wakeup lateness, control-step
execution time, and sensor staleness are all recorded and reported as
percentiles.

A1 acceptance context: the target is < ~100 us wakeup jitter WITH core
isolation + FIFO (a later, sudo'd session). Unprivileged runs of this same
binary establish the baseline that tuning gets compared against. The
consolidated-vs-federated question (PLAN §4) is settled by THIS instrument:
sensor->bus->loop->bus->actuator, measured, not argued.

Usage:
  python -m flight.adcs.loop --duration 20            # needs an IMU publisher
  python -m flight.adcs.loop --spawn-imu --duration 20 # self-contained
  sudo ... --fifo 80 --pin 3                           # RT flavor
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import zenoh

from flight.msgs import adcs_pb2, hal_pb2
from flight.rt import pin_to_core, quiesce_gc, try_fifo, try_mlockall

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class LoopReport:
    """Everything one loop run measured about itself.

    Attributes:
        cycles: Control cycles executed.
        wakeup_lateness_us: Per-cycle wakeup lateness samples.
        exec_time_us: Per-cycle control-step execution time samples.
        stale_cycles: Cycles that ran on a sensor sample older than the
            staleness threshold.
        conditions: Human-readable RT-hygiene state strings.
    """

    cycles: int = 0
    wakeup_lateness_us: list[float] = field(default_factory=list)
    exec_time_us: list[float] = field(default_factory=list)
    stale_cycles: int = 0
    conditions: list[str] = field(default_factory=list)

    def print_summary(self) -> None:
        """Print the percentile report (same shape as bus_bench/cyclictest)."""
        print("-" * 64)
        for cond in self.conditions:
            print(f"  {cond}")
        print(f"  cycles        : {self.cycles}  (stale-input cycles: {self.stale_cycles})")
        for label, data in (
            ("wakeup lateness", np.asarray(self.wakeup_lateness_us)),
            ("exec time      ", np.asarray(self.exec_time_us)),
        ):
            if data.size == 0:
                continue
            print(
                f"  {label}: p50 {np.percentile(data, 50):7.1f}  "
                f"p99 {np.percentile(data, 99):7.1f}  "
                f"p99.9 {np.percentile(data, 99.9):7.1f}  "
                f"MAX {data.max():7.1f}  us"
            )
        print("-" * 64)


class AdcsLoop:
    """PD rate-damping loop: latest gyro sample in, wheel torque out."""

    def __init__(
        self,
        session: zenoh.Session,
        rate_hz: float,
        sensor_topic: str = "hal/imu0/sample",
        command_topic: str = "adcs/wheel_torque",
        kp: float = 0.02,
        kd: float = 0.005,
        stale_after_s: float = 0.05,
    ) -> None:
        """Wire the loop to its topics.

        Args:
            session: Open zenoh session.
            rate_hz: Control rate.
            sensor_topic: ImuSample source topic.
            command_topic: WheelTorqueCommand output topic.
            kp: Proportional gain on body rate.
            kd: Derivative gain on body rate.
            stale_after_s: Input age beyond which a cycle counts as stale.
        """
        self._rate_hz = rate_hz
        self._stale_after_ns = int(stale_after_s * 1e9)
        self._kp = kp
        self._kd = kd
        self._pub = session.declare_publisher(command_topic)
        self._latest: hal_pb2.ImuSample | None = None
        self._latest_recv_ns = 0
        self._lock = threading.Lock()
        self._sub = session.declare_subscriber(sensor_topic, self._on_sample)
        self._stop = threading.Event()
        self._prev_rates = (0.0, 0.0, 0.0)

    def _on_sample(self, sample: zenoh.Sample) -> None:
        """Store the latest sensor sample (subscriber thread).

        Args:
            sample: Incoming ImuSample.
        """
        msg = hal_pb2.ImuSample.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._latest = msg
            self._latest_recv_ns = time.monotonic_ns()

    def wait_for_first_sample(self, timeout_s: float = 5.0) -> bool:
        """Block until the first sensor sample arrives.

        Args:
            timeout_s: Give-up horizon.

        Returns:
            True if a sample arrived in time.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None:
                    return True
            time.sleep(0.01)
        return False

    def _control_step(self, seq: int) -> bool:
        """Run one PD step on the latest sample and publish the command.

        Args:
            seq: Command sequence number.

        Returns:
            True if the input sample was stale.
        """
        now_ns = time.monotonic_ns()
        with self._lock:
            imu = self._latest
            age_ns = now_ns - self._latest_recv_ns
        assert imu is not None  # guaranteed by wait_for_first_sample
        stale = age_ns > self._stale_after_ns

        rates = (imu.gyro_x_rad_s, imu.gyro_y_rad_s, imu.gyro_z_rad_s)
        dt = 1.0 / self._rate_hz
        cmd = adcs_pb2.WheelTorqueCommand()
        for axis, (rate, prev) in enumerate(zip(rates, self._prev_rates, strict=True)):
            torque = -self._kp * rate - self._kd * (rate - prev) / dt
            if axis == 0:
                cmd.torque_x_n_m = torque
            elif axis == 1:
                cmd.torque_y_n_m = torque
            else:
                cmd.torque_z_n_m = torque
        self._prev_rates = rates

        cmd.header.source = "adcs_loop"
        cmd.header.seq = seq
        cmd.header.sample_time_ns = time.time_ns()
        cmd.header.publish_time_ns = time.time_ns()
        cmd.header.validity = hal_pb2.VALIDITY_FLAG_STALE if stale else 0
        self._pub.put(cmd.SerializeToString())
        return stale

    def run(self, duration_s: float) -> LoopReport:
        """Run the loop for a fixed duration and return the self-measurement.

        Args:
            duration_s: How long to run.

        Returns:
            The populated LoopReport.
        """
        report = LoopReport()
        period_ns = int(1_000_000_000 / self._rate_hz)
        end_ns = time.monotonic_ns() + int(duration_s * 1e9)
        next_wake = time.monotonic_ns() + period_ns
        seq = 0
        while not self._stop.is_set():
            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                time.sleep(delta / 1e9)
            woke = time.monotonic_ns()
            if woke >= end_ns:
                break
            report.wakeup_lateness_us.append((woke - next_wake) / 1000.0)

            seq += 1
            stale = self._control_step(seq)
            report.exec_time_us.append((time.monotonic_ns() - woke) / 1000.0)
            report.cycles += 1
            report.stale_cycles += int(stale)
            next_wake += period_ns
        return report

    def stop(self) -> None:
        """Request the loop to exit (thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        self._sub.undeclare()


def main() -> int:
    """Run the mock ADCS loop from the command line.

    Returns:
        0 on success; 3 if no sensor data ever arrived.
    """
    parser = argparse.ArgumentParser(description="A1 mock ADCS control loop (bus-first).")
    parser.add_argument("--rate", type=float, default=100.0, help="control rate [Hz]")
    parser.add_argument("--duration", type=float, default=30.0, help="run time [s]")
    parser.add_argument("--fifo", type=int, default=None, help="SCHED_FIFO priority to attempt")
    parser.add_argument("--pin", type=int, default=None, help="CPU core to pin to")
    parser.add_argument(
        "--spawn-imu", action="store_true", help="spawn a fake IMU daemon child for the run"
    )
    args = parser.parse_args()

    imu_proc: subprocess.Popen[bytes] | None = None
    if args.spawn_imu:
        imu_proc = subprocess.Popen(
            [sys.executable, "-m", "flight.hal.fake_imu", "--rate", "100"], cwd=REPO_ROOT
        )

    conditions = [
        try_fifo(args.fifo),
        try_mlockall(),
        pin_to_core(args.pin),
        quiesce_gc(),
        f"control rate {args.rate:g} Hz over the bus (zenoh loopback)",
    ]
    session = zenoh.open(zenoh.Config())
    loop = AdcsLoop(session, args.rate)
    try:
        if not loop.wait_for_first_sample():
            print("no sensor data on hal/imu0/sample — start an IMU or use --spawn-imu")
            return 3
        report = loop.run(args.duration)
        report.conditions = conditions
        report.print_summary()
    finally:
        loop.close()
        session.close()
        if imu_proc is not None:
            imu_proc.terminate()
            imu_proc.wait(timeout=5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
