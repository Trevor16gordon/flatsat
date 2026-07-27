#!/usr/bin/env python3
"""Generic control loop: runs ANY controller named in a vehicle composition.

The imperative shell around a control strategy. It owns everything that is
true of every controller — subscribing to the state input, real-time
hygiene, absolute-deadline cadence, actuator publishing, self-measurement —
and delegates the strategy-specific part to an
:class:`~flight.adcs.controller.AttitudeController` resolved by name, with
its target supplied by a :class:`~flight.adcs.guidance.ReferenceSource`.

Swapping PD for PID for an ML policy, or detumble for a pointing objective,
is a vehicle-file edit. This file does not change.

Self-measurement mirrors cyclictest: wakeup lateness, control-step
execution time, stale-input cycles, and actuator saturation, reported as
percentiles on an interval (bounded memory for indefinite service runs).

Usage:
  python -m flight.apps.control_loop --duration 20
  python -m flight.apps.control_loop --spawn-sensors --duration 20
  sudo ... --fifo 80 --pin 3          # RT flavor (systemd normally grants)
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import zenoh

from flight.adcs.controller import AttitudeController, AttitudeState
from flight.adcs.guidance import ReferenceSource
from flight.core.config import ControlEntry, VehicleSpec, load_vehicle
from flight.core.rt import describe_actual, pin_to_core, quiesce_gc, try_fifo, try_mlockall
from flight.msgs import adcs_pb2, hal_pb2
from flight.registry import get_controller_class, get_guidance_class

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class LoopReport:
    """Everything one loop run measured about itself.

    Attributes:
        cycles: Control cycles executed.
        wakeup_lateness_us: Per-cycle wakeup lateness samples.
        exec_time_us: Per-cycle control-step execution time samples.
        stale_cycles: Cycles whose input was older than the threshold.
        saturated_cycles: Cycles whose command hit the actuator limit.
        conditions: Configuration/hygiene strings echoed with every report.
    """

    cycles: int = 0
    wakeup_lateness_us: list[float] = field(default_factory=list)
    exec_time_us: list[float] = field(default_factory=list)
    stale_cycles: int = 0
    saturated_cycles: int = 0
    conditions: list[str] = field(default_factory=list)

    def print_summary(self) -> None:
        """Print the percentile report (same shape as cyclictest/bus_bench)."""
        print("-" * 64)
        for cond in self.conditions:
            print(f"  {cond}")
        print(
            f"  cycles        : {self.cycles}  (stale {self.stale_cycles}, "
            f"saturated {self.saturated_cycles})"
        )
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
        print("-" * 64, flush=True)


class ControlLoop:
    """Bus-connected shell driving one controller against one reference."""

    def __init__(
        self,
        session: zenoh.Session,
        entry: ControlEntry,
        controller: AttitudeController,
        guidance: ReferenceSource,
    ) -> None:
        """Wire the loop to its topics, strategy, and objective.

        Args:
            session: Open zenoh session.
            entry: Control composition (rates, topics, thresholds).
            controller: Strategy computing torque.
            guidance: Source of the reference to track.
        """
        self.entry = entry
        self._controller = controller
        self._guidance = guidance
        self._stale_after_ns = int(entry.stale_after_s * 1e9)
        self._pub = session.declare_publisher(entry.output_topic)
        self._sub = session.declare_subscriber(entry.input_topic, self._on_sample)
        self._latest: hal_pb2.ImuSample | None = None
        self._latest_recv_ns = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_torque = (0.0, 0.0, 0.0)
        self.samples_received = 0

    def _on_sample(self, sample: zenoh.Sample) -> None:
        """Store the latest sensor sample (subscriber thread).

        Args:
            sample: Incoming ImuSample.
        """
        msg = hal_pb2.ImuSample.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._latest = msg
            self._latest_recv_ns = time.monotonic_ns()
            self.samples_received += 1

    def wait_for_first_sample(self, timeout_s: float = 5.0) -> bool:
        """Block until the first state input arrives.

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

    def _step(self, seq: int, t_s: float) -> tuple[bool, bool]:
        """Run one control step and publish the command.

        Args:
            seq: Command sequence number.
            t_s: Seconds since the loop started (for time-varying guidance).

        Returns:
            Tuple of (input was stale, command was saturated).
        """
        with self._lock:
            imu = self._latest
            age_ns = time.monotonic_ns() - self._latest_recv_ns
        assert imu is not None  # guaranteed by wait_for_first_sample
        stale = age_ns > self._stale_after_ns

        state = AttitudeState(
            body_rates_rad_s=(imu.gyro_x_rad_s, imu.gyro_y_rad_s, imu.gyro_z_rad_s),
            age_s=age_ns / 1e9,
            valid=not stale and imu.header.validity == hal_pb2.VALIDITY_FLAG_VALID,
        )
        output = self._controller.update(
            state, self._guidance.reference_at(t_s), 1.0 / self.entry.rate_hz
        )
        self._last_torque = output.torque_n_m

        cmd = adcs_pb2.WheelTorqueCommand()
        cmd.torque_x_n_m, cmd.torque_y_n_m, cmd.torque_z_n_m = output.torque_n_m
        cmd.header.source = "adcs_loop"
        cmd.header.seq = seq
        cmd.header.sample_time_ns = time.time_ns()
        cmd.header.publish_time_ns = time.time_ns()
        cmd.header.validity = int(hal_pb2.VALIDITY_FLAG_STALE) if stale else 0
        self._pub.put(cmd.SerializeToString())
        return stale, output.saturated

    def print_status(self) -> None:
        """Print a one-line live status: rates, input freshness, torque out."""
        with self._lock:
            imu = self._latest
            age_ms = (time.monotonic_ns() - self._latest_recv_ns) / 1e6
            rx = self.samples_received
        if imu is None:
            print("[adcs] no sensor data yet", flush=True)
            return
        omega = (imu.gyro_x_rad_s, imu.gyro_y_rad_s, imu.gyro_z_rad_s)
        mag_mrad = 1000.0 * math.sqrt(sum(w * w for w in omega))
        tx, ty, tz = self._last_torque
        print(
            f"[adcs] |omega|={mag_mrad:7.2f} mrad/s  "
            f"torque=({tx:+.2e},{ty:+.2e},{tz:+.2e}) N·m  "
            f"input age {age_ms:6.1f} ms  samples rx {rx}",
            flush=True,
        )

    def run(
        self,
        duration_s: float,
        report_every_s: float = 0.0,
        conditions: list[str] | None = None,
        status_every_s: float = 0.0,
    ) -> LoopReport:
        """Run the loop and return the self-measurement.

        Args:
            duration_s: Run time; <= 0 means until :meth:`stop` (service mode).
            report_every_s: Print+reset the report at this interval.
            conditions: Configuration strings echoed with every report.
            status_every_s: Print a one-line live status at this interval.

        Returns:
            The report covering the final (possibly partial) interval.
        """
        report = LoopReport(conditions=list(conditions or []))
        period_ns = int(1_000_000_000 / self.entry.rate_hz)
        start = time.monotonic_ns()
        end_ns = start + int(duration_s * 1e9) if duration_s > 0 else None
        report_ns = int(report_every_s * 1e9) if report_every_s > 0 else None
        next_report = start + report_ns if report_ns else None
        status_ns = int(status_every_s * 1e9) if status_every_s > 0 else None
        next_status = start + status_ns if status_ns else None
        next_wake = start + period_ns
        seq = 0
        while not self._stop.is_set():
            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                time.sleep(delta / 1e9)
            woke = time.monotonic_ns()
            if end_ns is not None and woke >= end_ns:
                break
            report.wakeup_lateness_us.append((woke - next_wake) / 1000.0)

            seq += 1
            stale, saturated = self._step(seq, (woke - start) / 1e9)
            report.exec_time_us.append((time.monotonic_ns() - woke) / 1000.0)
            report.cycles += 1
            report.stale_cycles += int(stale)
            report.saturated_cycles += int(saturated)
            next_wake += period_ns

            if next_status is not None and status_ns is not None and woke >= next_status:
                self.print_status()
                next_status += status_ns
            if next_report is not None and report_ns is not None and woke >= next_report:
                report.print_summary()
                report = LoopReport(conditions=report.conditions)
                next_report += report_ns
        return report

    def stop(self) -> None:
        """Request the loop to exit (thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        self._sub.undeclare()


def build_loop(session: zenoh.Session, vehicle: VehicleSpec) -> ControlLoop:
    """Compose the control loop declared by a vehicle.

    Args:
        session: Open zenoh session.
        vehicle: Loaded vehicle composition.

    Returns:
        The composed loop, ready to run.
    """
    entry = vehicle.control
    controller = get_controller_class(entry.strategy).from_config(entry.options)
    guidance = get_guidance_class(entry.objective).from_config(entry.objective_options)
    return ControlLoop(session, entry, controller, guidance)


def main() -> int:
    """Run the control loop from the command line.

    Returns:
        0 on success; 2 on unknown strategy/objective; 3 if no input arrived.
    """
    parser = argparse.ArgumentParser(description="Generic ADCS control loop.")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    parser.add_argument("--duration", type=float, default=30.0, help="run time [s]; 0 = forever")
    parser.add_argument("--rate", type=float, default=None, help="override control rate [Hz]")
    parser.add_argument("--fifo", type=int, default=None, help="SCHED_FIFO priority to attempt")
    parser.add_argument("--pin", type=int, default=None, help="CPU core to pin to")
    parser.add_argument("--report-every", type=float, default=0.0, help="jitter report period [s]")
    parser.add_argument("--status-every", type=float, default=5.0, help="status line period [s]")
    parser.add_argument(
        "--spawn-sensors", action="store_true", help="spawn the vehicle's sensor daemons"
    )
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    if args.rate is not None:
        vehicle = VehicleSpec(
            name=vehicle.name,
            description=vehicle.description,
            sensors=vehicle.sensors,
            control=ControlEntry(
                strategy=vehicle.control.strategy,
                objective=vehicle.control.objective,
                rate_hz=args.rate,
                input_topic=vehicle.control.input_topic,
                output_topic=vehicle.control.output_topic,
                stale_after_s=vehicle.control.stale_after_s,
                options=vehicle.control.options,
                objective_options=vehicle.control.objective_options,
            ),
            provenance=vehicle.provenance,
        )

    children: list[subprocess.Popen[bytes]] = []
    if args.spawn_sensors:
        for sensor in vehicle.sensors:
            children.append(
                subprocess.Popen(
                    [sys.executable, "-m", "flight.apps.sensor_daemon", "--sensor", sensor.name],
                    cwd=REPO_ROOT,
                )
            )

    session = zenoh.open(zenoh.Config())
    try:
        loop = build_loop(session, vehicle)
    except KeyError as exc:
        print(f"cannot compose control loop: {exc}", file=sys.stderr)
        session.close()
        return 2

    conditions = [
        try_fifo(args.fifo),
        try_mlockall(),
        pin_to_core(args.pin),
        quiesce_gc(),
        *describe_actual(),
        *vehicle.describe(),
        *loop._controller.describe(),  # noqa: SLF001 — echoing configuration
        *loop._guidance.describe(),  # noqa: SLF001 — echoing configuration
    ]
    try:
        if not loop.wait_for_first_sample():
            print(f"no data on {vehicle.control.input_topic} — start sensors or --spawn-sensors")
            return 3
        report_every = args.report_every
        if report_every <= 0 and args.duration <= 0:
            report_every = 60.0  # service mode must bound memory
        report = loop.run(
            args.duration,
            report_every_s=report_every,
            conditions=conditions,
            status_every_s=args.status_every,
        )
        report.print_summary()
    finally:
        loop.close()
        session.close()
        for child in children:
            child.terminate()
            child.wait(timeout=5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
