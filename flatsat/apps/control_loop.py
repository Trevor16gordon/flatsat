#!/usr/bin/env python3
"""Generic control loop: runs ANY controller named in a vehicle composition.

The imperative shell around a control strategy. It owns everything that is
true of every controller — subscribing to the state input, real-time
hygiene, absolute-deadline cadence, actuator publishing, self-measurement —
and delegates the strategy-specific part to an
:class:`~flatsat.control.attitude.controller.AttitudeController` resolved by name, with
its target supplied by a :class:`~flatsat.control.attitude.guidance.ReferenceSource`.

Swapping PD for PID for an ML policy, or detumble for a pointing objective,
is a vehicle-file edit. This file does not change.

Self-measurement mirrors cyclictest: wakeup lateness, control-step
execution time, stale-input cycles, and actuator saturation, reported as
percentiles on an interval (bounded memory for indefinite service runs).

Usage:
  python -m flatsat.apps.control_loop --duration 20
  python -m flatsat.apps.control_loop --spawn-sensors --duration 20
  sudo ... --fifo 80 --pin 3          # RT flavor (systemd normally grants)
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from functools import partial
from pathlib import Path

import numpy as np
import zenoh

from flatsat import vehicle_pb2
from flatsat.control.attitude.controller import AttitudeController
from flatsat.control.attitude.estimators.estimator import StateEstimator
from flatsat.control.attitude.guidance import ReferenceSource
from flatsat.core.bus import SamplePublisher
from flatsat.core.config import VehicleSpec, load_vehicle, which_impl
from flatsat.core.health import health_topic, percentiles
from flatsat.core.registry import get_controller_class, get_estimator_class, get_guidance_class
from flatsat.core.rt import describe_actual, pin_to_core, quiesce_gc, try_fifo, try_mlockall
from flatsat.mode.client import ModeClient
from flatsat.msgs import adcs_pb2, hal_pb2, health_pb2

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

    def to_proto(self, loop: ControlLoop, environment: str, affinity: str) -> health_pb2.LoopHealth:
        """Render this window as a LoopHealth message.

        Args:
            loop: The loop that produced the window (for composition
                provenance).
            environment: Verified scheduling policy/priority string.
            affinity: Verified CPU affinity string.

        Returns:
            The health message, unstamped (the publisher fills the header).
        """
        msg = health_pb2.LoopHealth()
        msg.vehicle = loop.vehicle_name
        msg.config_checksum = loop.config_checksum
        msg.strategy = loop.strategy_name
        msg.objective = loop.objective_name
        msg.rate_hz = loop.entry.rate_hz
        msg.scheduling = environment
        msg.cpu_affinity = affinity
        msg.window_cycles = self.cycles
        msg.stale_cycles = self.stale_cycles
        msg.saturated_cycles = self.saturated_cycles
        msg.wakeup_lateness_us.CopyFrom(percentiles(self.wakeup_lateness_us))
        msg.exec_time_us.CopyFrom(percentiles(self.exec_time_us))
        return msg

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
        entry: vehicle_pb2.ControlConfig,
        controller: AttitudeController,
        guidance: ReferenceSource,
        estimator: StateEstimator,
        vehicle_name: str = "",
        config_checksum: str = "",
        wheels: list[tuple[str, tuple[float, float, float]]] | None = None,
    ) -> None:
        """Wire the loop to its topics, strategy, objective, and estimator.

        Args:
            session: Open zenoh session.
            entry: Control composition (rates, topics, thresholds).
            controller: Strategy computing torque.
            guidance: Source of the reference to track.
            estimator: Estimator turning measurements into the state the
                controller acts on.
            vehicle_name: Composition name, echoed into health telemetry.
            config_checksum: Composition checksum, echoed into health
                telemetry so a recorded window traces to its parameters.
            wheels: (state_topic, body-frame spin axis) per reaction
                wheel — the momentum-dump input. The loop subscribes to
                each wheel's own state topic and hands the strategy the
                summed body-frame momentum vector; None or empty means
                no strategy on this vehicle needs it.

        Raises:
            ValueError: If the strategy emits torque AND dipole but the
                composition names no dipole_output_topic — a dump law
                whose dipole silently went nowhere would look exactly
                like a broken one.
        """
        self.entry = entry
        self.strategy_name = which_impl(entry, "strategy", "control")
        self.objective_name = which_impl(entry, "objective", "control")
        # Validate the composition BEFORE declaring any bus resources, so
        # a refused loop leaves nothing behind to undeclare.
        if type(controller).output_kind == "torque_and_dipole" and not entry.dipole_output_topic:
            raise ValueError(
                f"strategy {self.strategy_name!r} emits torque and dipole but the "
                "control block names no dipole_output_topic"
            )
        self.vehicle_name = vehicle_name
        self.config_checksum = config_checksum
        self._health = SamplePublisher(session, health_topic("adcs"), "adcs_loop")
        self._controller = controller
        self._guidance = guidance
        self._estimator = estimator
        self._stale_after_ns = int(entry.stale_after_s * 1e9)
        self._pub = session.declare_publisher(entry.output_topic)
        self._sub = session.declare_subscriber(entry.input_topic, self._on_sample)
        self._latest: hal_pb2.ImuSample | None = None
        self._latest_recv_ns = 0
        # Optional magnetometer side input: the loop still paces on the
        # main input; the field rides along on the state it hands the
        # strategy (magnetic laws need B, others never see it).
        self._mag_sub = (
            session.declare_subscriber(entry.mag_input_topic, self._on_mag)
            if entry.mag_input_topic
            else None
        )
        self._latest_mag: hal_pb2.MagnetometerSample | None = None
        self._latest_mag_recv_ns = 0
        # Optional sun sensor side input, same contract as the mag.
        self._sun_sub = (
            session.declare_subscriber(entry.sun_input_topic, self._on_sun)
            if entry.sun_input_topic
            else None
        )
        self._latest_sun: hal_pb2.SunSensorSample | None = None
        self._latest_sun_recv_ns = 0
        # Optional dipole output: a strategy that emits torque AND dipole
        # (momentum_dump) publishes the dipole here.
        self._dipole_pub = (
            session.declare_publisher(entry.dipole_output_topic)
            if entry.dipole_output_topic
            else None
        )
        # Optional wheel-momentum side input, one subscriber per wheel.
        self._wheel_axes: dict[str, tuple[float, float, float]] = dict(wheels or [])
        self._wheel_momentum: dict[str, tuple[float, int]] = {}
        self._wheel_subs = [
            session.declare_subscriber(topic, partial(self._on_wheel_state, topic))
            for topic, _ in (wheels or [])
        ]
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_command = (0.0, 0.0, 0.0)
        self.samples_received = 0
        self.scheduling = ""
        self.cpu_affinity = ""

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

    def _on_mag(self, sample: zenoh.Sample) -> None:
        """Store the latest magnetometer sample (subscriber thread).

        Args:
            sample: Incoming MagnetometerSample.
        """
        msg = hal_pb2.MagnetometerSample.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._latest_mag = msg
            self._latest_mag_recv_ns = time.monotonic_ns()

    def _on_sun(self, sample: zenoh.Sample) -> None:
        """Store the latest sun sensor sample (subscriber thread).

        Args:
            sample: Incoming SunSensorSample.
        """
        msg = hal_pb2.SunSensorSample.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._latest_sun = msg
            self._latest_sun_recv_ns = time.monotonic_ns()

    def _on_wheel_state(self, topic: str, sample: zenoh.Sample) -> None:
        """Store one wheel's latest stored momentum (subscriber thread).

        Args:
            topic: The wheel state topic the sample arrived on.
            sample: Incoming WheelState.
        """
        msg = hal_pb2.WheelState.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._wheel_momentum[topic] = (msg.momentum_n_m_s, time.monotonic_ns())

    def _wheel_momentum_body(self) -> tuple[float, float, float] | None:
        """Sum the wheels' stored momentum into a body-frame vector.

        Caller must hold the lock.

        Returns:
            The body-frame momentum, or None unless EVERY wheel has
            reported freshly — a dump law fed a partial sum would bleed
            momentum that is not there.
        """
        if not self._wheel_axes:
            return None
        now = time.monotonic_ns()
        total = [0.0, 0.0, 0.0]
        for topic, axis in self._wheel_axes.items():
            entry = self._wheel_momentum.get(topic)
            if entry is None or now - entry[1] > self._stale_after_ns:
                return None
            for index in range(3):
                total[index] += axis[index] * entry[0]
        return (total[0], total[1], total[2])

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
            mag = self._latest_mag
            mag_age_ns = time.monotonic_ns() - self._latest_mag_recv_ns
            sun = self._latest_sun
            sun_age_ns = time.monotonic_ns() - self._latest_sun_recv_ns
            wheel_momentum = self._wheel_momentum_body()
        assert imu is not None  # guaranteed by wait_for_first_sample
        stale = age_ns > self._stale_after_ns
        # Estimator-grade inputs: fresh by age AND unflagged. TRIAD fed a
        # range-flagged field would fold garbage into an attitude.
        mag_fresh_sample = (
            mag
            if (
                mag is not None
                and mag_age_ns <= self._stale_after_ns
                and mag.header.validity == hal_pb2.VALIDITY_FLAG_VALID
            )
            else None
        )
        sun_fresh_sample = (
            sun
            if (
                sun is not None
                and sun_age_ns <= self._stale_after_ns
                and sun.header.validity == hal_pb2.VALIDITY_FLAG_VALID
            )
            else None
        )

        dt_s = 1.0 / self.entry.rate_hz
        state = self._estimator.update(
            imu,
            age_ns / 1e9,
            not stale,
            dt_s,
            mag=mag_fresh_sample,
            sun=sun_fresh_sample,
        )
        if self._mag_sub is not None and mag is not None:
            state = replace(
                state,
                mag_field_t=(mag.mag_x_t, mag.mag_y_t, mag.mag_z_t),
                mag_fresh=(
                    mag_age_ns <= self._stale_after_ns
                    and mag.header.validity == hal_pb2.VALIDITY_FLAG_VALID
                ),
            )
        if self._sun_sub is not None and sun_fresh_sample is not None:
            state = replace(
                state,
                sun_body=(sun_fresh_sample.sun_x, sun_fresh_sample.sun_y, sun_fresh_sample.sun_z),
                sun_visible=sun_fresh_sample.sun_visible,
            )
        if wheel_momentum is not None:
            state = replace(state, wheel_momentum_n_m_s=wheel_momentum)
        output = self._controller.update(state, self._guidance.reference_at(t_s), dt_s)

        header_validity = int(hal_pb2.VALIDITY_FLAG_STALE) if stale else 0
        kind = type(self._controller).output_kind
        if kind == "dipole":
            self._last_command = output.dipole_a_m2
            self._publish_dipole(self._pub, output.dipole_a_m2, seq, header_validity)
            return stale, output.saturated

        self._last_command = output.torque_n_m
        cmd = adcs_pb2.WheelTorqueCommand()
        cmd.torque_x_n_m, cmd.torque_y_n_m, cmd.torque_z_n_m = output.torque_n_m
        cmd.header.source = "adcs_loop"
        cmd.header.seq = seq
        cmd.header.sample_time_ns = time.time_ns()
        cmd.header.publish_time_ns = time.time_ns()
        cmd.header.validity = header_validity
        self._pub.put(cmd.SerializeToString())
        if kind == "torque_and_dipole" and self._dipole_pub is not None:
            self._publish_dipole(self._dipole_pub, output.dipole_a_m2, seq, header_validity)
        return stale, output.saturated

    def _publish_dipole(
        self,
        publisher: zenoh.Publisher,
        dipole_a_m2: tuple[float, float, float],
        seq: int,
        validity: int,
    ) -> None:
        """Publish one body-frame dipole command.

        Args:
            publisher: Where it goes (main or dipole output).
            dipole_a_m2: The commanded body dipole.
            seq: Command sequence number.
            validity: Header validity word (STALE when inputs were).
        """
        cmd = adcs_pb2.DipoleCommand()
        cmd.dipole_x_a_m2, cmd.dipole_y_a_m2, cmd.dipole_z_a_m2 = dipole_a_m2
        cmd.header.source = "adcs_loop"
        cmd.header.seq = seq
        cmd.header.sample_time_ns = time.time_ns()
        cmd.header.publish_time_ns = time.time_ns()
        cmd.header.validity = validity
        publisher.put(cmd.SerializeToString())

    def print_status(self) -> None:
        """Print a one-line live status: rates, input freshness, command out.

        The command line is labeled in the strategy's own units — an
        operator watching a magnetic vehicle must not read dipoles as
        newton-meters.
        """
        with self._lock:
            imu = self._latest
            age_ms = (time.monotonic_ns() - self._latest_recv_ns) / 1e6
            rx = self.samples_received
        if imu is None:
            print("[adcs] no sensor data yet", flush=True)
            return
        omega = (imu.gyro_x_rad_s, imu.gyro_y_rad_s, imu.gyro_z_rad_s)
        mag_mrad = 1000.0 * math.sqrt(sum(w * w for w in omega))
        cx, cy, cz = self._last_command
        label, units = (
            ("dipole", "A·m²")
            if type(self._controller).output_kind == "dipole"
            else ("torque", "N·m")
        )
        print(
            f"[adcs] |omega|={mag_mrad:7.2f} mrad/s  "
            f"{label}=({cx:+.2e},{cy:+.2e},{cz:+.2e}) {units}  "
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
                self.publish_health(report)
                report = LoopReport(conditions=report.conditions)
                next_report += report_ns
        return report

    def publish_health(self, report: LoopReport) -> None:
        """Publish one window of loop health on the bus.

        Args:
            report: The window to publish.
        """
        self._health.publish(report.to_proto(self, self.scheduling, self.cpu_affinity))

    def stop(self) -> None:
        """Request the loop to exit (thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        self._sub.undeclare()
        if self._mag_sub is not None:
            self._mag_sub.undeclare()
        if self._sun_sub is not None:
            self._sun_sub.undeclare()
        for sub in self._wheel_subs:
            sub.undeclare()


def build_loop(session: zenoh.Session, vehicle: VehicleSpec) -> ControlLoop:
    """Compose the control loop declared by a vehicle.

    Args:
        session: Open zenoh session.
        vehicle: Loaded vehicle composition.

    Returns:
        The composed loop, ready to run.
    """
    entry = vehicle.control
    strategy = which_impl(entry, "strategy", "control")
    objective = which_impl(entry, "objective", "control")
    estimator_name = which_impl(entry, "estimator", "control")
    controller = get_controller_class(strategy).from_config(getattr(entry, strategy))
    guidance = get_guidance_class(objective).from_config(getattr(entry, objective))
    estimator = get_estimator_class(estimator_name).from_config(getattr(entry, estimator_name))
    # Wheel-state inputs cost a subscription and lock traffic per wheel —
    # only strategies that consume the momentum pay for it.
    needs_momentum = type(controller).output_kind == "torque_and_dipole"
    return ControlLoop(
        session,
        entry,
        controller,
        guidance,
        estimator,
        vehicle_name=vehicle.name,
        config_checksum=vehicle.provenance.checksum,
        wheels=wheel_state_inputs(vehicle) if needs_momentum else None,
    )


def wheel_state_inputs(vehicle: VehicleSpec) -> list[tuple[str, tuple[float, float, float]]]:
    """The wheel state topics and axes a momentum-aware strategy needs.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        (state_topic, body-frame spin axis) per declared reaction wheel.
    """
    return [
        (
            entry.state_topic,
            (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2]),
        )
        for entry in vehicle.actuators
        if which_impl(entry, "options", entry.name).endswith("reaction_wheel")
    ]


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
        vehicle.config.control.rate_hz = args.rate  # protos are mutable views

    children: list[subprocess.Popen[bytes]] = []
    if args.spawn_sensors:
        for sensor in vehicle.sensors:
            children.append(
                subprocess.Popen(
                    [sys.executable, "-m", "flatsat.apps.sensor_daemon", "--sensor", sensor.name],
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
    # Join the mode contract: late-join query + ack every transition.
    mode_client = ModeClient(session, "adcs")

    verified = describe_actual()
    loop.scheduling = verified[0].removeprefix("verified: ")
    loop.cpu_affinity = verified[1].removeprefix("verified: affinity cpus ")
    conditions = [
        try_fifo(args.fifo),
        try_mlockall(),
        pin_to_core(args.pin),
        quiesce_gc(),
        *verified,
        *vehicle.describe(),
        *loop._controller.describe(),  # noqa: SLF001 — echoing configuration
        *loop._guidance.describe(),  # noqa: SLF001 — echoing configuration
        *loop._estimator.describe(),  # noqa: SLF001 — echoing configuration
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
        mode_client.close()
        loop.close()
        session.close()
        for child in children:
            child.terminate()
            child.wait(timeout=5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
