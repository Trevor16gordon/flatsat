#!/usr/bin/env python3
"""Generic actuator daemon: runs ANY actuator named in a vehicle composition.

One executable serves every actuator on the spacecraft. It owns the parts
that are identical for all of them — command subscription, cadence, the
mounting projection, state publishing, health telemetry, lifecycle — and
delegates the device-specific parts, ``apply()`` and ``state()``, to a
driver resolved by name from the vehicle file.

Two protections every actuator inherits from THIS daemon, never from the
driver (learned from the first HIL run, where a dead controller's last
torque spun the simulated vehicle back up):

  * **Stale-command zeroing.** If no fresh body-frame command has arrived
    within ``stale_zero_s``, the daemon applies ZERO. The quiet state is
    the default; only a live controller makes an actuator interesting.
  * **Mounting projection.** The controller publishes body-frame torque;
    the daemon projects it through this device's own mounting geometry
    before the driver sees it. No central mixer until joint allocation is
    actually needed.

Cadence uses the absolute-deadline pattern (the next wake time advances by
exactly one period), so scheduling lateness never accumulates into drift.

Usage:
  python -m flatsat.apps.actuator_daemon --actuator wheel0
  python -m flatsat.apps.actuator_daemon --actuator wheel0 --vehicle other.txtpb
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import zenoh

from flatsat import vehicle_pb2
from flatsat.core.bus import SamplePublisher
from flatsat.core.config import load_vehicle, which_impl
from flatsat.core.health import health_topic
from flatsat.core.registry import get_actuator_class
from flatsat.hardware.actuator import ActuatorDriver, project_body_torque
from flatsat.mode.client import ModeClient
from flatsat.msgs import adcs_pb2, health_pb2


class ActuatorDaemon:
    """Applies bus commands to one driver and publishes its state."""

    def __init__(
        self, entry: vehicle_pb2.ActuatorConfig, driver: ActuatorDriver, session: zenoh.Session
    ) -> None:
        """Bind a driver to its topics and cadence.

        Args:
            entry: Vehicle-file actuator entry (topics, rate, mounting,
                stale-zero threshold).
            driver: Device driver to command.
            session: Open zenoh session; not owned — caller closes it.
        """
        self.entry = entry
        self.driver_name = which_impl(entry, "options", entry.name)
        self._driver = driver
        self._state_pub = SamplePublisher(session, entry.state_topic, entry.name)
        self._health = SamplePublisher(session, health_topic(entry.name), entry.name)
        self._stale_zero_ns = int(entry.stale_zero_s * 1e9)
        self._lock = threading.Lock()
        self._torque_body = (0.0, 0.0, 0.0)
        self._recv_ns = 0
        self.commands_received = 0
        self._stop = threading.Event()
        self._window_cycles = 0
        self._stale_zeroed_cycles = 0
        self._flagged_cycles = 0
        self._flag_union = 0
        self._sub = session.declare_subscriber(entry.command_topic, self._on_command)

    def _on_command(self, sample: zenoh.Sample) -> None:
        """Store the newest body-frame command (subscriber thread).

        The message type follows the driver's ``command_kind``: a wheel
        consumes WheelTorqueCommand (N·m), a magnetorquer DipoleCommand
        (A·m²). Either way it is a body-frame vector the daemon projects
        through this device's mounting.

        Args:
            sample: Incoming command message.
        """
        payload = bytes(sample.payload.to_bytes())
        if self._driver.command_kind == "dipole":
            dipole = adcs_pb2.DipoleCommand.FromString(payload)
            command = (dipole.dipole_x_a_m2, dipole.dipole_y_a_m2, dipole.dipole_z_a_m2)
        else:
            torque = adcs_pb2.WheelTorqueCommand.FromString(payload)
            command = (torque.torque_x_n_m, torque.torque_y_n_m, torque.torque_z_n_m)
        with self._lock:
            self._torque_body = command
            self._recv_ns = time.monotonic_ns()
            self.commands_received += 1

    def commanded_axis_torque(self) -> tuple[float, bool]:
        """Resolve what the device should do RIGHT NOW.

        Returns:
            Tuple of (effort about the device axis, in the driver's own
            command units, and whether the command was stale). The stale
            case is always (0.0, True): an actuator must never keep
            flying a dead controller's last order.
        """
        with self._lock:
            torque_body = self._torque_body
            age_ns = time.monotonic_ns() - self._recv_ns
        if self._recv_ns == 0 or age_ns > self._stale_zero_ns:
            return 0.0, True
        return project_body_torque(self.entry.mounting, torque_body), False

    def apply_once(self) -> None:
        """Run one apply-and-publish cycle."""
        torque, stale_zeroed = self.commanded_axis_torque()
        apply_flags = self._driver.apply(torque)
        state_msg, state_flags = self._driver.state()
        flags = apply_flags | state_flags
        self._state_pub.publish(state_msg, validity=flags)
        self._window_cycles += 1
        self._stale_zeroed_cycles += int(stale_zeroed)
        if flags:
            self._flagged_cycles += 1
            self._flag_union |= flags

    def publish_health(self) -> health_pb2.ActuatorHealth:
        """Publish and reset this window's actuation statistics.

        Stale-zeroed counts are how a recorded window shows the protection
        actually engaging (a controller crash looks like a rising
        stale_zeroed_cycles, not silence).

        Returns:
            The published message, for tests and introspection.
        """
        msg = health_pb2.ActuatorHealth()
        msg.driver = self.driver_name
        msg.rate_hz = self.entry.rate_hz
        msg.window_cycles = self._window_cycles
        msg.stale_zeroed_cycles = self._stale_zeroed_cycles
        msg.flagged_cycles = self._flagged_cycles
        msg.flag_union = self._flag_union
        self._window_cycles = 0
        self._stale_zeroed_cycles = 0
        self._flagged_cycles = 0
        self._flag_union = 0
        return self._health.publish(msg)  # type: ignore[return-value]

    def run(self, health_every_s: float = 30.0) -> None:
        """Apply at the configured cadence until :meth:`stop` is called.

        Args:
            health_every_s: Interval between ActuatorHealth publications;
                <= 0 disables health reporting.
        """
        period_ns = int(1_000_000_000 / self.entry.rate_hz)
        health_ns = int(health_every_s * 1e9) if health_every_s > 0 else None
        next_wake = time.monotonic_ns() + period_ns
        next_health = next_wake + health_ns if health_ns else None
        while not self._stop.is_set():
            self.apply_once()
            now = time.monotonic_ns()
            if next_health is not None and health_ns is not None and now >= next_health:
                self.publish_health()
                next_health += health_ns
            delta_ns = next_wake - now
            if delta_ns > 0:
                self._stop.wait(delta_ns / 1e9)
            next_wake += period_ns

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources and release the driver's device."""
        self._sub.undeclare()
        self._driver.close()


def build_daemon(
    actuator_name: str, session: zenoh.Session, vehicle_path: Path | None = None
) -> ActuatorDaemon:
    """Compose a daemon for one actuator of a vehicle.

    Args:
        actuator_name: Actuator instance name in the vehicle file.
        session: Open zenoh session.
        vehicle_path: Vehicle composition file; default when omitted.

    Returns:
        The composed daemon, ready to run.
    """
    vehicle = load_vehicle(vehicle_path)
    entry = vehicle.actuator(actuator_name)
    driver_name = which_impl(entry, "options", entry.name)
    driver_cls = get_actuator_class(driver_name)
    driver = driver_cls.from_config(entry.name, getattr(entry, driver_name))
    for line in (*vehicle.describe(), *driver.describe()):
        print(f"[{entry.name}] {line}", flush=True)
    print(
        f"[{entry.name}] commands {entry.command_topic} -> axis {tuple(entry.mounting.axis)}, "
        f"state on {entry.state_topic} at {entry.rate_hz:g} Hz, "
        f"stale-zero after {entry.stale_zero_s:g} s",
        flush=True,
    )
    return ActuatorDaemon(entry, driver, session)


def main() -> int:
    """Run one actuator daemon from the command line.

    Returns:
        0 on clean exit; 2 if the actuator or driver is unknown.
    """
    parser = argparse.ArgumentParser(description="Generic actuator daemon.")
    parser.add_argument("--actuator", required=True, help="actuator name in the vehicle file")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    args = parser.parse_args()

    session = zenoh.open(zenoh.Config())
    try:
        daemon = build_daemon(args.actuator, session, args.vehicle)
    except (KeyError, FileNotFoundError) as exc:
        print(f"cannot start actuator {args.actuator!r}: {exc}", file=sys.stderr)
        session.close()
        return 2
    # Join the mode contract: late-join query + ack every transition.
    mode_client = ModeClient(session, args.actuator)
    try:
        daemon.run()
    except KeyboardInterrupt:
        daemon.stop()
    finally:
        mode_client.close()
        daemon.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
