#!/usr/bin/env python3
"""Generic sensor daemon: runs ANY driver named in a vehicle composition.

One executable serves every sensor on the spacecraft. It owns the parts that
are identical for all of them — cadence, header stamping, publishing,
lifecycle — and delegates the only device-specific part, ``read()``, to a
driver resolved by name from the vehicle file. Adding a sensor is a vehicle
file entry plus a driver module; no new application, no copy-pasted loop.

Cadence uses the absolute-deadline pattern (the next wake time advances by
exactly one period), so scheduling lateness never accumulates into drift.

Usage:
  python -m flatsat.apps.sensor_daemon --sensor imu0
  python -m flatsat.apps.sensor_daemon --sensor thermal_tj --vehicle other.txtpb
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import zenoh

from flatsat import vehicle_pb2
from flatsat.core.bus import SamplePublisher, bus_config
from flatsat.core.config import load_vehicle, which_impl
from flatsat.core.health import health_topic
from flatsat.core.registry import get_driver_class
from flatsat.hardware.sensor import SensorDriver
from flatsat.mode.client import ModeClient
from flatsat.msgs import health_pb2


class SensorDaemon:
    """Publishes one driver's readings on the bus at a fixed cadence."""

    def __init__(
        self, entry: vehicle_pb2.SensorConfig, driver: SensorDriver, session: zenoh.Session
    ) -> None:
        """Bind a driver to its topic and cadence.

        Args:
            entry: Vehicle-file sensor entry (name, topic, rate).
            driver: Device driver to poll.
            session: Open zenoh session; not owned — caller closes it.
        """
        self.entry = entry
        self.driver_name = which_impl(entry, "options", entry.name)
        self._driver = driver
        self._publisher = SamplePublisher(session, entry.topic, entry.name)
        self._health = SamplePublisher(session, health_topic(entry.name), entry.name)
        self._stop = threading.Event()
        self._window_samples = 0
        self._flagged_samples = 0
        self._flag_union = 0

    def publish_once(self) -> None:
        """Acquire, stamp, and publish one sample."""
        sample_time_ns = time.time_ns()
        msg, flags = self._driver.read()
        self._publisher.publish(msg, validity=flags, sample_time_ns=sample_time_ns)
        self._window_samples += 1
        if flags:
            self._flagged_samples += 1
            self._flag_union |= flags

    def publish_health(self) -> health_pb2.SensorHealth:
        """Publish and reset this window's acquisition statistics.

        Flag counts are the honest record of how the device actually
        behaved — a zone that EAGAINs half the time, a gyro that rails —
        which is exactly what FDIR limit-checks and the anomaly corpus
        learn from. Printing them would throw that away.

        Returns:
            The published message, for tests and introspection.
        """
        msg = health_pb2.SensorHealth()
        msg.driver = self.driver_name
        msg.rate_hz = self.entry.rate_hz
        msg.window_samples = self._window_samples
        msg.flagged_samples = self._flagged_samples
        msg.flag_union = self._flag_union
        self._window_samples = 0
        self._flagged_samples = 0
        self._flag_union = 0
        return self._health.publish(msg)  # type: ignore[return-value]

    def run(self, health_every_s: float = 30.0) -> None:
        """Publish at the configured cadence until :meth:`stop` is called.

        Args:
            health_every_s: Interval between SensorHealth publications;
                <= 0 disables health reporting.
        """
        period_ns = int(1_000_000_000 / self.entry.rate_hz)
        health_ns = int(health_every_s * 1e9) if health_every_s > 0 else None
        next_wake = time.monotonic_ns() + period_ns
        next_health = next_wake + health_ns if health_ns else None
        while not self._stop.is_set():
            self.publish_once()
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
        """Release the driver's device resources."""
        self._driver.close()


def build_daemon(
    sensor_name: str, session: zenoh.Session, vehicle_path: Path | None = None
) -> SensorDaemon:
    """Compose a daemon for one sensor of a vehicle.

    Args:
        sensor_name: Sensor instance name in the vehicle file.
        session: Open zenoh session.
        vehicle_path: Vehicle composition file; default when omitted.

    Returns:
        The composed daemon, ready to run.
    """
    vehicle = load_vehicle(vehicle_path)
    entry = vehicle.sensor(sensor_name)
    driver_name = which_impl(entry, "options", entry.name)
    driver_cls = get_driver_class(driver_name)
    driver = driver_cls.from_config(entry.name, getattr(entry, driver_name))
    for line in (*vehicle.describe(), *driver.describe()):
        print(f"[{entry.name}] {line}", flush=True)
    print(f"[{entry.name}] publishing {entry.topic} at {entry.rate_hz:g} Hz", flush=True)
    return SensorDaemon(entry, driver, session)


def main() -> int:
    """Run one sensor daemon from the command line.

    Returns:
        0 on clean exit; 2 if the sensor or driver is unknown.
    """
    parser = argparse.ArgumentParser(description="Generic HAL sensor daemon.")
    parser.add_argument("--sensor", required=True, help="sensor name in the vehicle file")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    args = parser.parse_args()

    session = zenoh.open(bus_config())
    try:
        daemon = build_daemon(args.sensor, session, args.vehicle)
    except (KeyError, FileNotFoundError) as exc:
        print(f"cannot start sensor {args.sensor!r}: {exc}", file=sys.stderr)
        session.close()
        return 2
    # Join the mode contract: late-join query + ack every transition.
    # Behavior per mode arrives with the mode table; acking is the
    # liveness signal the manager's missing-ack fault detection needs.
    mode_client = ModeClient(session, args.sensor)
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
