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
  python -m flight.apps.sensor_daemon --sensor imu0
  python -m flight.apps.sensor_daemon --sensor thermal_tj --vehicle other.toml
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import zenoh

from flight.core.bus import SamplePublisher
from flight.core.config import SensorEntry, load_vehicle
from flight.hal.driver import SensorDriver
from flight.registry import get_driver_class


class SensorDaemon:
    """Publishes one driver's readings on the bus at a fixed cadence."""

    def __init__(self, entry: SensorEntry, driver: SensorDriver, session: zenoh.Session) -> None:
        """Bind a driver to its topic and cadence.

        Args:
            entry: Vehicle-file sensor entry (name, topic, rate).
            driver: Device driver to poll.
            session: Open zenoh session; not owned — caller closes it.
        """
        self.entry = entry
        self._driver = driver
        self._publisher = SamplePublisher(session, entry.topic, entry.name)
        self._stop = threading.Event()

    def publish_once(self) -> None:
        """Acquire, stamp, and publish one sample."""
        sample_time_ns = time.time_ns()
        msg, flags = self._driver.read()
        self._publisher.publish(msg, validity=flags, sample_time_ns=sample_time_ns)

    def run(self) -> None:
        """Publish at the configured cadence until :meth:`stop` is called."""
        period_ns = int(1_000_000_000 / self.entry.rate_hz)
        next_wake = time.monotonic_ns() + period_ns
        while not self._stop.is_set():
            self.publish_once()
            delta_ns = next_wake - time.monotonic_ns()
            if delta_ns > 0:
                self._stop.wait(delta_ns / 1e9)
            next_wake += period_ns

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()


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
    driver_cls = get_driver_class(entry.driver)
    driver = driver_cls.from_config(entry.name, entry.options)
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

    session = zenoh.open(zenoh.Config())
    try:
        daemon = build_daemon(args.sensor, session, args.vehicle)
    except (KeyError, FileNotFoundError) as exc:
        print(f"cannot start sensor {args.sensor!r}: {exc}", file=sys.stderr)
        session.close()
        return 2
    try:
        daemon.run()
    except KeyboardInterrupt:
        daemon.stop()
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
