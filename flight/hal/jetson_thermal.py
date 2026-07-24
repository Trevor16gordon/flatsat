#!/usr/bin/env python3
"""Jetson onboard thermal-zone daemon — the first real HAL sensor.

Reads one kernel thermal zone (``/sys/class/thermal/thermal_zone*/temp``,
selected by type name, e.g. ``tj-thermal`` = max junction temperature) and
publishes ``TemperatureSample`` on the bus at the configured rate.

Known real-world behavior this daemon demonstrates: the ``cv*-thermal``
zones EAGAIN while the CV accelerator cluster is power-gated — a genuine
acquisition failure. Per the contract, the daemon still publishes on
cadence with ``VALIDITY_FLAG_COMM`` set (flag and forward, never repair).

Usage:
  ./jetson_thermal.py                       # tj-thermal at 1 Hz, forever
  ./jetson_thermal.py --zone cpu-thermal --rate 5 --count 10
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import zenoh

from flight.hal.daemon import HalMessage, SensorConfig, SensorDaemon
from flight.msgs import hal_pb2

THERMAL_ROOT = Path("/sys/class/thermal")


def find_zone_temp_path(zone_type: str, root: Path = THERMAL_ROOT) -> Path:
    """Resolve a thermal zone type name to its ``temp`` file.

    Args:
        zone_type: Kernel zone type, e.g. ``tj-thermal`` or ``cpu-thermal``.
        root: Thermal sysfs root (overridable for tests).

    Returns:
        Path to the zone's ``temp`` file.

    Raises:
        FileNotFoundError: If no zone of that type exists.
    """
    for zone_dir in sorted(root.glob("thermal_zone*")):
        try:
            if (zone_dir / "type").read_text().strip() == zone_type:
                return zone_dir / "temp"
        except OSError:
            continue
    raise FileNotFoundError(f"no thermal zone of type {zone_type!r} under {root}")


class JetsonThermalDaemon(SensorDaemon):
    """Publishes one Jetson thermal zone as TemperatureSample."""

    def __init__(self, config: SensorConfig, session: zenoh.Session, zone_type: str) -> None:
        """Resolve the zone once at startup and bind to the bus.

        Args:
            config: Instance configuration (name, topic, rate).
            session: Open zenoh session.
            zone_type: Kernel thermal zone type name to read.
        """
        super().__init__(config, session)
        self._zone_type = zone_type
        self._temp_path = find_zone_temp_path(zone_type)

    def read(self) -> tuple[HalMessage, int]:
        """Read the zone once; EAGAIN/garbage becomes a COMM-flagged sample.

        Returns:
            Tuple of (TemperatureSample, validity flags).
        """
        msg = hal_pb2.TemperatureSample()
        msg.location = self._zone_type
        try:
            # Raw os.read, deliberately: sysfs EAGAIN (power-gated zone) is a
            # clean BlockingIOError here. Python's buffered/text IO layers
            # instead yield None (binary) or TypeError from the codecs module
            # (text) on EAGAIN — both evade OSError handlers and killed the
            # daemon in the first version of this function.
            fd = os.open(self._temp_path, os.O_RDONLY)
            try:
                raw = os.read(fd, 32)
            finally:
                os.close(fd)
            millideg = int(raw.decode("ascii").strip())
        except (OSError, ValueError):
            return msg, hal_pb2.VALIDITY_FLAG_COMM
        msg.temperature_c = millideg / 1000.0
        return msg, hal_pb2.VALIDITY_FLAG_VALID


def main() -> int:
    """Run the thermal daemon from the command line.

    Returns:
        0 on clean exit; 2 if the requested zone does not exist.
    """
    parser = argparse.ArgumentParser(description="Jetson thermal-zone HAL daemon.")
    parser.add_argument("--zone", default="tj-thermal", help="thermal zone type name")
    parser.add_argument("--rate", type=float, default=1.0, help="publish rate [Hz]")
    parser.add_argument(
        "--count", type=int, default=0, help="samples to publish then exit (0 = run forever)"
    )
    args = parser.parse_args()

    name = f"thermal_{args.zone.removesuffix('-thermal')}"
    config = SensorConfig(name=name, topic=f"hal/{name}/sample", rate_hz=args.rate)
    session = zenoh.open(zenoh.Config())
    try:
        daemon = JetsonThermalDaemon(config, session, args.zone)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        session.close()
        return 2

    print(f"[{name}] publishing {args.zone} on {config.topic} at {args.rate:g} Hz")
    try:
        if args.count > 0:
            for _ in range(args.count):
                daemon.publish_once()
                time.sleep(1.0 / args.rate)
        else:
            daemon.run()
    except KeyboardInterrupt:
        daemon.stop()
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
