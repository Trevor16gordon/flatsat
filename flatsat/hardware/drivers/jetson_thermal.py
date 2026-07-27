"""Jetson kernel thermal-zone driver (a real device).

Reads ``/sys/class/thermal/thermal_zone*/temp`` selected by type name.

Acquisition note that cost a debugging session: power-gated zones (the
``cv*`` cluster when the CV accelerators are off) return EAGAIN, and
Python's IO layers hide that — text mode raises TypeError from the codecs
module, buffered binary mode returns None. Both evade ``except OSError``
and killed the daemon. Raw ``os.read`` surfaces it honestly as
BlockingIOError, which becomes a COMM flag.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path

from flatsat.core.bus import HalMessage
from flatsat.hardware.sensor import SensorDriver
from flatsat.msgs import hal_pb2

THERMAL_ROOT = Path("/sys/class/thermal")


def find_zone_temp_path(zone_type: str, root: Path = THERMAL_ROOT) -> Path:
    """Resolve a thermal zone type name to its ``temp`` file.

    Args:
        zone_type: Kernel zone type, e.g. ``tj-thermal``.
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


class JetsonThermalDriver(SensorDriver):
    """Publishes one Jetson thermal zone as TemperatureSample."""

    def __init__(self, zone_type: str) -> None:
        """Resolve the zone once at startup.

        Args:
            zone_type: Kernel thermal zone type name to read.
        """
        self._zone_type = zone_type
        self._temp_path = find_zone_temp_path(zone_type)

    @classmethod
    def from_config(cls, name: str, options: Mapping[str, object]) -> JetsonThermalDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the zone identifies the device).
            options: Must contain ``zone``.

        Returns:
            The driver bound to that thermal zone.
        """
        return cls(zone_type=str(options["zone"]))

    def read(self) -> tuple[HalMessage, int]:
        """Read the zone once; EAGAIN/garbage becomes a COMM-flagged sample.

        Returns:
            Tuple of (TemperatureSample, validity flags).
        """
        msg = hal_pb2.TemperatureSample()
        msg.location = self._zone_type
        try:
            fd = os.open(self._temp_path, os.O_RDONLY)
            try:
                raw = os.read(fd, 32)
            finally:
                os.close(fd)
            millideg = int(raw.decode("ascii").strip())
        except (OSError, ValueError):
            return msg, int(hal_pb2.VALIDITY_FLAG_COMM)
        msg.temperature_c = millideg / 1000.0
        return msg, int(hal_pb2.VALIDITY_FLAG_VALID)

    def describe(self) -> list[str]:
        """Describe the bound zone.

        Returns:
            One line naming the sysfs path in use.
        """
        return [f"driver: jetson_thermal zone={self._zone_type} ({self._temp_path})"]
