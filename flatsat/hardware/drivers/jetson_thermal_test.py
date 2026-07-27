"""Jetson thermal driver: zone resolution and flag-and-forward at read().

The daemon-level cadence guarantee is proven in
``flatsat/apps/sensor_daemon_test.py``; these tests pin the driver's own
behavior, including zone resolution against a fake sysfs tree so they run
off-target too.
"""

from pathlib import Path

import pytest

from flatsat.hardware.drivers.jetson_thermal import (
    THERMAL_ROOT,
    JetsonThermalDriver,
    find_zone_temp_path,
)
from flatsat.msgs import hal_pb2

ON_TARGET = THERMAL_ROOT.exists()


def _fake_sysfs(root: Path, zones: dict[str, str]) -> None:
    """Build a thermal sysfs lookalike.

    Args:
        root: Directory standing in for /sys/class/thermal.
        zones: Mapping of zone type name to temp-file contents.
    """
    for index, (zone_type, temp) in enumerate(zones.items()):
        zone_dir = root / f"thermal_zone{index}"
        zone_dir.mkdir()
        (zone_dir / "type").write_text(zone_type + "\n")
        (zone_dir / "temp").write_text(temp)


def test_zone_resolution_by_type_name(tmp_path: Path) -> None:
    _fake_sysfs(tmp_path, {"cpu-thermal": "41000\n", "tj-thermal": "43500\n"})
    path = find_zone_temp_path("tj-thermal", root=tmp_path)
    assert path.read_text().strip() == "43500"


def test_missing_zone_fails_loud_at_startup(tmp_path: Path) -> None:
    _fake_sysfs(tmp_path, {"cpu-thermal": "41000\n"})
    with pytest.raises(FileNotFoundError, match="no thermal zone"):
        find_zone_temp_path("tj-thermal", root=tmp_path)


@pytest.mark.skipif(not ON_TARGET, reason="no thermal sysfs (not on target)")
def test_read_returns_valid_sane_temperature() -> None:
    driver = JetsonThermalDriver.from_config("tj", {"zone": "tj-thermal"})
    msg, flags = driver.read()
    assert flags == hal_pb2.VALIDITY_FLAG_VALID
    assert isinstance(msg, hal_pb2.TemperatureSample)
    assert 5.0 < msg.temperature_c < 110.0


@pytest.mark.skipif(not ON_TARGET, reason="no thermal sysfs (not on target)")
def test_unreadable_zone_flags_comm_without_a_value() -> None:
    """A power-gated zone EAGAINs; the read must flag, not raise or invent."""
    driver = JetsonThermalDriver.from_config("cv0", {"zone": "cv0-thermal"})
    msg, flags = driver.read()
    assert isinstance(msg, hal_pb2.TemperatureSample)
    if flags & hal_pb2.VALIDITY_FLAG_COMM:  # CV cluster is actually off
        assert msg.temperature_c == 0.0, "no value smuggled alongside a COMM flag"
    else:  # CV cluster happened to be powered — a real read is also a pass
        assert msg.temperature_c > 0.0
