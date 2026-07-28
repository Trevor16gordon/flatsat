"""System-level integration: mission profiles against the composed chain.

These are the whole-system tests: every component is resolved from the
scenario vehicle file through the registry — sensor daemon with the
sim-fed IMU, three actuator daemons projecting through their mountings,
the control loop, the mode manager — closed against the local
rigid-body plant. Nothing is wired by hand; a mission is a config file.
"""

from pathlib import Path

import pytest

from flatsat.sim.scenario import ScenarioRunner, load_mission

MISSIONS = Path("config/missions")


@pytest.mark.verifies("FSW-SYS-001", "FSW-SYS-002")
def test_detumble_mission_succeeds(tmp_path: Path) -> None:
    """A tumbling vehicle is brought to rest by the composed flight chain."""
    mission = load_mission(MISSIONS / "detumble_test.toml")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


@pytest.mark.verifies("FSW-SYS-002", "FSW-SYS-003")
def test_safe_entry_mission_succeeds(tmp_path: Path) -> None:
    """Safing mid-mission: automatic in, acked by all, ground-only out."""
    mission = load_mission(MISSIONS / "safe_entry_test.toml")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


def test_mission_loader_rejects_unknown_modes(tmp_path: Path) -> None:
    bad = tmp_path / "bad_mission.toml"
    bad.write_text(
        """
name = "bad"
vehicle = "config/vehicles/test_scenario.toml"
[initial]
omega0_rad_s = [0.1, 0.0, 0.0]
[[phases]]
name = "p1"
duration_s = 1.0
request_mode = "WARP"
"""
    )
    with pytest.raises(ValueError, match="unknown mode"):
        load_mission(bad)


def test_mission_files_parse() -> None:
    """Every checked-in mission profile must at least load cleanly."""
    missions = sorted(MISSIONS.glob("*.toml"))
    assert missions, "no mission profiles found"
    for path in missions:
        mission = load_mission(path)
        assert mission.phases, f"{path.name}: mission with no phases"
