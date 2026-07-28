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
    mission = load_mission(MISSIONS / "detumble_test.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


@pytest.mark.verifies("FSW-SYS-002", "FSW-SYS-003")
def test_safe_entry_mission_succeeds(tmp_path: Path) -> None:
    """Safing mid-mission: automatic in, acked by all, ground-only out."""
    mission = load_mission(MISSIONS / "safe_entry_test.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


def test_mission_loader_rejects_unknown_modes(tmp_path: Path) -> None:
    bad = tmp_path / "bad_mission.txtpb"
    bad.write_text(
        """
name: "bad"
vehicle: "config/vehicles/test_scenario.txtpb"
omega0_rad_s: [0.1, 0.0, 0.0]
phases { name: "p1" duration_s: 1.0 request_mode: "WARP" }
"""
    )
    with pytest.raises(ValueError, match="unknown mode"):
        load_mission(bad)


def test_mission_files_parse() -> None:
    """Every checked-in mission profile must at least load cleanly."""
    missions = sorted(MISSIONS.glob("*.txtpb"))
    assert missions, "no mission profiles found"
    for path in missions:
        mission = load_mission(path)
        assert mission.phases, f"{path.name}: mission with no phases"


def test_runner_rejects_unknown_plant(tmp_path: Path) -> None:
    mission = load_mission(MISSIONS / "detumble_test.txtpb")
    with pytest.raises(ValueError, match="unknown plant"):
        ScenarioRunner(mission, tmp_path, plant_kind="matrix")


def test_basilisk_plant_class_is_importable_without_basilisk() -> None:
    """Constructing the class must not import Basilisk (flight computer!).

    Only start() may touch the Basilisk packages — the scenario runner
    and this module must import cleanly on a machine that has none.
    """
    from flatsat.sim.basilisk_hil import BasiliskPlant

    assert callable(BasiliskPlant)


@pytest.mark.verifies("FSW-FDIR-002", "FSW-SYS-002")
def test_fault_blackout_mission_succeeds(tmp_path: Path) -> None:
    """Truth dies mid-mission; the STALE cascade must end in an FDIR safing."""
    mission = load_mission(MISSIONS / "fault_blackout_test.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()
