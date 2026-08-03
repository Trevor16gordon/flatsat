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


def test_build_plant_gives_basilisk_the_same_universe(tmp_path: Path) -> None:
    """The mission's orbit, attitude and epoch reach BOTH plants.

    This is the wiring the 2026-08-03 parity gap was made of: the
    runner passed sigma0 and the orbit to LocalPlant only, so the same
    mission file meant different physics under ``--plant basilisk``.
    Construction alone proves the pass-through — no Basilisk import
    happens before start().
    """
    import zenoh

    from flatsat.core.config import load_vehicle
    from flatsat.sim.scenario import REPO_ROOT

    mission = load_mission(MISSIONS / "deploy_detumble_sso.txtpb")
    assert mission.orbit_elements is not None, "mission under test must carry an orbit"
    runner = ScenarioRunner(mission, tmp_path, plant_kind="basilisk")
    vehicle = load_vehicle(REPO_ROOT / mission.vehicle_path)
    session = zenoh.open(zenoh.Config())
    try:
        plant = runner._build_plant(vehicle, session)
        assert plant._orbit is mission.orbit_elements  # type: ignore[attr-defined]
        assert plant._sigma0 == mission.sigma0  # type: ignore[attr-defined]
        assert plant._epoch_gmst_rad == mission.epoch_gmst_rad  # type: ignore[attr-defined]
        assert (
            plant._epoch_solar_angle_rad  # type: ignore[attr-defined]
            == mission.epoch_solar_angle_rad
        )
    finally:
        session.close()


@pytest.mark.verifies("FSW-ADCS-011", "FSW-SYS-001")
def test_bdot_detumble_mission_succeeds(tmp_path: Path) -> None:
    """Magnetic detumble end to end: magnetometer -> B-dot -> rods -> m x B.

    No wheels anywhere in this vehicle: if the rate comes down, the
    dipole-command chain and the plant's m x B coupling both work. The
    mission's bound is set above the along-B floor on purpose — see the
    mission file.
    """
    mission = load_mission(MISSIONS / "bdot_detumble_sso.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


@pytest.mark.verifies("FSW-ADCS-012", "FSW-SYS-001")
def test_momentum_dump_mission_succeeds(tmp_path: Path) -> None:
    """The combined system: wheels catch the tumble, rods drain the wheels.

    Phase 2's wheel-momentum criterion is read from the wheels' own
    state topics — the same device truth the flight side publishes — so
    a pass means the dipole chain moved real stored momentum out of the
    wheels while the body stayed at rest.
    """
    mission = load_mission(MISSIONS / "dump_detumble_sso.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()


@pytest.mark.verifies("FSW-FDIR-002", "FSW-SYS-002")
def test_fault_blackout_mission_succeeds(tmp_path: Path) -> None:
    """Truth dies mid-mission; the STALE cascade must end in an FDIR safing."""
    mission = load_mission(MISSIONS / "fault_blackout_test.txtpb")
    result = ScenarioRunner(mission, tmp_path).run()
    assert result.passed, result.describe()
