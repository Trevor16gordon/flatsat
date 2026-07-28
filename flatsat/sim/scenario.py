"""Scenario runner: execute a mission profile against the composed system.

A mission is DATA (``config/missions/*.txtpb``, schema
``flatsat/sim/mission.proto``): initial conditions, a
sequence of phases — each optionally commanding a mode transition — and
per-phase success criteria. This runner composes the REAL flight chain
in one process — sensor daemons, actuator daemons, the control loop, and
the mode manager, all resolved from the vehicle file through the
registry exactly as in flight — against the local rigid-body plant, then
judges each phase.

What this is: the system-level test tier. Every component below it has
its own contract tests; a mission proves the composition — config in,
behavior out, nothing wired by hand.

What this is not: physics fidelity. Basilisk remains the ground
machine's universe-fake; the same mission files are the script for real
HIL runs, where the Mac's bridge replaces the local plant behind the
same topics.

Missions run at wall clock (faster-than-real-time is a deliberate future
extension), so CI-able missions use scenario vehicles tuned to converge
in seconds.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import zenoh

from flatsat.apps.actuator_daemon import ActuatorDaemon
from flatsat.apps.control_loop import ControlLoop
from flatsat.apps.sensor_daemon import SensorDaemon
from flatsat.core.config import VehicleSpec, load_textproto, load_vehicle, which_impl
from flatsat.core.registry import (
    get_actuator_class,
    get_controller_class,
    get_driver_class,
    get_estimator_class,
    get_guidance_class,
)
from flatsat.mode import mode_config_pb2
from flatsat.mode.client import ModeClient
from flatsat.mode.manager import ModeManager
from flatsat.msgs import mode_pb2
from flatsat.sim import mission_pb2
from flatsat.sim.basilisk_hil import BasiliskPlant
from flatsat.sim.plant import LocalPlant, Plant

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_MODE_BASE = "test/scn/sys/mode"


@dataclass(frozen=True)
class SuccessCriteria:
    """What must be true at the end of a phase.

    Attributes:
        max_omega_mag_rad_s: Plant |omega| bound; None skips the check.
        require_mode: Bare mode name (``NOMINAL``, ``SAFE``, ...) the
            system must be latched in; None skips the check.
        require_all_acks: When True, every registered app must have
            acked the current mode sequence.
    """

    max_omega_mag_rad_s: float | None = None
    require_mode: str | None = None
    require_all_acks: bool = False


@dataclass(frozen=True)
class PhaseSpec:
    """One phase of a mission timeline.

    Attributes:
        name: Phase name, for the report.
        duration_s: How long the phase runs before judgment.
        request_mode: Bare mode name to request at phase start; None
            requests nothing.
        request_ground_authority: Whether the request carries ground
            authority (the runner playing ground vs playing FDIR).
        expect_request_refused: When True, the phase EXPECTS its mode
            request to be refused — asserting the authority asymmetry.
        success: End-of-phase criteria.
    """

    name: str
    duration_s: float
    request_mode: str | None = None
    request_ground_authority: bool = False
    expect_request_refused: bool = False
    success: SuccessCriteria = field(default_factory=SuccessCriteria)


@dataclass(frozen=True)
class MissionSpec:
    """A mission profile: what to fly and what must come true.

    Attributes:
        name: Mission identifier.
        description: Human-readable purpose.
        vehicle_path: Vehicle composition the mission flies.
        omega0_rad_s: Initial plant body rates.
        plant_rate_hz: Plant integration/publish rate.
        phases: The timeline.
    """

    name: str
    description: str
    vehicle_path: str
    omega0_rad_s: tuple[float, float, float]
    plant_rate_hz: float
    phases: tuple[PhaseSpec, ...]


@dataclass(frozen=True)
class PhaseResult:
    """One phase's judgment.

    Attributes:
        name: Phase name.
        passed: Whether every criterion held.
        details: Human-readable pass/fail lines, one per check.
    """

    name: str
    passed: bool
    details: tuple[str, ...]


@dataclass(frozen=True)
class ScenarioResult:
    """A full mission's judgment.

    Attributes:
        mission: Mission name.
        phases: Per-phase results, in order.
    """

    mission: str
    phases: tuple[PhaseResult, ...]

    @property
    def passed(self) -> bool:
        """Whether every phase passed."""
        return all(phase.passed for phase in self.phases)

    def describe(self) -> str:
        """Render the judgment for logs and assertion messages.

        Returns:
            A multi-line report.
        """
        lines = [f"mission {self.mission}: {'PASS' if self.passed else 'FAIL'}"]
        for phase in self.phases:
            lines.append(f"  phase {phase.name}: {'pass' if phase.passed else 'FAIL'}")
            lines.extend(f"    {detail}" for detail in phase.details)
        return "\n".join(lines)


def _mode_value(name: str) -> mode_pb2.SystemMode.ValueType:
    """Resolve a bare mode name from a mission file.

    Args:
        name: Bare name, e.g. ``SAFE``.

    Returns:
        The SystemMode value.

    Raises:
        ValueError: If the name is not a mode (fail loud at load).
    """
    try:
        value: mode_pb2.SystemMode.ValueType = mode_pb2.SystemMode.Value(f"SYSTEM_MODE_{name}")
    except ValueError as exc:
        raise ValueError(f"unknown mode {name!r} in mission file") from exc
    return value


def load_mission(path: Path | str) -> MissionSpec:
    """Load and validate a mission profile.

    Args:
        path: Mission file, e.g. ``config/missions/detumble_test.txtpb``.

    Returns:
        The parsed mission.

    Raises:
        ValueError: On a mode name that does not exist or bad dimensions.
        text_format.ParseError: On any unknown field (fail loud).
    """
    cfg = mission_pb2.MissionConfig()
    load_textproto(path, cfg)
    phases: list[PhaseSpec] = []
    for row in cfg.phases:
        if row.request_mode:
            _mode_value(row.request_mode)  # validate at load time
        criteria = SuccessCriteria(
            max_omega_mag_rad_s=(
                row.success.max_omega_mag_rad_s
                if row.success.HasField("max_omega_mag_rad_s")
                else None
            ),
            require_mode=row.success.require_mode or None,
            require_all_acks=row.success.require_all_acks,
        )
        if criteria.require_mode is not None:
            _mode_value(criteria.require_mode)
        phases.append(
            PhaseSpec(
                name=row.name,
                duration_s=row.duration_s,
                request_mode=row.request_mode or None,
                request_ground_authority=row.request_ground_authority,
                expect_request_refused=row.expect_request_refused,
                success=criteria,
            )
        )
    omega0 = tuple(cfg.omega0_rad_s)
    if len(omega0) != 3:
        raise ValueError(f"{path}: omega0_rad_s must have 3 values")
    return MissionSpec(
        name=cfg.name,
        description=cfg.description,
        vehicle_path=cfg.vehicle,
        omega0_rad_s=(omega0[0], omega0[1], omega0[2]),
        plant_rate_hz=cfg.plant_rate_hz if cfg.HasField("plant_rate_hz") else 100.0,
        phases=tuple(phases),
    )


def _truth_topic(vehicle: VehicleSpec) -> str:
    """Find the truth topic the vehicle's sim-fed IMU subscribes to.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        The truth key expression.

    Raises:
        KeyError: If no sensor declares a ``truth_topic`` — the mission
            needs a sim-fed vehicle.
    """
    for sensor in vehicle.sensors:
        if sensor.WhichOneof("options") == "basilisk_imu":
            return sensor.basilisk_imu.truth_topic or "sim/truth/state"
    raise KeyError(f"vehicle {vehicle.name!r} has no sim-fed sensor (basilisk_imu)")


class ScenarioRunner:
    """Composes the flight chain in-process and executes one mission."""

    def __init__(
        self,
        mission: MissionSpec,
        work_dir: Path,
        plant_kind: str = "local",
        viz_live: bool = False,
        viz_save: str | None = None,
    ) -> None:
        """Bind a mission to a scratch directory and a universe-fake.

        Args:
            mission: The mission to fly.
            work_dir: Scratch directory (clean-shutdown marker lives
                here — scenario runs must never touch flight state).
            plant_kind: ``local`` (rigid body, runs anywhere) or
                ``basilisk`` (full dynamics; needs Basilisk installed —
                the ground machine).
            viz_live: Basilisk only — live Vizard 3D view (start Vizard
                first in Direct Communication mode).
            viz_save: Basilisk only — record a Vizard playback ``.bin``.

        Raises:
            ValueError: On an unknown plant kind.
        """
        if plant_kind not in ("local", "basilisk"):
            raise ValueError(f"unknown plant {plant_kind!r}; use 'local' or 'basilisk'")
        self.mission = mission
        self._work_dir = work_dir
        self._plant_kind = plant_kind
        self._viz_live = viz_live
        self._viz_save = viz_save

    def _build_plant(self, vehicle: VehicleSpec, session: zenoh.Session) -> Plant:
        """Stand up the chosen universe-fake behind the sim topics.

        Args:
            vehicle: Loaded vehicle composition.
            session: Open zenoh session.

        Returns:
            The plant, not yet started.
        """
        if self._plant_kind == "basilisk":
            return BasiliskPlant(
                vehicle,
                session,
                truth_topic=_truth_topic(vehicle),
                omega0=self.mission.omega0_rad_s,
                rate_hz=self.mission.plant_rate_hz,
                viz_live=self._viz_live,
                viz_save=self._viz_save,
            )
        return LocalPlant(
            vehicle,
            session,
            truth_topic=_truth_topic(vehicle),
            omega0=self.mission.omega0_rad_s,
            rate_hz=self.mission.plant_rate_hz,
        )

    def run(self) -> ScenarioResult:
        """Fly the mission.

        Returns:
            The per-phase judgment.
        """
        vehicle = load_vehicle(REPO_ROOT / self.mission.vehicle_path)
        marker = self._work_dir / "clean-shutdown"
        marker.touch()  # scenario boots are clean boots: system goes NOMINAL
        mode_entry = mode_config_pb2.ModeConfig()
        mode_entry.CopyFrom(vehicle.mode)
        mode_entry.clean_shutdown_marker = str(marker)

        session = zenoh.open(zenoh.Config())
        threads: list[threading.Thread] = []
        sensor_daemons: list[SensorDaemon] = []
        actuator_daemons: list[ActuatorDaemon] = []
        clients: list[ModeClient] = []
        plant: Plant | None = None
        loop: ControlLoop | None = None
        manager: ModeManager | None = None
        try:
            manager = ModeManager(mode_entry, session, base_topic=SCENARIO_MODE_BASE)

            for entry in vehicle.sensors:
                driver_name = which_impl(entry, "options", entry.name)
                driver = get_driver_class(driver_name).from_config(
                    entry.name, getattr(entry, driver_name)
                )
                sensor_daemons.append(SensorDaemon(entry, driver, session))
            for actuator in vehicle.actuators:
                act_name = which_impl(actuator, "options", actuator.name)
                actuator_driver = get_actuator_class(act_name).from_config(
                    actuator.name, getattr(actuator, act_name)
                )
                actuator_daemons.append(ActuatorDaemon(actuator, actuator_driver, session))

            control = vehicle.control
            strategy = which_impl(control, "strategy", "control")
            objective = which_impl(control, "objective", "control")
            estimator = which_impl(control, "estimator", "control")
            loop = ControlLoop(
                session,
                control,
                get_controller_class(strategy).from_config(getattr(control, strategy)),
                get_guidance_class(objective).from_config(getattr(control, objective)),
                get_estimator_class(estimator).from_config(getattr(control, estimator)),
                vehicle_name=vehicle.name,
                config_checksum=vehicle.provenance.checksum,
            )

            clients = [
                ModeClient(session, app, base_topic=SCENARIO_MODE_BASE) for app in mode_entry.apps
            ]

            plant = self._build_plant(vehicle, session)
            plant.start()

            for daemon in sensor_daemons:
                thread = threading.Thread(
                    target=lambda d=daemon: d.run(health_every_s=0.0), daemon=True
                )
                thread.start()
                threads.append(thread)
            for act_daemon in actuator_daemons:
                thread = threading.Thread(
                    target=lambda d=act_daemon: d.run(health_every_s=0.0), daemon=True
                )
                thread.start()
                threads.append(thread)

            running_loop = loop  # narrow for the closure

            def run_loop() -> None:
                """Feed the control loop once input flows."""
                if running_loop.wait_for_first_sample(timeout_s=10.0):
                    running_loop.run(duration_s=0.0)

            loop_thread = threading.Thread(target=run_loop, daemon=True)
            loop_thread.start()
            threads.append(loop_thread)

            phases = tuple(self._run_phase(phase, manager, plant) for phase in self.mission.phases)
            return ScenarioResult(mission=self.mission.name, phases=phases)
        finally:
            if plant is not None:
                plant.stop()
            if loop is not None:
                loop.stop()
            for sensor_daemon in sensor_daemons:
                sensor_daemon.stop()
            for act_daemon in actuator_daemons:
                act_daemon.stop()
            for thread in threads:
                thread.join(timeout=3.0)
            if loop is not None:
                loop.close()
            for sensor_daemon in sensor_daemons:
                sensor_daemon.close()
            for act_daemon in actuator_daemons:
                act_daemon.close()
            for client in clients:
                client.close()
            if manager is not None:
                manager.close()
            session.close()

    def _run_phase(self, phase: PhaseSpec, manager: ModeManager, plant: Plant) -> PhaseResult:
        """Execute and judge one phase.

        Args:
            phase: The phase to run.
            manager: The live mode manager.
            plant: The live plant (truth source for criteria).

        Returns:
            The phase's judgment.
        """
        details: list[str] = []
        passed = True

        if phase.request_mode is not None:
            source = "ground" if phase.request_ground_authority else "fdir"
            decision = manager.request(
                mode_pb2.ModeRequest(
                    source=source,
                    requested=_mode_value(phase.request_mode),
                    reason=f"mission phase {phase.name}",
                    ground_authority=phase.request_ground_authority,
                )
            )
            if decision.accepted == phase.expect_request_refused:
                passed = False
                expectation = "refused" if phase.expect_request_refused else "accepted"
                details.append(
                    f"FAIL: request {phase.request_mode} should have been {expectation} "
                    f"({decision.reason})"
                )
            else:
                details.append(f"request {phase.request_mode}: {decision.reason}")

        time.sleep(phase.duration_s)

        criteria = phase.success
        if criteria.max_omega_mag_rad_s is not None:
            omega = plant.rate_magnitude()
            ok = omega <= criteria.max_omega_mag_rad_s
            passed = passed and ok
            details.append(
                f"{'ok' if ok else 'FAIL'}: |omega| {omega:.4f} rad/s "
                f"(bound {criteria.max_omega_mag_rad_s:g})"
            )
        if criteria.require_mode is not None:
            state = manager.state()
            actual = mode_pb2.SystemMode.Name(state.mode).removeprefix("SYSTEM_MODE_")
            ok = actual == criteria.require_mode
            passed = passed and ok
            details.append(
                f"{'ok' if ok else 'FAIL'}: mode {actual} (required {criteria.require_mode})"
            )
        if criteria.require_all_acks:
            missing = manager.missing_acks()
            ok = missing == []
            passed = passed and ok
            details.append(f"{'ok' if ok else 'FAIL'}: missing acks {missing}")

        return PhaseResult(name=phase.name, passed=passed, details=tuple(details))
