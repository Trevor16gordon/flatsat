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

import hashlib
import math
import threading
import time
from dataclasses import dataclass, field
from functools import partial
from pathlib import Path

import zenoh

from flatsat.apps.actuator_daemon import ActuatorDaemon
from flatsat.apps.control_loop import ControlLoop, wheel_state_inputs
from flatsat.apps.sensor_daemon import SensorDaemon
from flatsat.control.health.arbiter import Fdir
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
from flatsat.msgs import hal_pb2, mission_log_pb2, mode_pb2
from flatsat.sim import mission_pb2, orbit
from flatsat.sim.basilisk_hil import BasiliskPlant
from flatsat.sim.plant import LocalPlant, Plant
from flatsat.telemetry import mission_log, telemetry_config_pb2
from flatsat.telemetry.recorder import Recorder

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
        max_wheel_momentum_n_m_s: Bound on the body-frame wheel momentum
            magnitude, read from the wheels' own state topics — what a
            momentum-dump phase must drive down; None skips the check.
    """

    max_omega_mag_rad_s: float | None = None
    require_mode: str | None = None
    require_all_acks: bool = False
    max_wheel_momentum_n_m_s: float | None = None


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
    inject_truth_blackout: bool = False
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
        sigma0: Attitude at epoch, as an MRP. A deployer releases with an
            arbitrary orientation, not an identity one.
        orbit_path: Saved orbit file, or empty for an attitude-only run.
        orbit_elements: The resolved orbit; None when there is no orbit.
        epoch_gmst_rad: Greenwich sidereal angle at epoch.
        epoch_solar_angle_rad: Solar longitude at epoch.
    """

    name: str
    description: str
    vehicle_path: str
    omega0_rad_s: tuple[float, float, float]
    plant_rate_hz: float
    phases: tuple[PhaseSpec, ...]
    sigma0: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orbit_path: str = ""
    orbit_elements: orbit.OrbitalElements | None = None
    epoch_gmst_rad: float = 0.0
    epoch_solar_angle_rad: float = 0.0


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


def _file_digest(path: Path) -> str:
    """Hash a config file as flown.

    Args:
        path: File to digest.

    Returns:
        Hex sha256, or empty when the file is unreadable.
    """
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return ""


def _resolve_phase(declared: mission_pb2.PhaseConfig) -> mission_pb2.PhaseConfig:
    """Expand a phase that reuses a saved sub-mission.

    A phase naming ``use`` takes that file as its base and overrides
    whatever it sets itself. The override is proto3 MergeFrom, so an
    unset scalar leaves the base alone and a set one wins — which is
    what lets a mission reuse "keep nominal" but shorten it.

    Args:
        declared: The phase as written in the mission file.

    Returns:
        The phase with its base merged in; the same message when it
        names no base.

    Raises:
        FileNotFoundError: If the referenced sub-mission does not exist.
    """
    if not declared.use:
        return declared
    base_path = REPO_ROOT / declared.use
    if not base_path.is_file():
        raise FileNotFoundError(f"sub-mission {declared.use!r} not found at {base_path}")
    resolved = mission_pb2.PhaseConfig()
    load_textproto(base_path, resolved)
    overrides = mission_pb2.PhaseConfig()
    overrides.CopyFrom(declared)
    overrides.ClearField("use")
    resolved.MergeFrom(overrides)
    resolved.ClearField("use")
    return resolved


def load_orbit(path: Path | str) -> tuple[orbit.OrbitalElements, float, float]:
    """Load a saved orbit, resolving intent into numbers.

    An unset inclination means "sun-synchronous at this altitude", so it
    is computed rather than copied. That keeps the property the mission
    actually wants — fixed local solar time — true by construction if
    the altitude ever changes.

    Args:
        path: Orbit file, e.g. ``config/orbits/spacex_rideshare_sso.txtpb``.

    Returns:
        Tuple of (elements, epoch GMST radians, epoch solar angle radians).
    """
    cfg = mission_pb2.OrbitConfig()
    load_textproto(path, cfg)
    inclination = (
        math.radians(cfg.inclination_deg)
        if cfg.HasField("inclination_deg")
        else orbit.sun_synchronous_inclination_rad(cfg.altitude_m, cfg.eccentricity)
    )
    elements = orbit.OrbitalElements(
        semi_major_axis_m=orbit.R_EARTH_M + cfg.altitude_m,
        eccentricity=cfg.eccentricity,
        inclination_rad=inclination,
        raan_rad=math.radians(cfg.raan_deg),
        arg_periapsis_rad=math.radians(cfg.arg_periapsis_deg),
        true_anomaly_rad=math.radians(cfg.true_anomaly_deg),
    )
    return elements, math.radians(cfg.epoch_gmst_deg), math.radians(cfg.epoch_solar_angle_deg)


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
    for declared in cfg.phases:
        row = _resolve_phase(declared)
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
            max_wheel_momentum_n_m_s=(
                row.success.max_wheel_momentum_n_m_s
                if row.success.HasField("max_wheel_momentum_n_m_s")
                else None
            ),
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
                inject_truth_blackout=row.inject_truth_blackout,
                success=criteria,
            )
        )
    omega0 = tuple(cfg.omega0_rad_s)
    if len(omega0) != 3:
        raise ValueError(f"{path}: omega0_rad_s must have 3 values")
    sigma0 = tuple(cfg.sigma0) or (0.0, 0.0, 0.0)
    if len(sigma0) != 3:
        raise ValueError(f"{path}: sigma0 must have 3 values")

    elements = None
    gmst = 0.0
    solar = 0.0
    if cfg.orbit:
        elements, gmst, solar = load_orbit(REPO_ROOT / cfg.orbit)
    return MissionSpec(
        name=cfg.name,
        description=cfg.description,
        vehicle_path=cfg.vehicle,
        omega0_rad_s=(omega0[0], omega0[1], omega0[2]),
        plant_rate_hz=cfg.plant_rate_hz if cfg.HasField("plant_rate_hz") else 100.0,
        phases=tuple(phases),
        sigma0=(sigma0[0], sigma0[1], sigma0[2]),
        orbit_path=cfg.orbit,
        orbit_elements=elements,
        epoch_gmst_rad=gmst,
        epoch_solar_angle_rad=solar,
    )


class WheelMomentumMonitor:
    """Latest stored momentum per wheel, summed into the body frame.

    The judge's view of the wheels: the same state topics the flight
    side publishes, mapped through the same mounting axes — the runner
    never asks the drivers directly, because a mission criterion must be
    checkable from recorded telemetry alone.
    """

    def __init__(
        self, session: zenoh.Session, wheels: list[tuple[str, tuple[float, float, float]]]
    ) -> None:
        """Subscribe to every wheel's state topic.

        Args:
            session: Open zenoh session.
            wheels: (state_topic, body-frame spin axis) per wheel.
        """
        self._axes = dict(wheels)
        self._momentum: dict[str, float] = {}
        self._lock = threading.Lock()
        self._subs = [
            session.declare_subscriber(topic, partial(self._on_state, topic)) for topic, _ in wheels
        ]

    def _on_state(self, topic: str, sample: zenoh.Sample) -> None:
        """Store one wheel's latest momentum (subscriber thread).

        Args:
            topic: The state topic the sample arrived on.
            sample: Incoming WheelState.
        """
        msg = hal_pb2.WheelState.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._momentum[topic] = msg.momentum_n_m_s

    def body_momentum_magnitude(self) -> float:
        """|sum over wheels of axis * momentum|.

        Returns:
            The body-frame wheel momentum magnitude, N·m·s; wheels that
            have not reported contribute zero.
        """
        total = [0.0, 0.0, 0.0]
        with self._lock:
            for topic, axis in self._axes.items():
                momentum = self._momentum.get(topic, 0.0)
                for index in range(3):
                    total[index] += axis[index] * momentum
        return math.sqrt(sum(value * value for value in total))

    def close(self) -> None:
        """Undeclare the subscriptions."""
        for sub in self._subs:
            sub.undeclare()


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
        run_id: str | None = None,
        record_dir: Path | None = None,
        source_kind: mission_log_pb2.SourceKind.ValueType | None = None,
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
            run_id: Identity for this execution, carried on every span so
                the archive can be sliced by run. None generates one.
            record_dir: Archive the run beneath this directory, one
                subdirectory per run. None records nothing.
            source_kind: What produced the data. None infers it from the
                plant, which is a guess the caller can override — the
                plant alone cannot tell a synthetic device from a real
                one behind the same driver contract.

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
        self.run_id = run_id or mission_log.default_run_id(mission.name or "mission")
        self._record_dir = record_dir
        self._source_kind = source_kind or (
            mission_log_pb2.SOURCE_KIND_HIL
            if plant_kind == "basilisk"
            else mission_log_pb2.SOURCE_KIND_SIM
        )
        self.archive_dir: Path | None = None

    def _start_recorder(self, session: zenoh.Session, vehicle: VehicleSpec) -> Recorder | None:
        """Archive this run, if the caller asked for it.

        The recorder subscribes to the same bus the mission runs on, so
        it captures the mission's spans alongside its telemetry and the
        archive needs no separate assembly step.

        Args:
            session: The mission's open bus session.
            vehicle: Loaded vehicle, for its topic list and bounds.

        Returns:
            The running recorder, or None when not recording.
        """
        if self._record_dir is None:
            return None
        self.archive_dir = self._record_dir / self.run_id
        self.archive_dir.mkdir(parents=True, exist_ok=True)
        entry = telemetry_config_pb2.TelemetryConfig()
        entry.CopyFrom(vehicle.telemetry)
        entry.output_dir = str(self.archive_dir)
        # Capture EVERYTHING for a scenario run. The flight recorder's
        # allowlist is deliberately narrow because it runs forever on a
        # bounded disk; a mission run is bounded by definition, and a
        # blob that silently omitted whatever the vehicle happened to
        # publish under a scenario prefix would be worse than large.
        del entry.topics[:]
        entry.topics.append("**")
        header = mission_log.build_session_header(
            run_id=self.run_id,
            source_kind=self._source_kind,
            mission_name=self.mission.name,
            vehicle_path=self.mission.vehicle_path,
            vehicle_sha256=_file_digest(REPO_ROOT / self.mission.vehicle_path),
            plant=self._plant_kind,
        )
        return Recorder(entry, session, session_header=header)

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
                sigma0=self.mission.sigma0,
                orbit_elements=self.mission.orbit_elements,
                epoch_gmst_rad=self.mission.epoch_gmst_rad,
                epoch_solar_angle_rad=self.mission.epoch_solar_angle_rad,
                viz_live=self._viz_live,
                viz_save=self._viz_save,
            )
        return LocalPlant(
            vehicle,
            session,
            truth_topic=_truth_topic(vehicle),
            omega0=self.mission.omega0_rad_s,
            rate_hz=self.mission.plant_rate_hz,
            sigma0=self.mission.sigma0,
            orbit_elements=self.mission.orbit_elements,
            epoch_gmst_rad=self.mission.epoch_gmst_rad,
            epoch_solar_angle_rad=self.mission.epoch_solar_angle_rad,
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
        recorder = self._start_recorder(session, vehicle)
        threads: list[threading.Thread] = []
        sensor_daemons: list[SensorDaemon] = []
        actuator_daemons: list[ActuatorDaemon] = []
        clients: list[ModeClient] = []
        plant: Plant | None = None
        loop: ControlLoop | None = None
        manager: ModeManager | None = None
        fdir: Fdir | None = None
        momentum_monitor: WheelMomentumMonitor | None = None
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
                wheels=wheel_state_inputs(vehicle),
            )
            momentum_monitor = WheelMomentumMonitor(session, wheel_state_inputs(vehicle))

            clients = [
                ModeClient(session, app, base_topic=SCENARIO_MODE_BASE) for app in mode_entry.apps
            ]

            if len(vehicle.fdir.rules) > 0:
                fdir = Fdir(vehicle.fdir, session, base_topic=SCENARIO_MODE_BASE)
                fdir_thread = threading.Thread(
                    target=lambda: fdir.run(health_every_s=1.0), daemon=True
                )
                fdir_thread.start()
                threads.append(fdir_thread)

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

            # Mission structure goes onto the BUS, so the recorder
            # captures it alongside the telemetry and the archive can be
            # sliced by sub-mission without a sidecar.
            logger = mission_log.MissionLogger(session, self.run_id)
            results: list[PhaseResult] = []
            with logger.span(
                self.mission.name,
                mission_log_pb2.SPAN_KIND_MISSION,
                {"vehicle": self.mission.vehicle_path, "plant": self._plant_kind},
            ):
                for phase in self.mission.phases:
                    span_id = logger.start(phase.name, mission_log_pb2.SPAN_KIND_PHASE)
                    result = self._run_phase(phase, manager, plant, momentum_monitor)
                    for detail in result.details:
                        logger.annotate(detail, "info" if result.passed else "error", "runner")
                    logger.end(
                        span_id,
                        outcome="pass" if result.passed else "fail",
                        detail="; ".join(result.details),
                    )
                    results.append(result)
            return ScenarioResult(mission=self.mission.name, phases=tuple(results))
        finally:
            if recorder is not None:
                recorder.close()
            if momentum_monitor is not None:
                momentum_monitor.close()
            if fdir is not None:
                fdir.stop()
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
            if fdir is not None:
                fdir.close()
            if manager is not None:
                manager.close()
            session.close()

    def _run_phase(
        self,
        phase: PhaseSpec,
        manager: ModeManager,
        plant: Plant,
        momentum_monitor: WheelMomentumMonitor | None = None,
    ) -> PhaseResult:
        """Execute and judge one phase.

        Args:
            phase: The phase to run.
            manager: The live mode manager.
            plant: The live plant (truth source for criteria).
            momentum_monitor: The wheels' momentum as published on their
                state topics, for the wheel-momentum criterion.

        Returns:
            The phase's judgment.
        """
        details: list[str] = []
        passed = True

        if phase.inject_truth_blackout:
            plant.stop()  # truth goes dark: the bench-without-bridge signature
            details.append("injected: truth blackout (plant stopped)")

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
        if criteria.max_wheel_momentum_n_m_s is not None:
            momentum = (
                momentum_monitor.body_momentum_magnitude() if momentum_monitor is not None else 0.0
            )
            ok = momentum <= criteria.max_wheel_momentum_n_m_s
            passed = passed and ok
            details.append(
                f"{'ok' if ok else 'FAIL'}: |wheel momentum| {momentum:.5f} N·m·s "
                f"(bound {criteria.max_wheel_momentum_n_m_s:g})"
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
