#!/usr/bin/env python3
"""Basilisk bridge: the universe-fake, slimmed to physics + truth topics.

Runs on the GROUND host (Mac — the sim computer is never the flight
computer). A rigid spacecraft integrates in Basilisk; every step the
bridge publishes rigid-body TRUTH on the truth topic and folds each
wheel's applied axis torque (from the flight-side
``basilisk_reaction_wheel`` drivers) into the plant.

The plant is built FROM THE VEHICLE FILE — body mass/inertia, one torque
input per declared actuator mapped through ITS mounting — so the sim's
spacecraft and the flight software's model of it cannot diverge by
accident. Deliberate divergence (a ``truth_overrides`` block: "actual
inertia 8% higher than believed") is the future robustness-campaign knob.

Device corruption lives in the flight-side drivers (shared
``hardware/models``), NOT here: flight-clock timestamps, sequence
numbers, staleness flags, and health telemetry all stay in the daemon
machinery during HIL. No stopping services, no topic squatting.

Time discipline: the sim is paced to WALL CLOCK with absolute deadlines
(the same drift-free pattern as the flight loops); lost time is dropped,
never replayed — the flight software never knows sim time exists.

:class:`BasiliskPlant` is interchangeable with
:class:`~flatsat.sim.plant.LocalPlant` behind the same topics — the
scenario runner picks one, so the SAME mission files drive quick local
runs and full-fidelity Basilisk runs (with Vizard 3D when asked).

Usage (from the repo root, ground venv active):
  python -m flatsat.sim.basilisk_hil
  python -m flatsat.sim.basilisk_hil --connect tcp/jetson.local:7447
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from collections.abc import Sequence

import numpy as np
import zenoh

from flatsat.core.bus import SamplePublisher
from flatsat.core.config import VehicleSpec, load_vehicle, which_impl
from flatsat.hardware.drivers.basilisk_magnetorquer import magnetorquer_dipole_topic
from flatsat.hardware.drivers.basilisk_reaction_wheel import wheel_torque_topic
from flatsat.msgs import sim_pb2
from flatsat.sim import orbit

TRUTH_TOPIC = "sim/truth/state"

Vec3 = tuple[float, float, float]


def mrp_to_dcm(sigma: Sequence[float]) -> list[list[float]]:
    """Rotation carrying an inertial vector into the body frame.

    Lives here, beside the other shared plant machinery, so BOTH
    universe-fakes rotate the environment with the same code — the
    local plant's rigid body delegates to it. Two implementations of
    the same rotation is exactly the kind of quiet divergence the
    plant-parity contract forbids.

    Args:
        sigma: Attitude as a modified Rodrigues parameter (sigma_BN).

    Returns:
        A 3x3 direction cosine matrix (body from inertial).
    """
    s0, s1, s2 = sigma
    square = s0 * s0 + s1 * s1 + s2 * s2
    skew = [[0.0, -s2, s1], [s2, 0.0, -s0], [-s1, s0, 0.0]]
    skew2 = [[sum(skew[i][k] * skew[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    denom = (1.0 + square) ** 2
    return [
        [
            (1.0 if i == j else 0.0)
            + (8.0 * skew2[i][j] - 4.0 * (1.0 - square) * skew[i][j]) / denom
            for j in range(3)
        ]
        for i in range(3)
    ]


def fill_environment(
    truth: sim_pb2.TruthState,
    position_eci_m: np.ndarray,
    dcm_body_from_inertial: Sequence[Sequence[float]],
    t_s: float,
    epoch_gmst_rad: float,
    epoch_solar_angle_rad: float,
) -> None:
    """Fill TruthState's environment fields from a position and attitude.

    Shared by both plants: the position may come from either universe's
    dynamics, but the field, sun and eclipse models are ALWAYS
    ``flatsat.sim.orbit`` — the plants differ in how faithfully they
    integrate motion, never in what universe they inhabit.

    The field is rotated into the BODY frame because that is where a
    magnetometer measures it and where ``m x B`` has to be evaluated;
    leaving the rotation to every consumer would mean every consumer
    needs the attitude.

    Args:
        truth: The message being filled.
        position_eci_m: Inertial position of the vehicle.
        dcm_body_from_inertial: Rotation carrying inertial into body.
        t_s: Seconds since epoch.
        epoch_gmst_rad: Greenwich sidereal angle at epoch.
        epoch_solar_angle_rad: Solar longitude at epoch.
    """
    field_eci = orbit.magnetic_field_eci(position_eci_m, t_s, epoch_gmst_rad)
    dcm = dcm_body_from_inertial
    field_body = [sum(dcm[i][j] * field_eci[j] for j in range(3)) for i in range(3)]
    truth.position_x_m, truth.position_y_m, truth.position_z_m = (float(v) for v in position_eci_m)
    truth.mag_field_x_t, truth.mag_field_y_t, truth.mag_field_z_t = field_body
    sun = orbit.sun_direction_eci(t_s, epoch_solar_angle_rad)
    truth.in_eclipse = orbit.in_eclipse(position_eci_m, sun)


def open_session(connect: str | None) -> zenoh.Session:
    """Open a zenoh session, optionally with an explicit peer endpoint.

    Args:
        connect: Endpoint like ``tcp/jetson.local:7447``, or None to rely on
            multicast discovery on the LAN.

    Returns:
        The open session.
    """
    config = zenoh.Config()
    if connect:
        config.insert_json5("connect/endpoints", f'["{connect}"]')
    return zenoh.open(config)


def plant_from_vehicle(
    vehicle: VehicleSpec,
) -> tuple[float, list[list[float]], list[tuple[str, Vec3]]]:
    """Derive the simulation plant from the vehicle file.

    This is the no-accidental-divergence property as a function: the same
    body and mounting entries the flight software reads become the
    physics engine's parameters.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        Tuple of (mass_kg, 3x3 inertia matrix as nested lists, list of
        (wheel name, body-frame spin axis) per declared reaction wheel —
        magnetorquers are not wheels; see :func:`rods_from_vehicle`).

    Raises:
        KeyError: If the vehicle declares no body physical model.
    """
    body = vehicle.require_body()
    flat = list(body.inertia_kg_m2)
    inertia = [flat[0:3], flat[3:6], flat[6:9]]
    wheels = [
        (entry.name, (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2]))
        for entry in vehicle.actuators
        if which_impl(entry, "options", entry.name).endswith("reaction_wheel")
    ]
    return body.mass_kg, inertia, wheels


def rods_from_vehicle(vehicle: VehicleSpec) -> list[tuple[str, Vec3]]:
    """The vehicle's magnetorquer rods, by the same mounting entries.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        List of (rod name, body-frame rod axis) per declared magnetorquer.
    """
    return [
        (entry.name, (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2]))
        for entry in vehicle.actuators
        if which_impl(entry, "options", entry.name).endswith("magnetorquer")
    ]


def magnetorquer_torque(rod_dipoles: Sequence[tuple[Vec3, float]], field_body_t: Vec3) -> Vec3:
    """Torque the rods produce in THIS field: ``(sum m_i) x B``.

    This is the whole point of specifying a magnetorquer by dipole: the
    torque is decided here, by the field where the vehicle happens to
    be — zero about the field line, always. Shared by both plants so
    neither can grant magnetic authority the other would not.

    Args:
        rod_dipoles: Per-rod (body-frame unit axis, applied dipole A·m²).
        field_body_t: Magnetic field in the body frame, tesla.

    Returns:
        Body-frame torque in N·m.
    """
    mx = sum(axis[0] * dipole for axis, dipole in rod_dipoles)
    my = sum(axis[1] * dipole for axis, dipole in rod_dipoles)
    mz = sum(axis[2] * dipole for axis, dipole in rod_dipoles)
    bx, by, bz = field_body_t
    return (my * bz - mz * by, mz * bx - mx * bz, mx * by - my * bx)


class WheelTorqueSink:
    """Latest applied axis torque from one wheel, with a stale cutoff.

    The flight-side actuator daemon already zeroes on stale commands; this
    cutoff is the bridge's own defense in depth — if the DAEMON dies, its
    last feedback message must not keep torquing the plant forever.
    """

    def __init__(self, session: zenoh.Session, wheel: str, timeout_s: float = 0.5) -> None:
        """Subscribe to one wheel's applied-torque topic.

        Args:
            session: Open zenoh session.
            wheel: Actuator instance name.
            timeout_s: Feedback age beyond which the contribution is zero.
        """
        self.wheel = wheel
        self._lock = threading.Lock()
        self._torque = 0.0
        self._recv_ns = 0
        self._timeout_ns = int(timeout_s * 1e9)
        self.messages_received = 0
        self._sub = session.declare_subscriber(wheel_torque_topic(wheel), self._on_torque)

    def _on_torque(self, sample: zenoh.Sample) -> None:
        """Store the newest applied torque.

        Args:
            sample: Incoming WheelAxisTorque.
        """
        msg = sim_pb2.WheelAxisTorque.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._torque = msg.torque_n_m
            self._recv_ns = time.monotonic_ns()
            self.messages_received += 1

    def latest(self) -> float:
        """Return the applied axis torque, or zero if feedback is stale.

        Returns:
            Torque in N·m about the wheel's axis; 0.0 when nothing fresh.
        """
        with self._lock:
            if self._recv_ns == 0 or time.monotonic_ns() - self._recv_ns > self._timeout_ns:
                return 0.0
            return self._torque


class DipoleSink:
    """Latest applied dipole from one magnetorquer rod, with a stale cutoff.

    The same defense-in-depth rule as :class:`WheelTorqueSink`: if the
    flight-side daemon dies, its last dipole must not keep torquing the
    plant forever.
    """

    def __init__(self, session: zenoh.Session, rod: str, timeout_s: float = 0.5) -> None:
        """Subscribe to one rod's applied-dipole topic.

        Args:
            session: Open zenoh session.
            rod: Actuator instance name.
            timeout_s: Feedback age beyond which the contribution is zero.
        """
        self.rod = rod
        self._lock = threading.Lock()
        self._dipole = 0.0
        self._recv_ns = 0
        self._timeout_ns = int(timeout_s * 1e9)
        self.messages_received = 0
        self._sub = session.declare_subscriber(magnetorquer_dipole_topic(rod), self._on_dipole)

    def _on_dipole(self, sample: zenoh.Sample) -> None:
        """Store the newest applied dipole.

        Args:
            sample: Incoming MagnetorquerDipole.
        """
        msg = sim_pb2.MagnetorquerDipole.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._dipole = msg.dipole_a_m2
            self._recv_ns = time.monotonic_ns()
            self.messages_received += 1

    def latest(self) -> float:
        """Return the applied axis dipole, or zero if feedback is stale.

        Returns:
            Dipole in A·m² along the rod's axis; 0.0 when nothing fresh.
        """
        with self._lock:
            if self._recv_ns == 0 or time.monotonic_ns() - self._recv_ns > self._timeout_ns:
                return 0.0
            return self._dipole


class BasiliskPlant:
    """Basilisk physics behind the sim topics: truth out, wheel torque in.

    Interchangeable with :class:`~flatsat.sim.plant.LocalPlant` (same
    constructor shape, same ``start``/``stop``/``rate_magnitude``), so a
    mission runs against real dynamics — optionally with a live Vizard
    3D view or a recorded Vizard playback file — by a plant swap.

    Basilisk imports happen inside :meth:`start`, so this module stays
    importable on the flight computer, where Basilisk never runs.
    """

    def __init__(
        self,
        vehicle: VehicleSpec,
        session: zenoh.Session,
        truth_topic: str,
        omega0: Vec3,
        rate_hz: float = 100.0,
        sigma0: Vec3 = (0.0, 0.0, 0.0),
        orbit_elements: orbit.OrbitalElements | None = None,
        epoch_gmst_rad: float = 0.0,
        epoch_solar_angle_rad: float = 0.0,
        viz_live: bool = False,
        viz_save: str | None = None,
        viz_address: str = "localhost",
        report_every_s: float = 5.0,
    ) -> None:
        """Build the plant from the vehicle file and wire its topics.

        Args:
            vehicle: Loaded vehicle composition (body + mounting).
            session: Open zenoh session; not owned — caller closes it.
            truth_topic: Key to publish TruthState on (the same key the
                vehicle's basilisk_imu entries subscribe to).
            omega0: Initial body rates.
            rate_hz: Sim step and publish rate.
            sigma0: Attitude at epoch, as an MRP.
            orbit_elements: Orbit to fly; None runs attitude-only in an
                empty universe, exactly like the local plant.
            epoch_gmst_rad: Greenwich sidereal angle at epoch, which
                fixes where the tilted dipole points to begin with.
            epoch_solar_angle_rad: Solar longitude at epoch.
            viz_live: Live-stream to Vizard (start Vizard FIRST in its
                'Direct Communication' mode; the sim connects out).
            viz_save: Record a Vizard playback ``.bin`` to this path
                instead of streaming (open it in Vizard afterwards).
            viz_address: Host Vizard is bound to. Basilisk's default
                0.0.0.0 is a valid BIND address but not a valid DIAL
                address on macOS/BSD — live streaming silently times out
                there unless this is localhost.
            report_every_s: Status print period; <= 0 silences it.
        """
        self.vehicle = vehicle
        self._session = session
        self._truth_topic = truth_topic
        self._omega0 = omega0
        self._sigma0 = sigma0
        self._orbit = orbit_elements
        self._epoch_gmst_rad = epoch_gmst_rad
        self._epoch_solar_angle_rad = epoch_solar_angle_rad
        self._rate_hz = rate_hz
        self._viz_live = viz_live
        self._viz_save = viz_save
        self._viz_address = viz_address
        self._report_every_s = report_every_s
        self.mass_kg, self.inertia, self.wheels = plant_from_vehicle(vehicle)
        self.rods = rods_from_vehicle(vehicle)
        self._axes = dict(self.wheels)
        self._rod_axes = dict(self.rods)
        self._sinks = [WheelTorqueSink(session, name) for name, _ in self.wheels]
        self._dipole_sinks = [DipoleSink(session, name) for name, _ in self.rods]
        self._field_body: Vec3 | None = None
        self._truth_pub = SamplePublisher(session, truth_topic, "sim_truth")
        self._lock = threading.Lock()
        self._omega: Vec3 = omega0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def rate_magnitude(self) -> float:
        """Current |omega| of the simulated body.

        Returns:
            The body-rate magnitude in rad/s.
        """
        with self._lock:
            return math.sqrt(sum(w * w for w in self._omega))

    def start(self) -> None:
        """Build the Basilisk simulation and start the pacing thread."""
        # Basilisk imports live here: this module must import on the
        # flight computer, where Basilisk is never installed.
        from Basilisk.simulation import extForceTorque, spacecraft
        from Basilisk.utilities import SimulationBaseClass, macros

        dt_s = 1.0 / self._rate_hz
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("dynProcess")
        proc.addTask(sim.CreateNewTask("dynTask", macros.sec2nano(dt_s)))

        sc = spacecraft.Spacecraft()
        sc.ModelTag = "flatsat-sim"
        sc.hub.mHub = self.mass_kg  # FROM THE VEHICLE FILE — never a literal here
        sc.hub.IHubPntBc_B = self.inertia
        sc.hub.omega_BN_BInit = [[self._omega0[0]], [self._omega0[1]], [self._omega0[2]]]
        sc.hub.sigma_BNInit = [[self._sigma0[0]], [self._sigma0[1]], [self._sigma0[2]]]
        sim.AddModelToTask("dynTask", sc)

        if self._orbit is not None:
            self._add_earth_and_orbit(sc)

        ext = extForceTorque.ExtForceTorque()
        ext.ModelTag = "wheelTorque"
        sc.addDynamicEffector(ext)
        sim.AddModelToTask("dynTask", ext)

        if self._viz_live or self._viz_save:
            from Basilisk.utilities import vizSupport

            viz = vizSupport.enableUnityVisualization(
                sim, "dynTask", sc, liveStream=self._viz_live, saveFile=self._viz_save
            )
            if self._viz_live:
                viz.reqComAddress = self._viz_address
                viz.pubComAddress = self._viz_address
                print(
                    f"[sim] live stream: the SIM CONNECTS OUT to Vizard at "
                    f"{self._viz_address}:5556 — start Vizard's 'Start Visualization' "
                    f"(Receive & Reply) so it is listening",
                    flush=True,
                )
            else:
                print(f"[sim] recording Vizard playback file: {self._viz_save}", flush=True)

        sim.InitializeSimulation()
        print(
            f"[sim] plant from {self.vehicle.provenance.describe()}: mass {self.mass_kg:g} kg, "
            f"wheels {[name for name, _ in self.wheels]}",
            flush=True,
        )
        print(
            f"[sim] truth on {self._truth_topic} at {self._rate_hz:g} Hz",
            flush=True,
        )
        self._thread = threading.Thread(
            target=self._run, args=(sim, sc, ext, macros, dt_s), daemon=True
        )
        self._thread.start()

    def _add_earth_and_orbit(self, sc: object) -> None:
        """Give the spacecraft the SAME universe the local plant flies in.

        Earth's mu, radius and J2 come from ``flatsat.sim.orbit`` — the
        plants must share the universe's constants and differ only in how
        faithfully they integrate motion through it. J2 enters as the
        normalized C20 spherical-harmonic coefficient (built in Python:
        the pip wheel ships no coefficient data files); without it,
        Basilisk would silently delete the nodal regression a
        sun-synchronous mission exists to exercise. The initial position
        and velocity are the analytic propagator's epoch state, so both
        plants start from the identical point on the identical orbit.

        Args:
            sc: The Basilisk Spacecraft module.
        """
        from Basilisk.simulation.gravityEffector import SphericalHarmonicsGravityModel
        from Basilisk.utilities import simIncludeGravBody

        assert self._orbit is not None  # caller-checked; narrows the type
        grav_factory = simIncludeGravBody.gravBodyFactory()
        earth = grav_factory.createEarth()
        earth.isCentralBody = True
        earth.mu = orbit.MU_EARTH_M3_S2
        earth.radEquator = orbit.R_EARTH_M
        spherical = SphericalHarmonicsGravityModel()
        spherical.muBody = orbit.MU_EARTH_M3_S2
        spherical.radEquator = orbit.R_EARTH_M
        spherical.maxDeg = 2
        c20_bar = -orbit.J2 / math.sqrt(5.0)  # J2, normalized
        spherical.cBar = [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [c20_bar, 0.0, 0.0]]
        spherical.sBar = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        earth.gravityModel = spherical
        grav_factory.addBodiesTo(sc)

        position0, velocity0 = orbit.propagate_eci(self._orbit, 0.0)
        sc.hub.r_CN_NInit = [[float(v)] for v in position0]  # type: ignore[attr-defined]
        sc.hub.v_CN_NInit = [[float(v)] for v in velocity0]  # type: ignore[attr-defined]

    def _run(self, sim: object, sc: object, ext: object, macros: object, dt_s: float) -> None:
        """Pace the physics with absolute deadlines (drift-free).

        Args:
            sim: The Basilisk SimBaseClass instance.
            sc: The Spacecraft module.
            ext: The ExtForceTorque effector.
            macros: Basilisk's macros module (sec2nano).
            dt_s: Step length.
        """
        sim_t_s = 0.0
        period_ns = int(1_000_000_000 / self._rate_hz)
        next_wake = time.monotonic_ns() + period_ns
        report_s = self._report_every_s
        next_report = time.monotonic() + report_s if report_s > 0 else None
        while not self._stop.is_set():
            # Sum every wheel's applied axis torque into a body torque,
            # through the SAME mounting the flight side used.
            torque_body = [0.0, 0.0, 0.0]
            for sink in self._sinks:
                applied = sink.latest()
                axis = self._axes[sink.wheel]
                for index in range(3):
                    torque_body[index] += axis[index] * applied
            # Magnetorquers: torque is m x B at the LAST PUBLISHED field —
            # exactly the field the flight-side magnetometer saw, one step
            # old at the sim rate, which is what a rod driven off that
            # measurement would actually feel.
            if self._dipole_sinks and self._field_body is not None:
                mtq = magnetorquer_torque(
                    [(self._rod_axes[sink.rod], sink.latest()) for sink in self._dipole_sinks],
                    self._field_body,
                )
                for index in range(3):
                    torque_body[index] += mtq[index]
            ext.extTorquePntB_B = [[torque_body[0]], [torque_body[1]], [torque_body[2]]]  # type: ignore[attr-defined]

            sim_t_s += dt_s
            sim.ConfigureStopTime(macros.sec2nano(sim_t_s))  # type: ignore[attr-defined]
            sim.ExecuteSimulation()  # type: ignore[attr-defined]

            state = sc.scStateOutMsg.read()  # type: ignore[attr-defined]
            omega = state.omega_BN_B
            sigma = state.sigma_BN
            with self._lock:
                self._omega = (float(omega[0]), float(omega[1]), float(omega[2]))
            truth = sim_pb2.TruthState()
            truth.omega_x_rad_s = float(omega[0])
            truth.omega_y_rad_s = float(omega[1])
            truth.omega_z_rad_s = float(omega[2])
            truth.sigma_x = float(sigma[0])
            truth.sigma_y = float(sigma[1])
            truth.sigma_z = float(sigma[2])
            if self._orbit is not None:
                # Position is Basilisk's OWN integrated state — the whole
                # point of this plant — but the field, sun and eclipse
                # evaluated there are the shared orbit.py models.
                position_eci = np.array(
                    [float(state.r_BN_N[0]), float(state.r_BN_N[1]), float(state.r_BN_N[2])]
                )
                dcm = mrp_to_dcm((truth.sigma_x, truth.sigma_y, truth.sigma_z))
                fill_environment(
                    truth,
                    position_eci,
                    dcm,
                    sim_t_s,
                    self._epoch_gmst_rad,
                    self._epoch_solar_angle_rad,
                )
                self._field_body = (truth.mag_field_x_t, truth.mag_field_y_t, truth.mag_field_z_t)
            self._truth_pub.publish(truth)

            now = time.monotonic()
            if next_report is not None and now >= next_report:
                received = ", ".join(
                    [f"{s.wheel} rx {s.messages_received}" for s in self._sinks]
                    + [f"{s.rod} rx {s.messages_received}" for s in self._dipole_sinks]
                )
                environment = ""
                if self._orbit is not None:
                    altitude_km = (
                        math.sqrt(
                            truth.position_x_m**2 + truth.position_y_m**2 + truth.position_z_m**2
                        )
                        - orbit.R_EARTH_M
                    ) / 1e3
                    field_ut = (
                        math.sqrt(
                            truth.mag_field_x_t**2 + truth.mag_field_y_t**2 + truth.mag_field_z_t**2
                        )
                        * 1e6
                    )
                    environment = f"alt={altitude_km:6.1f} km  |B|={field_ut:5.1f} uT  "
                print(
                    f"[sim] t={sim_t_s:7.1f}s  "
                    f"|omega|={self.rate_magnitude() * 1000.0:7.2f} mrad/s  "
                    f"{environment}{received}",
                    flush=True,
                )
                next_report += report_s

            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                self._stop.wait(delta / 1e9)
            elif delta < -5 * period_ns:
                # Fell far behind wall clock (Vizard handshake, laptop nap):
                # a real-time feed DROPS lost time rather than sprinting a
                # faster-than-real-time burst at the flight software.
                next_wake = time.monotonic_ns()
            next_wake += period_ns

    def stop(self) -> None:
        """Stop the physics thread and join it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)


def main() -> int:
    """Run the physics bridge against the flatsat bus.

    Returns:
        0 on clean exit (Ctrl-C).
    """
    parser = argparse.ArgumentParser(description="Basilisk physics bridge for the flatsat bus.")
    parser.add_argument("--vehicle", default=None, help="vehicle composition file")
    parser.add_argument("--rate", type=float, default=100.0, help="sim step + publish rate [Hz]")
    parser.add_argument(
        "--connect", default=None, help="explicit zenoh endpoint, e.g. tcp/jetson.local:7447"
    )
    parser.add_argument(
        "--omega0",
        default="0.08,-0.05,0.03",
        help="initial body rates [rad/s], comma-separated",
    )
    parser.add_argument(
        "--orbit",
        default=None,
        help="saved orbit file, e.g. config/orbits/spacex_rideshare_sso.txtpb; "
        "omitted = attitude-only in an empty universe",
    )
    parser.add_argument(
        "--sigma0",
        default="0,0,0",
        help="initial attitude MRP, comma-separated",
    )
    parser.add_argument("--report-every", type=float, default=5.0, help="status print period [s]")
    parser.add_argument("--viz", action="store_true", help="live-stream to Vizard (start it first)")
    parser.add_argument("--viz-save", default=None, help="record a Vizard playback .bin instead")
    parser.add_argument("--viz-address", default="localhost", help="host Vizard is bound to")
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    omega0 = [float(value) for value in args.omega0.split(",")]
    sigma0 = [float(value) for value in args.sigma0.split(",")]
    orbit_elements = None
    epoch_gmst_rad = 0.0
    epoch_solar_angle_rad = 0.0
    if args.orbit:
        # Lazy import: scenario imports this module, so importing it at
        # the top would be a cycle. At call time it is already loaded.
        from flatsat.sim.scenario import load_orbit

        orbit_elements, epoch_gmst_rad, epoch_solar_angle_rad = load_orbit(args.orbit)
    session = open_session(args.connect)
    plant = BasiliskPlant(
        vehicle,
        session,
        truth_topic=TRUTH_TOPIC,
        omega0=(omega0[0], omega0[1], omega0[2]),
        rate_hz=args.rate,
        sigma0=(sigma0[0], sigma0[1], sigma0[2]),
        orbit_elements=orbit_elements,
        epoch_gmst_rad=epoch_gmst_rad,
        epoch_solar_angle_rad=epoch_solar_angle_rad,
        viz_live=args.viz,
        viz_save=args.viz_save,
        viz_address=args.viz_address,
        report_every_s=args.report_every,
    )
    plant.start()
    print("[sim] Ctrl-C to stop", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[sim] stopped", flush=True)
    finally:
        plant.stop()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
