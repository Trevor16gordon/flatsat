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

import zenoh

from flatsat.core.bus import SamplePublisher
from flatsat.core.config import VehicleSpec, load_vehicle
from flatsat.hardware.drivers.basilisk_reaction_wheel import wheel_torque_topic
from flatsat.msgs import sim_pb2

TRUTH_TOPIC = "sim/truth/state"

Vec3 = tuple[float, float, float]


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
        (wheel name, body-frame spin axis) per declared actuator).

    Raises:
        KeyError: If the vehicle declares no body physical model.
    """
    body = vehicle.require_body()
    flat = list(body.inertia_kg_m2)
    inertia = [flat[0:3], flat[3:6], flat[6:9]]
    wheels = [
        (entry.name, (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2]))
        for entry in vehicle.actuators
    ]
    return body.mass_kg, inertia, wheels


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
        self._rate_hz = rate_hz
        self._viz_live = viz_live
        self._viz_save = viz_save
        self._viz_address = viz_address
        self._report_every_s = report_every_s
        self.mass_kg, self.inertia, self.wheels = plant_from_vehicle(vehicle)
        self._axes = dict(self.wheels)
        self._sinks = [WheelTorqueSink(session, name) for name, _ in self.wheels]
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
        sc.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        sim.AddModelToTask("dynTask", sc)

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
            self._truth_pub.publish(truth)

            now = time.monotonic()
            if next_report is not None and now >= next_report:
                received = ", ".join(f"{s.wheel} rx {s.messages_received}" for s in self._sinks)
                print(
                    f"[sim] t={sim_t_s:7.1f}s  "
                    f"|omega|={self.rate_magnitude() * 1000.0:7.2f} mrad/s  {received}",
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
    parser.add_argument("--report-every", type=float, default=5.0, help="status print period [s]")
    parser.add_argument("--viz", action="store_true", help="live-stream to Vizard (start it first)")
    parser.add_argument("--viz-save", default=None, help="record a Vizard playback .bin instead")
    parser.add_argument("--viz-address", default="localhost", help="host Vizard is bound to")
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    omega0 = [float(value) for value in args.omega0.split(",")]
    session = open_session(args.connect)
    plant = BasiliskPlant(
        vehicle,
        session,
        truth_topic=TRUTH_TOPIC,
        omega0=(omega0[0], omega0[1], omega0[2]),
        rate_hz=args.rate,
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
