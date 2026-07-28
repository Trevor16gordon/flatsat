#!/usr/bin/env python3
"""Basilisk bridge: the universe-fake, slimmed to physics + truth topics.

Runs on the GROUND host (Mac — the sim computer is never the flight
computer). A rigid spacecraft integrates in Basilisk; every step this
bridge publishes rigid-body TRUTH on ``sim/truth/state`` and folds each
wheel's applied axis torque (from the flight-side
``basilisk_reaction_wheel`` drivers) into the plant.

The plant is built FROM THE VEHICLE FILE — ``[body]`` mass/inertia, one
torque input per declared actuator mapped through ITS mounting — so the
sim's spacecraft and the flight software's model of it cannot diverge by
accident. Deliberate divergence (a ``truth_overrides`` block: "actual
inertia 8% higher than believed") is the future robustness-campaign knob.

Device corruption lives in the flight-side drivers (shared
``hardware/models``), NOT here: flight-clock timestamps, sequence
numbers, staleness flags, and health telemetry all stay in the daemon
machinery during HIL. No stopping services, no topic squatting.

Time discipline: the sim is paced to WALL CLOCK with absolute deadlines
(the same drift-free pattern as the flight loops); lost time is dropped,
never replayed — the flight software never knows sim time exists.

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
) -> tuple[float, list[list[float]], list[tuple[str, tuple[float, float, float]]]]:
    """Derive the simulation plant from the vehicle file.

    This is the no-accidental-divergence property as a function: the same
    ``[body]`` and ``mounting`` entries the flight software reads become
    the physics engine's parameters.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        Tuple of (mass_kg, 3x3 inertia matrix as nested lists, list of
        (wheel name, body-frame spin axis) per declared actuator).

    Raises:
        KeyError: If the vehicle declares no ``[body]`` physical model.
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
    parser.add_argument(
        "--viz",
        action="store_true",
        help=(
            "live-stream to Vizard. NOTE: the SIM is the client — start Vizard "
            "first in its hosting/'Direct Communication' mode on port 5556, "
            "then run this; the sim blocks until Vizard answers"
        ),
    )
    parser.add_argument(
        "--viz-save",
        default=None,
        help=(
            "record a Vizard playback file instead of live streaming (no "
            "sockets, nothing to time out): open the resulting .bin in Vizard"
        ),
    )
    parser.add_argument(
        "--viz-address",
        default="localhost",
        help=(
            "host Vizard is bound to. Basilisk's default is 0.0.0.0, which is "
            "a valid BIND address but not a valid DIAL address on macOS/BSD — "
            "so live streaming silently times out there unless this is set"
        ),
    )
    args = parser.parse_args()

    # Basilisk imports live here so --help works without bsk installed.
    from Basilisk.simulation import extForceTorque, spacecraft
    from Basilisk.utilities import SimulationBaseClass, macros

    vehicle = load_vehicle(args.vehicle)
    mass_kg, inertia, wheels = plant_from_vehicle(vehicle)
    dt_s = 1.0 / args.rate
    omega0 = [float(v) for v in args.omega0.split(",")]

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("dynProcess")
    proc.addTask(sim.CreateNewTask("dynTask", macros.sec2nano(dt_s)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "flatsat-sim"
    sc.hub.mHub = mass_kg  # FROM THE VEHICLE FILE — never a literal here
    sc.hub.IHubPntBc_B = inertia
    sc.hub.omega_BN_BInit = [[omega0[0]], [omega0[1]], [omega0[2]]]
    sc.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask("dynTask", sc)

    ext = extForceTorque.ExtForceTorque()
    ext.ModelTag = "wheelTorque"
    sc.addDynamicEffector(ext)
    sim.AddModelToTask("dynTask", ext)

    if args.viz or args.viz_save:
        from Basilisk.utilities import vizSupport

        viz = vizSupport.enableUnityVisualization(
            sim, "dynTask", sc, liveStream=bool(args.viz), saveFile=args.viz_save
        )
        if args.viz:
            viz.reqComAddress = args.viz_address
            viz.pubComAddress = args.viz_address
            print(
                f"[sim] live stream: the SIM CONNECTS OUT to Vizard at "
                f"{args.viz_address}:5556 — start Vizard's 'Start Visualization' "
                f"(Receive & Reply) so it is listening"
            )
        else:
            print(f"[sim] recording Vizard playback file: {args.viz_save}")

    session = open_session(args.connect)
    truth_pub = SamplePublisher(session, TRUTH_TOPIC, "sim_truth")
    sinks = [WheelTorqueSink(session, name) for name, _ in wheels]
    axes = {name: axis for name, axis in wheels}

    sim.InitializeSimulation()
    print(
        f"[sim] plant from {vehicle.provenance.describe()}: mass {mass_kg:g} kg, "
        f"wheels {[name for name, _ in wheels]}",
        flush=True,
    )
    print(f"[sim] truth on {TRUTH_TOPIC} at {args.rate:g} Hz — Ctrl-C to stop", flush=True)

    sim_t_s = 0.0
    period_ns = int(1_000_000_000 / args.rate)
    next_wake = time.monotonic_ns() + period_ns
    next_report = time.monotonic() + args.report_every
    try:
        while True:
            # Sum every wheel's applied axis torque into a body torque,
            # through the SAME mounting the flight side used.
            torque_body = [0.0, 0.0, 0.0]
            for sink in sinks:
                u = sink.latest()
                axis = axes[sink.wheel]
                torque_body[0] += axis[0] * u
                torque_body[1] += axis[1] * u
                torque_body[2] += axis[2] * u
            ext.extTorquePntB_B = [[torque_body[0]], [torque_body[1]], [torque_body[2]]]

            sim_t_s += dt_s
            sim.ConfigureStopTime(macros.sec2nano(sim_t_s))
            sim.ExecuteSimulation()

            state = sc.scStateOutMsg.read()
            omega = state.omega_BN_B
            sigma = state.sigma_BN
            truth = sim_pb2.TruthState()
            truth.omega_x_rad_s = float(omega[0])
            truth.omega_y_rad_s = float(omega[1])
            truth.omega_z_rad_s = float(omega[2])
            truth.sigma_x = float(sigma[0])
            truth.sigma_y = float(sigma[1])
            truth.sigma_z = float(sigma[2])
            truth_pub.publish(truth)

            now = time.monotonic()
            if now >= next_report:
                rate_mag = math.sqrt(sum(float(w) ** 2 for w in omega))
                rx = ", ".join(f"{s.wheel} rx {s.messages_received}" for s in sinks)
                print(
                    f"[sim] t={sim_t_s:7.1f}s  |omega|={rate_mag * 1000.0:7.2f} mrad/s  {rx}",
                    flush=True,
                )
                next_report += args.report_every

            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                time.sleep(delta / 1e9)
            elif delta < -5 * period_ns:
                # Fell far behind wall clock (Vizard handshake, laptop nap):
                # a real-time feed DROPS lost time rather than sprinting a
                # faster-than-real-time burst at the flight software.
                next_wake = time.monotonic_ns()
            next_wake += period_ns
    except KeyboardInterrupt:
        print("\n[sim] stopped", flush=True)
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
