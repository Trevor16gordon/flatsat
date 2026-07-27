#!/usr/bin/env python3
"""Basilisk hardware-in-the-loop feed: simulated spacecraft on the flatsat bus.

Runs on the GROUND host (Mac — PLAN §10: the sim computer is never the
flight computer). A rigid spacecraft tumbles in Basilisk; every simulation
step this script publishes the body rates as an ``ImuSample`` on
``hal/imu0/sample`` — the exact message and topic the fake-IMU daemon uses,
so the ADCS loop on the Jetson cannot tell the difference. That is the HAL
seam doing its job.

Time discipline (PLAN §10): the sim is paced to WALL CLOCK by this script
(absolute-deadline stepping, the same drift-free pattern as the flight
loops) — the flight software never knows sim time exists.

Closed-loop mode (``--closed-loop``): additionally subscribes to
``adcs/wheel_torque`` and applies each received ``WheelTorqueCommand`` as
the external torque on the simulated hub. With the Jetson's PD loop
running, the spacecraft in the Mac's physics engine detumbles under
control computed on the flight hardware — the full sim -> flight software
-> actuator loop.

Stop the Jetson's synthetic IMU first so two publishers don't fight:
  sudo systemctl stop flatsat-imu0

Usage (from the repo root, ground venv active):
  python -m ground.basilisk_hil                       # open loop: tumble feed
  python -m ground.basilisk_hil --closed-loop         # apply Jetson torques
  python -m ground.basilisk_hil --connect tcp/jetson.local:7447
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import threading
import time

import zenoh

from flight.core.bus import SamplePublisher
from flight.core.config import load_imu_spec, load_vehicle
from flight.hal.models.imu import apply_gyro_model
from flight.msgs import adcs_pb2, hal_pb2


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


class TorqueSink:
    """Latest wheel-torque command, with a stale-command cutoff.

    The quiet state is the default (PLAN §1): if no fresh command arrives
    within ``timeout_s``, the applied torque is ZERO — an actuator must
    never keep flying a dead controller's last order. (First validation run
    proved the hazard: when the flight loop exited, the sim kept applying
    the final torque and spun the spacecraft back up.)
    """

    def __init__(self, session: zenoh.Session, topic: str, timeout_s: float = 0.1) -> None:
        """Subscribe to the command topic.

        Args:
            session: Open zenoh session.
            topic: Command topic to subscribe to.
            timeout_s: Command age beyond which the applied torque is zero.
        """
        self._lock = threading.Lock()
        self._torque = (0.0, 0.0, 0.0)
        self._recv_ns = 0
        self._timeout_ns = int(timeout_s * 1e9)
        self.commands_received = 0
        self._sub = session.declare_subscriber(topic, self._on_command)

    def _on_command(self, sample: zenoh.Sample) -> None:
        """Store the newest torque command.

        Args:
            sample: Incoming WheelTorqueCommand.
        """
        cmd = adcs_pb2.WheelTorqueCommand.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._torque = (cmd.torque_x_n_m, cmd.torque_y_n_m, cmd.torque_z_n_m)
            self._recv_ns = time.monotonic_ns()
            self.commands_received += 1

    def latest(self) -> tuple[float, float, float]:
        """Return the commanded torque, or zero if the command is stale.

        Returns:
            Torque (x, y, z) in N·m; (0, 0, 0) when no fresh command exists.
        """
        with self._lock:
            if time.monotonic_ns() - self._recv_ns > self._timeout_ns:
                return (0.0, 0.0, 0.0)
            return self._torque


def main() -> int:
    """Run the simulated spacecraft against the flatsat bus.

    Returns:
        0 on clean exit (Ctrl-C).
    """
    parser = argparse.ArgumentParser(description="Basilisk HIL feed for the flatsat bus.")
    parser.add_argument("--rate", type=float, default=100.0, help="sim step + publish rate [Hz]")
    parser.add_argument(
        "--closed-loop",
        action="store_true",
        help="apply WheelTorqueCommand from the bus to the simulated hub",
    )
    parser.add_argument(
        "--connect", default=None, help="explicit zenoh endpoint, e.g. tcp/jetson.local:7447"
    )
    parser.add_argument(
        "--omega0",
        default="0.08,-0.05,0.03",
        help="initial body rates [rad/s], comma-separated",
    )
    parser.add_argument("--report-every", type=float, default=5.0, help="status print period [s]")
    parser.add_argument("--seed", type=int, default=42, help="sensor-noise RNG seed")
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

    vehicle = load_vehicle()
    imu_spec = load_imu_spec()
    rng = random.Random(args.seed)
    dt_s = 1.0 / args.rate
    omega0 = [float(v) for v in args.omega0.split(",")]

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("dynProcess")
    proc.addTask(sim.CreateNewTask("dynTask", macros.sec2nano(dt_s)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "flatsat-sim"
    sc.hub.mHub = 10.0  # kg — smallsat-ish
    sc.hub.IHubPntBc_B = [[0.9, 0.0, 0.0], [0.0, 0.8, 0.0], [0.0, 0.0, 0.6]]  # kg m^2
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
    publisher = SamplePublisher(session, vehicle.control.input_topic, imu_spec.name)
    sink = TorqueSink(session, vehicle.control.output_topic) if args.closed_loop else None

    sim.InitializeSimulation()
    mode = "CLOSED loop (applying flight torques)" if sink else "open loop (tumble feed only)"
    for line in imu_spec.describe():
        print(f"[sim] {line}", flush=True)
    print(f"[sim] {mode} at {args.rate:g} Hz — Ctrl-C to stop", flush=True)

    sim_t_s = 0.0
    period_ns = int(1_000_000_000 / args.rate)
    next_wake = time.monotonic_ns() + period_ns
    next_report = time.monotonic() + args.report_every
    try:
        while True:
            if sink is not None:
                tx, ty, tz = sink.latest()
                ext.extTorquePntB_B = [[tx], [ty], [tz]]

            sim_t_s += dt_s
            sim.ConfigureStopTime(macros.sec2nano(sim_t_s))
            sim.ExecuteSimulation()

            state = sc.scStateOutMsg.read()
            omega = state.omega_BN_B
            truth = (float(omega[0]), float(omega[1]), float(omega[2]))
            # Physics gives truth; the DEVICE SPEC decides what the sensor
            # reports — same model the flight-side fake IMU and (later) the
            # real driver use, so HIL exercises the sensor that exists.
            (gx, gy, gz), flags = apply_gyro_model(truth, imu_spec, rng)

            msg = hal_pb2.ImuSample()
            msg.gyro_x_rad_s = gx
            msg.gyro_y_rad_s = gy
            msg.gyro_z_rad_s = gz
            msg.temperature_c = imu_spec.temperature_c
            publisher.publish(msg, validity=flags)

            now = time.monotonic()
            if now >= next_report:
                rate_mag = math.sqrt(sum(float(w) ** 2 for w in omega))
                extra = f", cmds rx {sink.commands_received}" if sink else ""
                print(
                    f"[sim] t={sim_t_s:7.1f}s  |omega|={rate_mag * 1000.0:7.2f} mrad/s{extra}",
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
