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

from flight.msgs import adcs_pb2, hal_pb2

SENSOR_TOPIC = "hal/imu0/sample"
COMMAND_TOPIC = "adcs/wheel_torque"
GYRO_NOISE_RAD_S = 0.002  # matches the fake IMU's noise floor


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

    def __init__(self, session: zenoh.Session, timeout_s: float = 0.1) -> None:
        """Subscribe to the command topic.

        Args:
            session: Open zenoh session.
            timeout_s: Command age beyond which the applied torque is zero.
        """
        self._lock = threading.Lock()
        self._torque = (0.0, 0.0, 0.0)
        self._recv_ns = 0
        self._timeout_ns = int(timeout_s * 1e9)
        self.commands_received = 0
        self._sub = session.declare_subscriber(COMMAND_TOPIC, self._on_command)

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
    args = parser.parse_args()

    # Basilisk imports live here so --help works without bsk installed.
    from Basilisk.simulation import extForceTorque, spacecraft
    from Basilisk.utilities import SimulationBaseClass, macros

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

    session = open_session(args.connect)
    pub = session.declare_publisher(SENSOR_TOPIC)
    sink = TorqueSink(session) if args.closed_loop else None

    sim.InitializeSimulation()
    mode = "CLOSED loop (applying flight torques)" if sink else "open loop (tumble feed only)"
    print(f"[sim] {mode} at {args.rate:g} Hz — Ctrl-C to stop", flush=True)

    seq = 0
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
            seq += 1
            msg = hal_pb2.ImuSample()
            msg.header.source = "imu0"
            msg.header.seq = seq
            msg.header.sample_time_ns = time.time_ns()
            msg.header.validity = hal_pb2.VALIDITY_FLAG_VALID
            msg.gyro_x_rad_s = float(omega[0]) + random.gauss(0.0, GYRO_NOISE_RAD_S)
            msg.gyro_y_rad_s = float(omega[1]) + random.gauss(0.0, GYRO_NOISE_RAD_S)
            msg.gyro_z_rad_s = float(omega[2]) + random.gauss(0.0, GYRO_NOISE_RAD_S)
            msg.temperature_c = 25.0
            msg.header.publish_time_ns = time.time_ns()
            pub.put(msg.SerializeToString())

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
            next_wake += period_ns
    except KeyboardInterrupt:
        print("\n[sim] stopped", flush=True)
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
