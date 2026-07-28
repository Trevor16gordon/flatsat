"""Local plant: rigid-body physics for scenario runs, no Basilisk needed.

The flight software only ever sees the sim CONTRACT — TruthState in,
per-wheel applied torque out — so a small Euler-integrated rigid body
can stand behind the same topics the Basilisk bridge serves. Scenario
tests get the entire flight chain, end to end, in one process on the
flight computer or in CI; Basilisk remains the higher-fidelity
universe-fake on the ground machine.

The plant derives its parameters through the SAME
:func:`~flatsat.sim.basilisk_hil.plant_from_vehicle` the bridge uses, so
neither universe-fake can disagree with the vehicle file.
"""

from __future__ import annotations

import threading
import time
from typing import Protocol

import zenoh

from flatsat.core.bus import SamplePublisher
from flatsat.core.config import VehicleSpec
from flatsat.msgs import sim_pb2
from flatsat.sim.basilisk_hil import WheelTorqueSink, plant_from_vehicle

Vec3 = tuple[float, float, float]


class Plant(Protocol):
    """What the scenario runner needs from any universe-fake.

    :class:`LocalPlant` (quick, on-target/CI) and
    :class:`~flatsat.sim.basilisk_hil.BasiliskPlant` (full dynamics,
    ground machine, optional Vizard 3D) both satisfy it — the mission is
    identical either way.
    """

    def start(self) -> None:
        """Start the physics behind the sim topics."""
        ...

    def stop(self) -> None:
        """Stop the physics and join its thread."""
        ...

    def rate_magnitude(self) -> float:
        """Current |omega| of the simulated body, rad/s."""
        ...


class RigidBody:
    """Euler-integrated rotational dynamics: I omega_dot = tau - omega x (I omega)."""

    def __init__(self, inertia: list[list[float]], omega0: Vec3) -> None:
        """Set the plant's inertia and initial state.

        Args:
            inertia: 3x3 inertia tensor (body frame).
            omega0: Initial body rates.
        """
        self._inertia = inertia
        self.omega: list[float] = list(omega0)

    def _matvec(self, vec: list[float]) -> list[float]:
        """Multiply the inertia tensor by a vector.

        Args:
            vec: The vector.

        Returns:
            I @ vec.
        """
        return [sum(self._inertia[i][j] * vec[j] for j in range(3)) for i in range(3)]

    def step(self, torque_body: Vec3, dt_s: float) -> Vec3:
        """Advance the dynamics one step.

        Args:
            torque_body: External body torque over the step.
            dt_s: Step length.

        Returns:
            The body rates after the step.
        """
        angular_momentum = self._matvec(self.omega)
        gyroscopic = (
            self.omega[1] * angular_momentum[2] - self.omega[2] * angular_momentum[1],
            self.omega[2] * angular_momentum[0] - self.omega[0] * angular_momentum[2],
            self.omega[0] * angular_momentum[1] - self.omega[1] * angular_momentum[0],
        )
        # Diagonal-dominant solve is enough here: scenario vehicles use
        # diagonal inertia; a full solver arrives with a vehicle that needs it.
        for axis in range(3):
            accel = (torque_body[axis] - gyroscopic[axis]) / self._inertia[axis][axis]
            self.omega[axis] += accel * dt_s
        return (self.omega[0], self.omega[1], self.omega[2])

    def rate_magnitude(self) -> float:
        """Current |omega|.

        Returns:
            The body-rate magnitude in rad/s.
        """
        return float(sum(w * w for w in self.omega) ** 0.5)


class LocalPlant:
    """Threaded plant behind the sim topics: truth out, wheel torque in."""

    def __init__(
        self,
        vehicle: VehicleSpec,
        session: zenoh.Session,
        truth_topic: str,
        omega0: Vec3,
        rate_hz: float = 100.0,
    ) -> None:
        """Build the plant from the vehicle file and wire its topics.

        Args:
            vehicle: Loaded vehicle composition ([body] + mounting).
            session: Open zenoh session; not owned — caller closes it.
            truth_topic: Key to publish TruthState on (the same key the
                vehicle's basilisk_imu entries subscribe to).
            omega0: Initial body rates.
            rate_hz: Integration and publish rate.
        """
        _, inertia, wheels = plant_from_vehicle(vehicle)
        self.body = RigidBody(inertia, omega0)
        self._axes = dict(wheels)
        self._sinks = [WheelTorqueSink(session, name) for name, _ in wheels]
        self._truth = SamplePublisher(session, truth_topic, "local_plant")
        self._rate_hz = rate_hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _run(self) -> None:
        """Pace the physics with absolute deadlines (drift-free)."""
        period_ns = int(1_000_000_000 / self._rate_hz)
        dt_s = 1.0 / self._rate_hz
        next_wake = time.monotonic_ns() + period_ns
        while not self._stop.is_set():
            torque = [0.0, 0.0, 0.0]
            for sink in self._sinks:
                applied = sink.latest()
                axis = self._axes[sink.wheel]
                for index in range(3):
                    torque[index] += axis[index] * applied
            self.body.step((torque[0], torque[1], torque[2]), dt_s)

            truth = sim_pb2.TruthState()
            truth.omega_x_rad_s = self.body.omega[0]
            truth.omega_y_rad_s = self.body.omega[1]
            truth.omega_z_rad_s = self.body.omega[2]
            self._truth.publish(truth)

            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                self._stop.wait(delta / 1e9)
            next_wake += period_ns

    def rate_magnitude(self) -> float:
        """Current |omega| of the simulated body.

        Returns:
            The body-rate magnitude in rad/s.
        """
        return self.body.rate_magnitude()

    def start(self) -> None:
        """Start the physics thread."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the physics thread and join it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
