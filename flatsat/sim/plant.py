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
from flatsat.sim import orbit
from flatsat.sim.basilisk_hil import (
    DipoleSink,
    WheelTorqueSink,
    fill_environment,
    magnetorquer_torque,
    mrp_to_dcm,
    plant_from_vehicle,
    rods_from_vehicle,
)

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

    def __init__(
        self, inertia: list[list[float]], omega0: Vec3, sigma0: Vec3 = (0.0, 0.0, 0.0)
    ) -> None:
        """Set the plant's inertia and initial state.

        Args:
            inertia: 3x3 inertia tensor (body frame).
            omega0: Initial body rates.
            sigma0: Initial attitude as a modified Rodrigues parameter.
        """
        self._inertia = inertia
        self.omega: list[float] = list(omega0)
        self.sigma: list[float] = list(sigma0)

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
        self._integrate_attitude(dt_s)
        return (self.omega[0], self.omega[1], self.omega[2])

    def _integrate_attitude(self, dt_s: float) -> None:
        """Advance the MRP attitude by the current body rates.

        MRPs are used rather than quaternions because they are three
        numbers with no norm constraint to drift, which suits a simple
        Euler step. The cost is a singularity at a full rotation, avoided
        by switching to the SHADOW set whenever |sigma| exceeds one —
        the same attitude, described from the other end, and the reason
        this cannot be a plain integration.

        Args:
            dt_s: Step length.
        """
        s0, s1, s2 = self.sigma
        w0, w1, w2 = self.omega
        square = s0 * s0 + s1 * s1 + s2 * s2
        # sigma_dot = 1/4 [(1 - s^2) I + 2 [s x] + 2 s s^T] omega
        common = 1.0 - square
        rate = (
            0.25
            * (
                common * w0
                + 2.0 * (s1 * w2 - s2 * w1)
                + 2.0 * (s0 * s0 * w0 + s0 * s1 * w1 + s0 * s2 * w2)
            ),
            0.25
            * (
                common * w1
                + 2.0 * (s2 * w0 - s0 * w2)
                + 2.0 * (s1 * s0 * w0 + s1 * s1 * w1 + s1 * s2 * w2)
            ),
            0.25
            * (
                common * w2
                + 2.0 * (s0 * w1 - s1 * w0)
                + 2.0 * (s2 * s0 * w0 + s2 * s1 * w1 + s2 * s2 * w2)
            ),
        )
        for axis in range(3):
            self.sigma[axis] += rate[axis] * dt_s
        square = sum(v * v for v in self.sigma)
        if square > 1.0:
            for axis in range(3):
                self.sigma[axis] = -self.sigma[axis] / square

    def dcm_body_from_inertial(self) -> list[list[float]]:
        """Rotation carrying an inertial vector into the body frame.

        Returns:
            A 3x3 direction cosine matrix built from the current MRP,
            by the SAME :func:`~flatsat.sim.basilisk_hil.mrp_to_dcm`
            both universe-fakes share.
        """
        return mrp_to_dcm(self.sigma)

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
        sigma0: Vec3 = (0.0, 0.0, 0.0),
        orbit_elements: orbit.OrbitalElements | None = None,
        epoch_gmst_rad: float = 0.0,
        epoch_solar_angle_rad: float = 0.0,
    ) -> None:
        """Build the plant from the vehicle file and wire its topics.

        Args:
            vehicle: Loaded vehicle composition ([body] + mounting).
            session: Open zenoh session; not owned — caller closes it.
            truth_topic: Key to publish TruthState on (the same key the
                vehicle's basilisk_imu entries subscribe to).
            omega0: Initial body rates.
            rate_hz: Integration and publish rate.
            sigma0: Attitude at epoch, as an MRP.
            orbit_elements: Orbit to propagate; None publishes attitude
                only, which is all a reaction-wheel detumble needs.
            epoch_gmst_rad: Greenwich sidereal angle at epoch, which
                fixes where the tilted dipole points to begin with.
            epoch_solar_angle_rad: Solar longitude at epoch.
        """
        _, inertia, wheels = plant_from_vehicle(vehicle)
        rods = rods_from_vehicle(vehicle)
        self.body = RigidBody(inertia, omega0, sigma0)
        self._orbit = orbit_elements
        self._epoch_gmst_rad = epoch_gmst_rad
        self._epoch_solar_angle_rad = epoch_solar_angle_rad
        self._sim_time_s = 0.0
        self._axes = dict(wheels)
        self._rod_axes = dict(rods)
        self._sinks = [WheelTorqueSink(session, name) for name, _ in wheels]
        self._dipole_sinks = [DipoleSink(session, name) for name, _ in rods]
        self._field_body: Vec3 | None = None
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
            # Magnetorquers: torque is m x B at the LAST PUBLISHED field —
            # the same one-step-old field the flight-side magnetometer
            # saw, shared with the Basilisk plant via magnetorquer_torque.
            if self._dipole_sinks and self._field_body is not None:
                mtq = magnetorquer_torque(
                    [(self._rod_axes[sink.rod], sink.latest()) for sink in self._dipole_sinks],
                    self._field_body,
                )
                for index in range(3):
                    torque[index] += mtq[index]
            self.body.step((torque[0], torque[1], torque[2]), dt_s)

            self._sim_time_s += dt_s
            truth = sim_pb2.TruthState()
            truth.omega_x_rad_s = self.body.omega[0]
            truth.omega_y_rad_s = self.body.omega[1]
            truth.omega_z_rad_s = self.body.omega[2]
            truth.sigma_x = self.body.sigma[0]
            truth.sigma_y = self.body.sigma[1]
            truth.sigma_z = self.body.sigma[2]
            self._fill_environment(truth)
            self._truth.publish(truth)

            delta = next_wake - time.monotonic_ns()
            if delta > 0:
                self._stop.wait(delta / 1e9)
            next_wake += period_ns

    def _fill_environment(self, truth: sim_pb2.TruthState) -> None:
        """Add position, body-frame field and eclipse, if there is an orbit.

        The position comes from the analytic propagator; everything
        evaluated there goes through the shared
        :func:`~flatsat.sim.basilisk_hil.fill_environment`, so this
        plant and the Basilisk one cannot drift apart on what universe
        they inhabit.

        Args:
            truth: The message being filled.
        """
        if self._orbit is None:
            return
        t = self._sim_time_s
        position, _ = orbit.propagate_eci(self._orbit, t)
        fill_environment(
            truth,
            position,
            self.body.dcm_body_from_inertial(),
            t,
            self._epoch_gmst_rad,
            self._epoch_solar_angle_rad,
        )
        self._field_body = (truth.mag_field_x_t, truth.mag_field_y_t, truth.mag_field_z_t)

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
