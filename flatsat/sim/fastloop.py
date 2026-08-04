"""Faster-than-real-time control-law simulation, composed from the vehicle file.

The scenario runner proves the COMPOSITION — daemons, bus, mode manager,
all at wall clock — which is why its missions must finish in seconds and
its vehicles are toy-scaled. This module proves the LAWS: the same
registry-resolved controller, estimator, guidance, and device models the
flight software runs, closed synchronously against the same rigid-body
physics and orbit environment, with no bus and no clock. A two-hour
momentum dump of the real flight vehicle runs in seconds of CPU.

What is faithfully shared with flight (all resolved from the vehicle
file, never re-implemented here): the control/estimator/guidance
composition, the wheel/magnetorquer/IMU/magnetometer device models with
their envelopes and corruption, the mounting projections, and the
orbit/field/eclipse environment. What is deliberately absent: zenoh,
daemon cadences, staleness (every sample is fresh by construction), and
mode management. A law that fails HERE is wrong physics or wrong tuning;
a law that passes here but fails the scenario tier has a composition
problem — the two tiers separate those diagnoses.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from flatsat.control.attitude.controller import AttitudeState
from flatsat.core.config import (
    VehicleSpec,
    load_magnetorquer_spec,
    load_wheel_spec,
    which_impl,
)
from flatsat.core.registry import (
    get_controller_class,
    get_estimator_class,
    get_guidance_class,
)
from flatsat.hardware.actuator import project_body_torque
from flatsat.hardware.models.imu import apply_gyro_model, imu_temperature
from flatsat.hardware.models.magnetometer import apply_mag_model
from flatsat.hardware.models.magnetorquer import MagnetorquerModel
from flatsat.hardware.models.wheel import WheelModel
from flatsat.msgs import hal_pb2, sim_pb2
from flatsat.sim import orbit
from flatsat.sim.basilisk_hil import fill_environment, magnetorquer_torque
from flatsat.sim.plant import RigidBody

Vec3 = tuple[float, float, float]


@dataclass
class FastLoopResult:
    """What a fast run measured, sampled once per control step.

    Attributes:
        times_s: Sim time of each sample.
        omega_mag_rad_s: TRUE body rate magnitude (plant, not gyro).
        wheel_momentum_n_m_s: |body-frame wheel momentum vector|.
        dipole_mag_a_m2: |commanded body dipole|.
        imu_temperature_c: The modeled die temperature.
        in_eclipse: Whether the vehicle was shadowed.
    """

    times_s: list[float] = field(default_factory=list)
    omega_mag_rad_s: list[float] = field(default_factory=list)
    wheel_momentum_n_m_s: list[float] = field(default_factory=list)
    dipole_mag_a_m2: list[float] = field(default_factory=list)
    imu_temperature_c: list[float] = field(default_factory=list)
    in_eclipse: list[bool] = field(default_factory=list)

    @property
    def final_omega_mag_rad_s(self) -> float:
        """|omega| at the end of the run."""
        return self.omega_mag_rad_s[-1] if self.omega_mag_rad_s else float("nan")

    @property
    def final_wheel_momentum_n_m_s(self) -> float:
        """|wheel momentum| at the end of the run."""
        return self.wheel_momentum_n_m_s[-1] if self.wheel_momentum_n_m_s else float("nan")


def run_fast_loop(
    vehicle: VehicleSpec,
    duration_s: float,
    omega0: Vec3,
    sigma0: Vec3 = (0.0, 0.0, 0.0),
    orbit_elements: orbit.OrbitalElements | None = None,
    epoch_gmst_rad: float = 0.0,
    epoch_solar_angle_rad: float = 0.0,
    dt_s: float | None = None,
    seed: int = 7,
) -> FastLoopResult:
    """Fly one vehicle's control laws against the physics, as fast as possible.

    Args:
        vehicle: Loaded vehicle composition; controller, estimator,
            guidance, and every device model are resolved from it.
        duration_s: SIM time to fly (wall time is whatever CPU needs).
        omega0: Initial body rates.
        sigma0: Initial attitude MRP.
        orbit_elements: Orbit to fly; None runs attitude-only, which
            silences every magnetic device (no field, no m x B).
        epoch_gmst_rad: Greenwich sidereal angle at epoch.
        epoch_solar_angle_rad: Solar longitude at epoch.
        dt_s: Step size for BOTH control and physics; defaults to the
            vehicle's control rate. Coarser steps run longer missions in
            less CPU at the cost of integration accuracy — fine for a
            two-hour momentum dump, wrong for a fast tumble.
        seed: RNG seed for the sensor corruption models.

    Returns:
        The sampled trajectory.
    """
    control = vehicle.control
    dt = dt_s if dt_s is not None else 1.0 / control.rate_hz
    rng = random.Random(seed)

    strategy = which_impl(control, "strategy", "control")
    objective = which_impl(control, "objective", "control")
    estimator_name = which_impl(control, "estimator", "control")
    controller = get_controller_class(strategy).from_config(getattr(control, strategy))
    guidance = get_guidance_class(objective).from_config(getattr(control, objective))
    estimator = get_estimator_class(estimator_name).from_config(getattr(control, estimator_name))
    emits_torque = type(controller).output_kind in ("torque", "torque_and_dipole")
    emits_dipole = type(controller).output_kind in ("dipole", "torque_and_dipole")

    # Devices, from the same entries the daemons resolve.
    imu_spec = None
    mag_spec = None
    for sensor in vehicle.sensors:
        kind = sensor.WhichOneof("options")
        if kind == "basilisk_imu":
            spec_path = sensor.basilisk_imu.spec
            from flatsat.core.config import load_imu_spec

            imu_spec, _ = load_imu_spec(spec_path) if spec_path else load_imu_spec()
        elif kind == "basilisk_magnetometer":
            spec_path = sensor.basilisk_magnetometer.spec
            from flatsat.core.config import load_magnetometer_spec

            mag_spec, _ = (
                load_magnetometer_spec(spec_path) if spec_path else load_magnetometer_spec()
            )
    wheels: list[tuple[Vec3, WheelModel, object]] = []
    rods: list[tuple[Vec3, MagnetorquerModel, object]] = []
    for entry in vehicle.actuators:
        kind = which_impl(entry, "options", entry.name)
        axis = (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2])
        if kind.endswith("reaction_wheel"):
            spec, _prov = load_wheel_spec(getattr(entry, kind).device)
            wheels.append((axis, WheelModel(spec), entry.mounting))
        elif kind.endswith("magnetorquer"):
            spec_m, _prov = load_magnetorquer_spec(getattr(entry, kind).device)
            rods.append((axis, MagnetorquerModel(spec_m), entry.mounting))

    _, inertia, _ = _body_inertia(vehicle)
    body = RigidBody(inertia, omega0, sigma0)
    result = FastLoopResult()
    temperature_c = imu_spec.temperature_c if imu_spec is not None else 0.0

    steps = int(duration_s / dt)
    truth = sim_pb2.TruthState()
    for step in range(steps):
        t = step * dt

        # Environment at the CURRENT attitude, exactly the plants' models.
        field_body: Vec3 | None = None
        eclipse = False
        if orbit_elements is not None:
            position, _velocity = orbit.propagate_eci(orbit_elements, t)
            fill_environment(
                truth,
                position,
                body.dcm_body_from_inertial(),
                t,
                epoch_gmst_rad,
                epoch_solar_angle_rad,
            )
            field_body = (truth.mag_field_x_t, truth.mag_field_y_t, truth.mag_field_z_t)
            eclipse = truth.in_eclipse

        # Sensors through the SAME corruption models the drivers run.
        imu = hal_pb2.ImuSample()
        if imu_spec is not None:
            (gx, gy, gz), _flags = apply_gyro_model(
                (body.omega[0], body.omega[1], body.omega[2]), imu_spec, rng
            )
            imu.gyro_x_rad_s, imu.gyro_y_rad_s, imu.gyro_z_rad_s = gx, gy, gz
            temperature_c = imu_temperature(temperature_c, imu_spec, eclipse, dt)
            imu.temperature_c = temperature_c
        state = estimator.update(imu, 0.0, True, dt)
        if mag_spec is not None and field_body is not None:
            measured, _mflags = apply_mag_model(field_body, mag_spec, rng)
            state = _with_mag(state, measured)
        if wheels:
            state = _with_wheel_momentum(state, wheels)

        output = controller.update(state, guidance.reference_at(t), dt)

        # Actuation through the same mounting projections and envelopes.
        torque = [0.0, 0.0, 0.0]
        if emits_torque:
            for axis, wheel, mounting in wheels:
                applied_flags = wheel.apply(project_body_torque(mounting, output.torque_n_m), dt)
                del applied_flags  # envelopes still bind; flags unused here
                for i in range(3):
                    torque[i] += axis[i] * wheel.applied_torque_n_m
        dipole_mag = 0.0
        if emits_dipole and field_body is not None:
            rod_dipoles = []
            for axis, rod, mounting in rods:
                rod.apply(project_body_torque(mounting, output.dipole_a_m2))
                rod_dipoles.append((axis, rod.applied_dipole_a_m2))
                dipole_mag += rod.applied_dipole_a_m2**2
            dipole_mag = math.sqrt(dipole_mag)
            mtq = magnetorquer_torque(rod_dipoles, field_body)
            for i in range(3):
                torque[i] += mtq[i]

        body.step((torque[0], torque[1], torque[2]), dt)

        result.times_s.append(t)
        result.omega_mag_rad_s.append(body.rate_magnitude())
        result.wheel_momentum_n_m_s.append(_wheel_momentum_mag(wheels))
        result.dipole_mag_a_m2.append(dipole_mag)
        result.imu_temperature_c.append(temperature_c)
        result.in_eclipse.append(eclipse)
    return result


def _body_inertia(vehicle: VehicleSpec) -> tuple[float, list[list[float]], None]:
    """The body physical model, shaped like the plants build it.

    Args:
        vehicle: Loaded vehicle composition.

    Returns:
        (mass, 3x3 inertia, None placeholder).
    """
    body = vehicle.require_body()
    flat = list(body.inertia_kg_m2)
    return body.mass_kg, [flat[0:3], flat[3:6], flat[6:9]], None


def _with_mag(state: AttitudeState, measured: Vec3) -> AttitudeState:
    """Attach a fresh field measurement, as the control loop would.

    Args:
        state: The estimator's output.
        measured: Body-frame field from the magnetometer model.

    Returns:
        The state with the field riding along.
    """
    from dataclasses import replace

    return replace(state, mag_field_t=measured, mag_fresh=True)


def _with_wheel_momentum(
    state: AttitudeState, wheels: list[tuple[Vec3, WheelModel, object]]
) -> AttitudeState:
    """Attach the body-frame wheel momentum, as the control loop would.

    Args:
        state: The estimator's output (possibly with mag attached).
        wheels: The wheel models with their axes.

    Returns:
        The state with the momentum vector riding along.
    """
    from dataclasses import replace

    total = [0.0, 0.0, 0.0]
    for axis, wheel, _mounting in wheels:
        for i in range(3):
            total[i] += axis[i] * wheel.momentum_n_m_s
    return replace(state, wheel_momentum_n_m_s=(total[0], total[1], total[2]))


def _wheel_momentum_mag(wheels: list[tuple[Vec3, WheelModel, object]]) -> float:
    """|body-frame wheel momentum| across all wheels.

    Args:
        wheels: The wheel models with their axes.

    Returns:
        The magnitude in N·m·s.
    """
    total = np.zeros(3)
    for axis, wheel, _mounting in wheels:
        total += np.array(axis) * wheel.momentum_n_m_s
    return float(np.linalg.norm(total))
