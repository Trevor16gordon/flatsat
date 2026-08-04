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

from flatsat import vehicle_pb2
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
from flatsat.hardware.models.star_tracker import apply_star_model
from flatsat.hardware.models.sun_sensor import apply_sun_model
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
        sun_angle_deg: Angle between the controller's point axis and the
            TRUE sun direction; NaN without a sun or a pointing law.
        nadir_angle_deg: Angle between the controller's point axis and
            the TRUE Earth direction; NaN without an orbit or an axis.
        attitude_error_deg: Angle between the estimated and TRUE
            attitude; NaN when the estimator produced no attitude.
    """

    times_s: list[float] = field(default_factory=list)
    omega_mag_rad_s: list[float] = field(default_factory=list)
    wheel_momentum_n_m_s: list[float] = field(default_factory=list)
    dipole_mag_a_m2: list[float] = field(default_factory=list)
    imu_temperature_c: list[float] = field(default_factory=list)
    in_eclipse: list[bool] = field(default_factory=list)
    sun_angle_deg: list[float] = field(default_factory=list)
    nadir_angle_deg: list[float] = field(default_factory=list)
    attitude_error_deg: list[float] = field(default_factory=list)

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
    sun_spec = None
    star_spec = None
    for sensor in vehicle.sensors:
        kind = sensor.WhichOneof("options")
        if kind == "basilisk_star_tracker":
            spec_path = sensor.basilisk_star_tracker.spec
            from flatsat.core.config import load_star_tracker_spec

            star_spec, _ = (
                load_star_tracker_spec(spec_path) if spec_path else load_star_tracker_spec()
            )
        elif kind == "basilisk_sun_sensor":
            spec_path = sensor.basilisk_sun_sensor.spec
            from flatsat.core.config import load_sun_sensor_spec

            sun_spec, _ = load_sun_sensor_spec(spec_path) if spec_path else load_sun_sensor_spec()
        elif kind == "basilisk_imu":
            spec_path = sensor.basilisk_imu.spec
            from flatsat.core.config import load_imu_spec

            imu_spec, _ = load_imu_spec(spec_path) if spec_path else load_imu_spec()
        elif kind == "basilisk_magnetometer":
            spec_path = sensor.basilisk_magnetometer.spec
            from flatsat.core.config import load_magnetometer_spec

            mag_spec, _ = (
                load_magnetometer_spec(spec_path) if spec_path else load_magnetometer_spec()
            )
    wheels: list[tuple[Vec3, WheelModel, vehicle_pb2.MountingConfig]] = []
    rods: list[tuple[Vec3, MagnetorquerModel, vehicle_pb2.MountingConfig]] = []
    for entry in vehicle.actuators:
        actuator_kind = which_impl(entry, "options", entry.name)
        axis = (entry.mounting.axis[0], entry.mounting.axis[1], entry.mounting.axis[2])
        if actuator_kind.endswith("reaction_wheel"):
            spec, _prov = load_wheel_spec(getattr(entry, actuator_kind).device)
            wheels.append((axis, WheelModel(spec), entry.mounting))
        elif actuator_kind.endswith("magnetorquer"):
            spec_m, _prov = load_magnetorquer_spec(getattr(entry, actuator_kind).device)
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
        mag_msg = None
        if mag_spec is not None and field_body is not None:
            measured, _mflags = apply_mag_model(field_body, mag_spec, rng)
            mag_msg = hal_pb2.MagnetometerSample()
            mag_msg.mag_x_t, mag_msg.mag_y_t, mag_msg.mag_z_t = measured
        sun_msg = None
        sun_true: Vec3 | None = None
        if sun_spec is not None and orbit_elements is not None:
            sun_true = (truth.sun_x, truth.sun_y, truth.sun_z)
            (sx, sy, sz), visible = apply_sun_model(sun_true, eclipse, sun_spec, rng)
            sun_msg = hal_pb2.SunSensorSample()
            sun_msg.sun_x, sun_msg.sun_y, sun_msg.sun_z = sx, sy, sz
            sun_msg.sun_visible = visible
        star_msg = None
        nadir_true: Vec3 | None = None
        if orbit_elements is not None:
            radius = float(np.linalg.norm(position))
            if radius > 0.0:
                nadir_eci = -np.asarray(position) / radius
                nadir_vec = np.array(body.dcm_body_from_inertial()) @ nadir_eci
                nadir_true = (float(nadir_vec[0]), float(nadir_vec[1]), float(nadir_vec[2]))
        if star_spec is not None:
            sigma_now = body.sigma
            (stx, sty, stz), star_ok = apply_star_model(
                (sigma_now[0], sigma_now[1], sigma_now[2]),
                sun_true if sun_true is not None else (0.0, 0.0, 0.0),
                nadir_true if nadir_true is not None else (0.0, 0.0, 0.0),
                star_spec,
                rng,
            )
            star_msg = hal_pb2.StarTrackerSample()
            star_msg.sigma_x, star_msg.sigma_y, star_msg.sigma_z = stx, sty, stz
            star_msg.star_valid = star_ok
        state = estimator.update(imu, 0.0, True, dt, mag=mag_msg, sun=sun_msg, star=star_msg)
        if mag_msg is not None:
            state = _with_mag(state, (mag_msg.mag_x_t, mag_msg.mag_y_t, mag_msg.mag_z_t))
        if sun_msg is not None:
            state = _with_sun(state, sun_msg)
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
        result.sun_angle_deg.append(_sun_angle_deg(controller, sun_true, eclipse))
        result.nadir_angle_deg.append(_axis_angle_deg(controller, nadir_true))
        result.attitude_error_deg.append(_attitude_error_deg(state, body))
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


def _with_sun(state: AttitudeState, sun_msg: hal_pb2.SunSensorSample) -> AttitudeState:
    """Attach a fresh sun measurement, as the control loop would.

    Args:
        state: The estimator's output.
        sun_msg: The corrupted sun sample.

    Returns:
        The state with the sun riding along.
    """
    from dataclasses import replace

    return replace(
        state,
        sun_body=(sun_msg.sun_x, sun_msg.sun_y, sun_msg.sun_z),
        sun_visible=sun_msg.sun_visible,
    )


def _sun_angle_deg(controller: object, sun_true: Vec3 | None, eclipse: bool) -> float:
    """Pointing error against the TRUE sun, when the law points at one.

    Args:
        controller: The active strategy; only sun-pointing laws qualify.
        sun_true: True body-frame sun direction, or None without orbit.
        eclipse: Whether the vehicle is shadowed (no meaningful angle).

    Returns:
        Degrees between point axis and the true sun; NaN when undefined.
    """
    axis = getattr(controller, "point_axis", None)
    if axis is None or sun_true is None or eclipse:
        return float("nan")
    dot = axis[0] * sun_true[0] + axis[1] * sun_true[1] + axis[2] * sun_true[2]
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _axis_angle_deg(controller: object, target_body: Vec3 | None) -> float:
    """Angle between the controller's point axis and a TRUE direction.

    Args:
        controller: The active strategy; needs a ``point_axis``.
        target_body: True body-frame target direction, or None.

    Returns:
        Degrees between them; NaN when undefined.
    """
    axis = getattr(controller, "point_axis", None)
    if axis is None or target_body is None:
        return float("nan")
    dot = axis[0] * target_body[0] + axis[1] * target_body[1] + axis[2] * target_body[2]
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _attitude_error_deg(state: AttitudeState, body: RigidBody) -> float:
    """Angle between the estimated and true attitude.

    Args:
        state: The estimator output (sigma_bn when attitude_valid).
        body: The plant carrying the true attitude.

    Returns:
        The rotation angle in degrees; NaN without a valid estimate.
    """
    if state.sigma_bn is None or not state.attitude_valid:
        return float("nan")
    from flatsat.sim.basilisk_hil import mrp_to_dcm

    est = np.array(mrp_to_dcm(state.sigma_bn))
    true = np.array(body.dcm_body_from_inertial())
    cos_angle = (float(np.trace(est @ true.T)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


def _with_wheel_momentum(
    state: AttitudeState, wheels: list[tuple[Vec3, WheelModel, vehicle_pb2.MountingConfig]]
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


def _wheel_momentum_mag(
    wheels: list[tuple[Vec3, WheelModel, vehicle_pb2.MountingConfig]],
) -> float:
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
