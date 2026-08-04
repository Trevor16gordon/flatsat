"""TRIAD attitude estimation from the magnetometer and sun sensor.

The deterministic two-vector method: two directions measured in the body
frame (B and sun) plus the same two directions predicted in the inertial
frame by ONBOARD MODELS give the full rotation between the frames — no
filter, no initialization, no history. Coarse (degrees, set by the sun
sensor and the dipole field model) but absolutely dependable, which is
why it has flown since the 1960s.

The onboard models are ``flatsat/sim/orbit.py`` — deliberately the SAME
pure-python models the plants fly. That is not a shortcut, it is the
testability property: an estimator whose reference models only exist in
the simulator can never be checked against it. Time is kept by
accumulating the loop's own dt from a configured epoch; the drift this
causes against the plant's clock moves the reference vectors by roughly
the orbit rate (~0.06 deg/s of offset), far below the sensor noise for
any realistic startup skew.

In eclipse the sun vector vanishes and one vector is not an attitude:
``attitude_valid`` goes False and the gyro rates keep passing through —
degrade honestly, never extrapolate silently.
"""

from __future__ import annotations

import math

import numpy as np

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeState, Vec3
from flatsat.control.attitude.estimators.estimator import StateEstimator
from flatsat.msgs import hal_pb2
from flatsat.sim import orbit
from flatsat.sim.orbit_config import load_orbit


def dcm_to_mrp(dcm: np.ndarray) -> Vec3:
    """Convert a direction cosine matrix to a modified Rodrigues parameter.

    Via the quaternion (Shepperd's method for the largest component),
    then the standard q -> MRP projection, picking the set with
    |sigma| <= 1 so the output matches the plants' shadow-set discipline.

    Args:
        dcm: 3x3 rotation matrix (body from inertial).

    Returns:
        The MRP (sigma_BN).
    """
    trace = float(np.trace(dcm))
    candidates = [
        1.0 + trace,
        1.0 + 2.0 * dcm[0, 0] - trace,
        1.0 + 2.0 * dcm[1, 1] - trace,
        1.0 + 2.0 * dcm[2, 2] - trace,
    ]
    largest = int(np.argmax(candidates))
    q = np.zeros(4)  # scalar-first
    if largest == 0:
        q[0] = 0.5 * math.sqrt(candidates[0])
        q[1] = (dcm[1, 2] - dcm[2, 1]) / (4.0 * q[0])
        q[2] = (dcm[2, 0] - dcm[0, 2]) / (4.0 * q[0])
        q[3] = (dcm[0, 1] - dcm[1, 0]) / (4.0 * q[0])
    else:
        i = largest - 1
        j = (i + 1) % 3
        k = (i + 2) % 3
        qi = 0.5 * math.sqrt(candidates[largest])
        q[i + 1] = qi
        q[0] = (dcm[j, k] - dcm[k, j]) / (4.0 * qi)
        q[j + 1] = (dcm[i, j] + dcm[j, i]) / (4.0 * qi)
        q[k + 1] = (dcm[i, k] + dcm[k, i]) / (4.0 * qi)
    if q[0] < 0.0:
        q = -q  # short rotation, so the MRP stays inside the unit ball
    denom = 1.0 + q[0]
    return (float(q[1] / denom), float(q[2] / denom), float(q[3] / denom))


def triad_dcm(
    primary_body: np.ndarray,
    secondary_body: np.ndarray,
    primary_inertial: np.ndarray,
    secondary_inertial: np.ndarray,
) -> np.ndarray | None:
    """The TRIAD solution: body-from-inertial from two vector pairs.

    The primary vector is honored exactly; the secondary only fixes the
    rotation about it — so the more trusted measurement (here the sun,
    whose model error is tiny compared to a dipole field's) goes first.

    Args:
        primary_body: Primary direction measured in the body frame.
        secondary_body: Secondary direction measured in the body frame.
        primary_inertial: Primary direction from the onboard model.
        secondary_inertial: Secondary direction from the onboard model.

    Returns:
        The 3x3 rotation, or None when the pair is degenerate (vectors
        parallel — one direction of information short).
    """
    t1_b = primary_body / np.linalg.norm(primary_body)
    t1_i = primary_inertial / np.linalg.norm(primary_inertial)
    cross_b = np.cross(t1_b, secondary_body)
    cross_i = np.cross(t1_i, secondary_inertial)
    norm_b = float(np.linalg.norm(cross_b))
    norm_i = float(np.linalg.norm(cross_i))
    if norm_b < 1e-6 or norm_i < 1e-6:
        return None
    t2_b = cross_b / norm_b
    t2_i = cross_i / norm_i
    t3_b = np.cross(t1_b, t2_b)
    t3_i = np.cross(t1_i, t2_i)
    body = np.column_stack((t1_b, t2_b, t3_b))
    inertial = np.column_stack((t1_i, t2_i, t3_i))
    return np.asarray(body @ inertial.T)


class TriadEstimator(StateEstimator):
    """Two-vector deterministic attitude, rates passed through."""

    def __init__(
        self,
        elements: orbit.OrbitalElements,
        epoch_gmst_rad: float,
        epoch_solar_angle_rad: float,
    ) -> None:
        """Bind the onboard models' orbit and epoch.

        Args:
            elements: The orbit the vehicle believes it flies.
            epoch_gmst_rad: Greenwich sidereal angle at epoch.
            epoch_solar_angle_rad: Solar longitude at epoch.
        """
        self._elements = elements
        self._epoch_gmst_rad = epoch_gmst_rad
        self._epoch_solar_angle_rad = epoch_solar_angle_rad
        self._t_s = 0.0

    @classmethod
    def from_config(cls, options: control_options_pb2.TriadOptions) -> TriadEstimator:
        """Build from vehicle-file estimator options.

        Args:
            options: Typed options; ``orbit`` is required — TRIAD without
                reference models is not an estimator, it is a guess.

        Returns:
            The configured estimator.

        Raises:
            ValueError: If no orbit file is configured.
        """
        if not options.orbit:
            raise ValueError("triad requires an orbit file for its onboard models")
        elements, gmst, solar = load_orbit(options.orbit)
        return cls(elements, gmst, solar)

    def update(
        self,
        measurement: hal_pb2.ImuSample,
        age_s: float,
        fresh: bool,
        dt_s: float,
        mag: hal_pb2.MagnetometerSample | None = None,
        sun: hal_pb2.SunSensorSample | None = None,
    ) -> AttitudeState:
        """Fold the vector measurements into a full attitude when possible.

        Args:
            measurement: Latest IMU sample; rates pass through.
            age_s: Age of the IMU measurement.
            fresh: False when the IMU measurement is stale.
            dt_s: Nominal step, accumulated as the onboard clock.
            mag: Latest magnetometer sample; required for attitude.
            sun: Latest sun sensor sample; required and sun_visible.

        Returns:
            The estimate; ``attitude_valid`` only when both vectors were
            usable this step.
        """
        self._t_s += dt_s
        rates = (
            measurement.gyro_x_rad_s,
            measurement.gyro_y_rad_s,
            measurement.gyro_z_rad_s,
        )
        state = AttitudeState(
            body_rates_rad_s=rates,
            age_s=age_s,
            valid=fresh and measurement.header.validity == hal_pb2.VALIDITY_FLAG_VALID,
        )
        if mag is None or sun is None or not sun.sun_visible:
            return state

        position, _velocity = orbit.propagate_eci(self._elements, self._t_s)
        mag_inertial = orbit.magnetic_field_eci(position, self._t_s, self._epoch_gmst_rad)
        sun_inertial = orbit.sun_direction_eci(self._t_s, self._epoch_solar_angle_rad)
        dcm = triad_dcm(
            np.array([sun.sun_x, sun.sun_y, sun.sun_z]),
            np.array([mag.mag_x_t, mag.mag_y_t, mag.mag_z_t]),
            sun_inertial,
            mag_inertial,
        )
        if dcm is None:
            return state
        from dataclasses import replace

        return replace(state, sigma_bn=dcm_to_mrp(dcm), attitude_valid=True)

    def describe(self) -> list[str]:
        """Describe the onboard models in force.

        Returns:
            One line naming the orbit the estimator believes in.
        """
        return [
            f"estimator: triad onboard orbit a={self._elements.semi_major_axis_m / 1e3:.0f} km "
            f"i={math.degrees(self._elements.inclination_rad):.1f} deg"
        ]
