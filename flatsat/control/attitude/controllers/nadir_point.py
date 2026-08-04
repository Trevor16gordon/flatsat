"""Nadir-pointing controller: aim a body axis at the Earth.

Nothing on this vehicle MEASURES the Earth direction, so unlike
sun_point (which steers on the raw sun sensor) this law runs on
knowledge: the estimator's attitude rotates the onboard orbit model's
``-r_hat`` into the body frame, and the same ``k_align * (a x n)``
alignment plus rate damping steers the axis onto it.

That dependency is embraced honestly:

  * no valid attitude (with a sun/mag TRIAD that means eclipse, or a
    degenerate vector pair) -> the alignment PAUSES and rate damping
    holds the vehicle still. A star-tracker estimator removes the
    eclipse hole without touching this file.
  * the onboard orbit MUST match the plant's — same contract as the
    TRIAD estimator, same KEEP CONSISTENT note in the vehicle file —
    and the controller's clock accumulates the loop dt from ITS start,
    so the flight stack must start alongside the plant (an epoch skew
    of minutes rotates the computed nadir by the orbit rate).

The magnetorquer momentum dump runs throughout, verbatim from
momentum_dump — Earth-pointing vehicles accumulate momentum too.
"""

from __future__ import annotations

import math
from typing import ClassVar

import numpy as np

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
    ControlOutput,
    Vec3,
    clip_torque,
)
from flatsat.control.attitude.controllers.momentum_dump import dump_dipole
from flatsat.control.attitude.controllers.rate_damping import RateDampingController
from flatsat.control.attitude.estimators.triad import mrp_to_dcm
from flatsat.sim import orbit
from flatsat.sim.orbit_config import load_orbit


class NadirPointController(AttitudeController):
    """Align + damp on the wheels toward computed nadir, dump on the rods."""

    output_kind: ClassVar[str] = "torque_and_dipole"

    def __init__(
        self,
        elements: orbit.OrbitalElements,
        point_axis: Vec3,
        k_align: float,
        kp: float,
        kd: float,
        max_torque_n_m: float,
        dump_gain: float,
        max_dipole_a_m2: float,
    ) -> None:
        """Configure the laws and bind the onboard orbit.

        Args:
            elements: The orbit the vehicle believes it flies.
            point_axis: Body axis to aim at Earth's center (normalized).
            k_align: Alignment gain on the a x nadir error vector.
            kp: Rate damping proportional gain.
            kd: Rate damping derivative gain.
            max_torque_n_m: Wheel torque envelope.
            dump_gain: Dipole per unit ``(h x B)/|B|²``.
            max_dipole_a_m2: Per-axis dipole clip.
        """
        self._elements = elements
        norm = math.sqrt(sum(v * v for v in point_axis))
        self.point_axis: Vec3 = (
            point_axis[0] / norm,
            point_axis[1] / norm,
            point_axis[2] / norm,
        )
        self.k_align = k_align
        self._damping = RateDampingController(
            kp=kp, kd=kd, limits=ControlLimits(max_torque_n_m=max_torque_n_m)
        )
        self.limits = self._damping.limits
        self.dump_gain = dump_gain
        self.max_dipole_a_m2 = max_dipole_a_m2
        self._t_s = 0.0
        # Off-target angle from the last step, for status displays only.
        self.last_target_angle_deg: float | None = None

    @classmethod
    def from_config(cls, options: control_options_pb2.NadirPointOptions) -> NadirPointController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; orbit, axis and every gain required.

        Returns:
            The configured controller.

        Raises:
            ValueError: If the orbit, axis or a required gain is absent.
        """
        if not options.orbit:
            raise ValueError("nadir_point requires an orbit file — Earth is computed, not sensed")
        if len(options.point_axis) != 3 or not any(options.point_axis):
            raise ValueError("nadir_point requires a 3-value non-zero point_axis")
        if not (
            options.HasField("k_align")
            and options.HasField("kp")
            and options.HasField("kd")
            and options.HasField("dump_gain")
        ):
            raise ValueError("nadir_point requires k_align, kp, kd and dump_gain")
        elements, _gmst, _solar = load_orbit(options.orbit)
        return cls(
            elements=elements,
            point_axis=(options.point_axis[0], options.point_axis[1], options.point_axis[2]),
            k_align=options.k_align,
            kp=options.kp,
            kd=options.kd,
            max_torque_n_m=(options.max_torque_n_m if options.HasField("max_torque_n_m") else 1.0),
            dump_gain=options.dump_gain,
            max_dipole_a_m2=(
                options.max_dipole_a_m2 if options.HasField("max_dipole_a_m2") else 1.0
            ),
        )

    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute the torque (align + damp) and dipole (dump) commands.

        Args:
            state: Current estimate; the attitude, field and wheel
                momentum ride on it when the vehicle provides them.
            reference: Rate reference for the damping term.
            dt_s: Time since the previous step.

        Returns:
            The clipped commands.
        """
        self._t_s += dt_s
        damping = self._damping.update(state, reference, dt_s)
        torque = list(damping.torque_n_m)
        self.last_target_angle_deg = None
        if state.sigma_bn is not None and state.attitude_valid:
            position, _velocity = orbit.propagate_eci(self._elements, self._t_s)
            nadir_inertial = -position / np.linalg.norm(position)
            nadir_body = mrp_to_dcm(state.sigma_bn) @ nadir_inertial
            ax, ay, az = self.point_axis
            nx, ny, nz = float(nadir_body[0]), float(nadir_body[1]), float(nadir_body[2])
            torque[0] += self.k_align * (ay * nz - az * ny)
            torque[1] += self.k_align * (az * nx - ax * nz)
            torque[2] += self.k_align * (ax * ny - ay * nx)
            dot = ax * nx + ay * ny + az * nz
            self.last_target_angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        clipped = clip_torque((torque[0], torque[1], torque[2]), self.limits)

        maybe_dipole, dipole_saturated = dump_dipole(state, self.dump_gain, self.max_dipole_a_m2)
        dipole: Vec3 = maybe_dipole if maybe_dipole is not None else (0.0, 0.0, 0.0)
        return ControlOutput(
            torque_n_m=clipped.torque_n_m,
            dipole_a_m2=dipole,
            torque_saturated=damping.torque_saturated or clipped.torque_saturated,
            dipole_saturated=dipole_saturated,
        )

    def reset(self) -> None:
        """Reset the damping law's history; the onboard clock keeps running."""
        self._damping.reset()

    def describe(self) -> list[str]:
        """Describe the tuning and onboard orbit in force.

        Returns:
            Lines naming axis, gains, envelopes, and the believed orbit.
        """
        return [
            f"controller: nadir_point axis={self.point_axis} k_align={self.k_align:g} "
            f"kp={self._damping.kp:g} kd={self._damping.kd:g} "
            f"torque limit={self.limits.max_torque_n_m:g} N·m, "
            f"dump gain={self.dump_gain:g} dipole limit={self.max_dipole_a_m2:g} A·m²",
            f"controller: nadir_point onboard orbit "
            f"a={self._elements.semi_major_axis_m / 1e3:.0f} km "
            f"i={math.degrees(self._elements.inclination_rad):.1f} deg",
        ]
