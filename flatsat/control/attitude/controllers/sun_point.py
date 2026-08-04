"""Sun-pointing controller: aim a body axis at the measured sun.

The flagship law, composed from parts that already earned trust:

  * alignment torque ``k_align * (a x s)`` steers the configured body
    axis ``a`` toward the MEASURED sun direction ``s`` — computed from
    the sun sensor directly, no attitude estimate required, which is
    what makes it robust to everything but darkness;
  * rate damping, the same PD the detumble law runs, so a tumbling
    release is caught first and the alignment settles second;
  * the magnetorquer momentum dump, verbatim from momentum_dump — a
    pointing vehicle accumulates momentum too.

In eclipse the sun vanishes and the alignment term PAUSES — rate
damping holds the vehicle still (wherever it points) and the dump keeps
working. Chasing a zero vector would torque toward garbage; holding is
the honest degraded mode.
"""

from __future__ import annotations

import math
from typing import ClassVar

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
from flatsat.control.attitude.controllers.rate_damping import RateDampingController


class SunPointController(AttitudeController):
    """Align + damp on the wheels, dump momentum on the rods."""

    output_kind: ClassVar[str] = "torque_and_dipole"

    def __init__(
        self,
        point_axis: Vec3,
        k_align: float,
        kp: float,
        kd: float,
        max_torque_n_m: float,
        dump_gain: float,
        max_dipole_a_m2: float,
    ) -> None:
        """Configure all three laws.

        Args:
            point_axis: Body axis to aim at the sun (normalized here).
            k_align: Alignment gain on the a x s error vector.
            kp: Rate damping proportional gain.
            kd: Rate damping derivative gain.
            max_torque_n_m: Wheel torque envelope.
            dump_gain: Dipole per unit ``(h x B)/|B|²``.
            max_dipole_a_m2: Per-axis dipole clip.
        """
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

    @classmethod
    def from_config(cls, options: control_options_pb2.SunPointOptions) -> SunPointController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; the axis and every gain are required.

        Returns:
            The configured controller.

        Raises:
            ValueError: If the axis or a required gain is absent.
        """
        if len(options.point_axis) != 3 or not any(options.point_axis):
            raise ValueError("sun_point requires a 3-value non-zero point_axis")
        if not (
            options.HasField("k_align")
            and options.HasField("kp")
            and options.HasField("kd")
            and options.HasField("dump_gain")
        ):
            raise ValueError("sun_point requires k_align, kp, kd and dump_gain")
        return cls(
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
            state: Current estimate; the sun vector and wheel momentum
                ride on it when the vehicle provides them.
            reference: Rate reference for the damping term.
            dt_s: Time since the previous step.

        Returns:
            The clipped commands.
        """
        damping = self._damping.update(state, reference, dt_s)
        torque = list(damping.torque_n_m)
        if state.sun_body is not None and state.sun_visible:
            ax, ay, az = self.point_axis
            sx, sy, sz = state.sun_body
            torque[0] += self.k_align * (ay * sz - az * sy)
            torque[1] += self.k_align * (az * sx - ax * sz)
            torque[2] += self.k_align * (ax * sy - ay * sx)
        clipped = clip_torque((torque[0], torque[1], torque[2]), self.limits)

        dipole: Vec3 = (0.0, 0.0, 0.0)
        dipole_saturated = False
        field = state.mag_field_t
        momentum = state.wheel_momentum_n_m_s
        if field is not None and state.mag_fresh and momentum is not None:
            b_square = field[0] ** 2 + field[1] ** 2 + field[2] ** 2
            if b_square > 0.0:
                hx, hy, hz = momentum
                bx, by, bz = field
                scale = self.dump_gain / b_square
                unclipped = (
                    scale * (hy * bz - hz * by),
                    scale * (hz * bx - hx * bz),
                    scale * (hx * by - hy * bx),
                )
                limit = self.max_dipole_a_m2
                dipole = (
                    max(-limit, min(limit, unclipped[0])),
                    max(-limit, min(limit, unclipped[1])),
                    max(-limit, min(limit, unclipped[2])),
                )
                dipole_saturated = any(abs(v) > limit for v in unclipped)
        return ControlOutput(
            torque_n_m=clipped.torque_n_m,
            dipole_a_m2=dipole,
            # The damping law clips internally, so its own saturation
            # must ride through — the outer clip sees an at-limit value
            # as unclipped.
            saturated=damping.saturated or clipped.saturated or dipole_saturated,
        )

    def reset(self) -> None:
        """Reset the damping law's derivative history."""
        self._damping.reset()

    def describe(self) -> list[str]:
        """Describe the tuning in force.

        Returns:
            One line naming axis, gains, and envelopes.
        """
        return [
            f"controller: sun_point axis={self.point_axis} k_align={self.k_align:g} "
            f"kp={self._damping.kp:g} kd={self._damping.kd:g} "
            f"torque limit={self.limits.max_torque_n_m:g} N·m, "
            f"dump gain={self.dump_gain:g} dipole limit={self.max_dipole_a_m2:g} A·m²"
        ]
