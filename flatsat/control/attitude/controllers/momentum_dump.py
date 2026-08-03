"""Momentum-dump controller: wheels fly the attitude, rods bleed the wheels.

The combined wheels + torquers system. Two laws run side by side and are
deliberately NOT allowed to fight:

  * torque = PD on body-rate error — delegated to the SAME
    :class:`~flatsat.control.attitude.controllers.rate_damping.
    RateDampingController` a wheels-only vehicle flies, so adding rods
    cannot change how the wheels behave.
  * dipole m = dump_gain·(h x B)/|B|², where h is the body-frame wheel
    momentum. The resulting external torque is -k·h_perp; the attitude
    loop counters it through the wheels, so the wheels hand their stored
    momentum to the field. The component of h ALONG B is untouchable —
    the same underactuation B-dot lives with, applied to momentum — and
    decays only as the orbit rotates the field.

Why dump momentum at all: a wheel that absorbed a deployment tumble is
spinning, and a wheel at its momentum rail has no torque authority left.
Without rods the only way down is thrusters or giving the rate back to
the body. This law is the reason wheel-equipped spacecraft carry
magnetorquers anyway.
"""

from __future__ import annotations

from typing import ClassVar

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
    ControlOutput,
)
from flatsat.control.attitude.controllers.rate_damping import RateDampingController


class MomentumDumpController(AttitudeController):
    """PD rate damping plus magnetorquer momentum bleed."""

    output_kind: ClassVar[str] = "torque_and_dipole"

    def __init__(
        self,
        kp: float,
        kd: float,
        max_torque_n_m: float,
        dump_gain: float,
        max_dipole_a_m2: float,
    ) -> None:
        """Configure both laws.

        Args:
            kp: Proportional gain on rate error [N·m / (rad/s)].
            kd: Derivative gain on rate error [N·m / (rad/s²)].
            max_torque_n_m: Wheel torque envelope.
            dump_gain: Dipole per unit ``(h x B)/|B|²``.
            max_dipole_a_m2: Per-axis dipole clip.
        """
        self._damping = RateDampingController(
            kp=kp, kd=kd, limits=ControlLimits(max_torque_n_m=max_torque_n_m)
        )
        self.limits = self._damping.limits
        self.dump_gain = dump_gain
        self.max_dipole_a_m2 = max_dipole_a_m2

    @classmethod
    def from_config(
        cls, options: control_options_pb2.MomentumDumpOptions
    ) -> MomentumDumpController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; ``kp``, ``kd`` and ``dump_gain`` are
                required (gains have no sensible zero); the envelopes
                default to 1.0.

        Returns:
            The configured controller.

        Raises:
            ValueError: If a required gain is absent.
        """
        if (
            not options.HasField("kp")
            or not options.HasField("kd")
            or not options.HasField("dump_gain")
        ):
            raise ValueError("momentum_dump requires kp, kd and dump_gain")
        return cls(
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
        """Compute the torque and dipole commands for one step.

        The dipole goes quiet — never the torque — when the field or the
        wheel momentum is unavailable: attitude control must not degrade
        because momentum management is blind, and a dump law without a
        trustworthy B would torque in an unknown direction.

        Args:
            state: Current estimated state; field and wheel momentum ride
                on it when the vehicle provides them.
            reference: Target body rates for the damping law.
            dt_s: Time since the previous step in seconds.

        Returns:
            The clipped torque command with the dump dipole alongside.
        """
        damping = self._damping.update(state, reference, dt_s)

        field = state.mag_field_t
        momentum = state.wheel_momentum_n_m_s
        if field is None or not state.mag_fresh or momentum is None:
            return damping
        b_square = field[0] ** 2 + field[1] ** 2 + field[2] ** 2
        if b_square <= 0.0:
            return damping

        hx, hy, hz = momentum
        bx, by, bz = field
        scale = self.dump_gain / b_square
        unclipped = (
            scale * (hy * bz - hz * by),
            scale * (hz * bx - hx * bz),
            scale * (hx * by - hy * bx),
        )
        limit = self.max_dipole_a_m2
        clipped = tuple(max(-limit, min(limit, value)) for value in unclipped)
        saturated = damping.saturated or any(abs(value) > limit for value in unclipped)
        return ControlOutput(
            torque_n_m=damping.torque_n_m,
            dipole_a_m2=(clipped[0], clipped[1], clipped[2]),
            saturated=saturated,
        )

    def reset(self) -> None:
        """Reset the damping law's derivative history (the dump is stateless)."""
        self._damping.reset()

    def describe(self) -> list[str]:
        """Describe the tuning in force.

        Returns:
            Lines naming both laws' gains and envelopes.
        """
        return [
            f"controller: momentum_dump kp={self._damping.kp:g} kd={self._damping.kd:g} "
            f"torque limit={self.limits.max_torque_n_m:g} N·m, "
            f"dump gain={self.dump_gain:g} dipole limit={self.max_dipole_a_m2:g} A·m²"
        ]
