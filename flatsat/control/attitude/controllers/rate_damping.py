"""PD rate-damping controller: the detumble law.

torque = -kp·e - kd·de/dt, where e is the body-rate error against the
reference. With a zero reference this is classic detumbling; the closed-form
decay for a rigid axis is exp(-t/tau) with tau = (I + kd)/kp, which the unit
tests pin numerically.
"""

from __future__ import annotations

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


class RateDampingController(AttitudeController):
    """Proportional-derivative control on body-rate error."""

    def __init__(self, kp: float, kd: float, limits: ControlLimits | None = None) -> None:
        """Configure the gains.

        Args:
            kp: Proportional gain on rate error [N·m / (rad/s)].
            kd: Derivative gain on rate error [N·m / (rad/s²)].
            limits: Actuator envelope; defaults apply when omitted.
        """
        self.kp = kp
        self.kd = kd
        self.limits = limits or ControlLimits()
        self._prev_error: Vec3 = (0.0, 0.0, 0.0)

    @classmethod
    def from_config(cls, options: control_options_pb2.RateDampingOptions) -> RateDampingController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; ``kp`` and ``kd`` are required (a gain
                has no sensible zero), ``max_torque_n_m`` defaults to 1.0.

        Returns:
            The configured controller.

        Raises:
            ValueError: If a required gain is absent.
        """
        if not options.HasField("kp") or not options.HasField("kd"):
            raise ValueError("rate_damping requires kp and kd")
        limit = options.max_torque_n_m if options.HasField("max_torque_n_m") else 1.0
        return cls(kp=options.kp, kd=options.kd, limits=ControlLimits(max_torque_n_m=limit))

    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute the damping torque for one step.

        A non-positive ``dt_s`` skips the derivative term rather than
        dividing by zero: a control law degrades, it does not raise.

        Args:
            state: Current estimated state.
            reference: Target body rates.
            dt_s: Time since the previous step in seconds.

        Returns:
            The clipped torque command.
        """
        error = tuple(
            rate - target
            for rate, target in zip(state.body_rates_rad_s, reference.body_rates_rad_s, strict=True)
        )
        torque: list[float] = []
        for value, prev in zip(error, self._prev_error, strict=True):
            derivative = (value - prev) / dt_s if dt_s > 0.0 else 0.0
            torque.append(-self.kp * value - self.kd * derivative)
        self._prev_error = (error[0], error[1], error[2])
        return clip_torque((torque[0], torque[1], torque[2]), self.limits)

    def reset(self) -> None:
        """Forget the previous error used by the derivative term."""
        self._prev_error = (0.0, 0.0, 0.0)

    def describe(self) -> list[str]:
        """Describe the tuning in force.

        Returns:
            One line naming gains and the torque limit.
        """
        return [
            f"controller: rate_damping kp={self.kp:g} kd={self.kd:g} "
            f"limit={self.limits.max_torque_n_m:g} N·m"
        ]
