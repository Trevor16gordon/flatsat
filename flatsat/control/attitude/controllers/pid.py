"""PID rate controller — a second strategy, to keep the interface honest.

Adds an integral term (and its anti-windup) to the PD law, which is what a
real vehicle needs once constant disturbance torques exist: gravity
gradient, residual magnetic dipole, solar pressure. A pure PD law leaves a
standing rate error against a constant disturbance; the integrator removes
it.

Its existence is also the proof that :class:`AttitudeController` is a real
abstraction rather than one implementation wearing a base class: this
strategy is stateful across steps, needs ``reset()``, and carries different
config keys — and the application running it does not change at all.
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


class PidRateController(AttitudeController):
    """Proportional-integral-derivative control on body-rate error."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = 1.0,
        limits: ControlLimits | None = None,
    ) -> None:
        """Configure gains and anti-windup.

        Args:
            kp: Proportional gain on rate error.
            ki: Integral gain on accumulated rate error.
            kd: Derivative gain on rate error.
            integral_limit: Per-axis clamp on the integral accumulator —
                without it, a saturated actuator winds the integrator up
                and the controller overshoots badly on recovery.
            limits: Actuator envelope; defaults apply when omitted.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.limits = limits or ControlLimits()
        self._integral: Vec3 = (0.0, 0.0, 0.0)
        self._prev_error: Vec3 = (0.0, 0.0, 0.0)

    @classmethod
    def from_config(cls, options: control_options_pb2.PidOptions) -> PidRateController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; ``kp``, ``ki``, ``kd`` are required,
                ``integral_limit`` and ``max_torque_n_m`` default to 1.0.

        Returns:
            The configured controller.

        Raises:
            ValueError: If a required gain is absent.
        """
        for gain in ("kp", "ki", "kd"):
            if not options.HasField(gain):
                raise ValueError(f"pid requires {gain}")
        limit = options.max_torque_n_m if options.HasField("max_torque_n_m") else 1.0
        return cls(
            kp=options.kp,
            ki=options.ki,
            kd=options.kd,
            integral_limit=(options.integral_limit if options.HasField("integral_limit") else 1.0),
            limits=ControlLimits(max_torque_n_m=limit),
        )

    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute the PID torque for one step.

        The integrator only accumulates on valid, non-degenerate steps: a
        stale estimate or a bad time step must not silently charge up the
        integral and fire a large command later.

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
        integral: list[float] = []
        torque: list[float] = []
        for value, prev, acc in zip(error, self._prev_error, self._integral, strict=True):
            if state.valid and dt_s > 0.0:
                acc = max(-self.integral_limit, min(self.integral_limit, acc + value * dt_s))
            integral.append(acc)
            derivative = (value - prev) / dt_s if dt_s > 0.0 else 0.0
            torque.append(-self.kp * value - self.ki * acc - self.kd * derivative)

        self._integral = (integral[0], integral[1], integral[2])
        self._prev_error = (error[0], error[1], error[2])
        return clip_torque((torque[0], torque[1], torque[2]), self.limits)

    def reset(self) -> None:
        """Zero the integrator and derivative history."""
        self._integral = (0.0, 0.0, 0.0)
        self._prev_error = (0.0, 0.0, 0.0)

    def describe(self) -> list[str]:
        """Describe the tuning in force.

        Returns:
            One line naming gains, anti-windup clamp, and torque limit.
        """
        return [
            f"controller: pid kp={self.kp:g} ki={self.ki:g} kd={self.kd:g} "
            f"iclamp={self.integral_limit:g} limit={self.limits.max_torque_n_m:g} N·m"
        ]
