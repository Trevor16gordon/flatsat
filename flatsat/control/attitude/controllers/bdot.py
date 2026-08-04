"""B-dot controller: magnetic detumble from the magnetometer alone.

``m = -k · dB/dt`` — when the vehicle tumbles, the field measured in the
body frame rotates, so dB/dt is dominated by ``-omega x B`` and the
commanded dipole produces torque ``m x B`` that opposes the tumble. No
rate gyro, no attitude knowledge: the reason every CubeSat detumbles on
B-dot first is that it works when nothing else is initialized yet.

What this law can NEVER do (physics, not tuning — see
docs/ARCHITECTURE.md, "A magnetorquer has no maximum torque"): produce
torque about the field line. The rate component along B decays only as
the orbit rotates the field (~degrees per minute), so short runs converge
to the along-B floor, not to zero. A success bound tighter than that
floor is a lie about magnetics.

The derivative is a low-passed finite difference: per-sample noise on B
differentiates into large dB/dt spikes, and an unfiltered law would chase
them (chatter at the dipole rail). The filter time constant trades that
chatter against phase lag on the true rotation signal.
"""

from __future__ import annotations

from typing import ClassVar

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlOutput,
    Vec3,
)


class BdotController(AttitudeController):
    """m = -k·dB/dt, clipped to the commandable dipole envelope."""

    output_kind: ClassVar[str] = "dipole"

    def __init__(self, gain: float, max_dipole_a_m2: float, filter_tau_s: float) -> None:
        """Configure the law.

        Args:
            gain: Dipole per unit field rate [A·m² / (T/s)].
            max_dipole_a_m2: Per-axis dipole clip (the device envelope is
                enforced again in the driver; this keeps the command
                honest at the source).
            filter_tau_s: Low-pass time constant on dB/dt.
        """
        self.gain = gain
        self.max_dipole_a_m2 = max_dipole_a_m2
        self.filter_tau_s = filter_tau_s
        self._prev_field: Vec3 | None = None
        self._deriv_t_s: Vec3 = (0.0, 0.0, 0.0)

    @classmethod
    def from_config(cls, options: control_options_pb2.BdotOptions) -> BdotController:
        """Build from vehicle-file control options.

        Args:
            options: Typed options; ``gain`` is required (a gain has no
                sensible zero), ``max_dipole_a_m2`` defaults to 1.0 and
                ``filter_tau_s`` to 0.5.

        Returns:
            The configured controller.

        Raises:
            ValueError: If the gain is absent.
        """
        if not options.HasField("gain"):
            raise ValueError("bdot requires gain")
        return cls(
            gain=options.gain,
            max_dipole_a_m2=(
                options.max_dipole_a_m2 if options.HasField("max_dipole_a_m2") else 1.0
            ),
            filter_tau_s=options.filter_tau_s if options.HasField("filter_tau_s") else 0.5,
        )

    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute the dipole command for one step.

        Quiet unless there is a fresh field measurement AND a previous
        one to difference against: differentiating a stale (frozen) field
        yields zero and then a spike when it unfreezes, and a law that
        invents dB/dt has invented angular rate. The reference is unused —
        B-dot only ever drives rates toward zero.

        Args:
            state: Current estimated state; the field rides on it.
            reference: Ignored (detumble is implicit in the law).
            dt_s: Time since the previous step in seconds.

        Returns:
            The clipped dipole command.
        """
        field = state.mag_field_t
        if field is None or not state.mag_fresh or dt_s <= 0.0:
            self._prev_field = None
            self._deriv_t_s = (0.0, 0.0, 0.0)
            return ControlOutput()
        if self._prev_field is None:
            self._prev_field = field
            return ControlOutput()

        alpha = dt_s / (self.filter_tau_s + dt_s)
        raw = tuple((b - p) / dt_s for b, p in zip(field, self._prev_field, strict=True))
        deriv = tuple(
            prev + alpha * (new - prev) for prev, new in zip(self._deriv_t_s, raw, strict=True)
        )
        self._prev_field = field
        self._deriv_t_s = (deriv[0], deriv[1], deriv[2])

        limit = self.max_dipole_a_m2
        unclipped = [-self.gain * value for value in deriv]
        clipped = [max(-limit, min(limit, value)) for value in unclipped]
        saturated = any(abs(value) > limit for value in unclipped)
        return ControlOutput(
            dipole_a_m2=(clipped[0], clipped[1], clipped[2]), dipole_saturated=saturated
        )

    def reset(self) -> None:
        """Forget the field history behind the derivative and its filter."""
        self._prev_field = None
        self._deriv_t_s = (0.0, 0.0, 0.0)

    def describe(self) -> list[str]:
        """Describe the tuning in force.

        Returns:
            One line naming the gain, dipole clip, and filter constant.
        """
        return [
            f"controller: bdot gain={self.gain:g} A·m²/(T/s) "
            f"limit={self.max_dipole_a_m2:g} A·m² filter tau={self.filter_tau_s:g} s"
        ]
