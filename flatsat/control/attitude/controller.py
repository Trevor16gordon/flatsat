"""What an attitude controller IS.

Every control strategy — PD rate damping, PID, LQR, an ML policy — answers
the same question: given an estimated state and a reference the vehicle is
supposed to track, what actuator command do I issue?

    update(state, reference, dt_s) -> body torque

Two design choices worth stating, because they are what make the interface
survive contact with strategies that do not exist yet:

  * The controller takes a REFERENCE, not just a measurement. Detumbling is
    "track zero body rate"; pointing is "track this attitude"; trajectory
    following is a time-varying reference. Those become different guidance
    sources feeding the same controller interface, rather than different
    control loops.
  * Controllers are as close to PURE as their strategy allows — no bus, no
    clock, no I/O. State comes in as numbers, torque goes out as numbers.
    Stateful strategies (integral terms, filters, recurrent policies) keep
    that state internal and expose ``reset()``. This is what makes a
    strategy unit-testable, Monte-Carlo-able, and replayable against
    recorded telemetry without any of the flight plumbing.

Registry: strategies are looked up by the vehicle file's ``strategy`` key,
so changing how a spacecraft is flown is a config edit.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class AttitudeState:
    """Estimated vehicle attitude state the controller acts on.

    Attributes:
        body_rates_rad_s: Estimated body angular rates (x, y, z).
        age_s: Age of the underlying measurement when the step ran.
        valid: False when the estimate is not trustworthy (stale input,
            flagged sensor); strategies decide how to degrade.
        mag_field_t: Body-frame magnetic field measurement, tesla; None
            when the vehicle carries no magnetometer (the loop attaches
            it when a mag input topic is configured).
        mag_fresh: False when the field measurement is stale or flagged;
            magnetic strategies must go quiet rather than differentiate
            a frozen value into a phantom dB/dt.
        wheel_momentum_n_m_s: Body-frame wheel momentum vector — each
            wheel's stored momentum mapped through its mounting axis and
            summed. None when the vehicle's wheels are not all reporting
            fresh state; a momentum-dump law must go quiet rather than
            bleed a stale number.
    """

    body_rates_rad_s: Vec3 = (0.0, 0.0, 0.0)
    age_s: float = 0.0
    valid: bool = True
    mag_field_t: Vec3 | None = None
    mag_fresh: bool = False
    wheel_momentum_n_m_s: Vec3 | None = None


@dataclass(frozen=True)
class AttitudeReference:
    """What guidance asks the controller to achieve.

    Attributes:
        body_rates_rad_s: Target body rates; all zeros is detumble.
    """

    body_rates_rad_s: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ControlLimits:
    """Actuator envelope the controller must respect.

    Attributes:
        max_torque_n_m: Per-axis torque magnitude limit; commands are
            clipped to it. Real wheels saturate — a controller that
            pretends otherwise is validated against an actuator that
            cannot be built.
    """

    max_torque_n_m: float = 1.0


@dataclass(frozen=True)
class ControlOutput:
    """What a control step produced.

    Attributes:
        torque_n_m: Commanded body torque (x, y, z); zeros for a dipole
            strategy — a magnetorquer law never speaks torque.
        dipole_a_m2: Commanded body dipole (x, y, z); meaningful only
            when the strategy declares ``output_kind == "dipole"``.
        saturated: True when a limit clipped the command.
    """

    torque_n_m: Vec3 = (0.0, 0.0, 0.0)
    dipole_a_m2: Vec3 = (0.0, 0.0, 0.0)
    saturated: bool = False


class AttitudeController(ABC):
    """Base class for attitude control strategies."""

    limits: ControlLimits = field(default=ControlLimits())

    # Which body-frame command this strategy emits: "torque"
    # (WheelTorqueCommand, the default) or "dipole" (DipoleCommand). The
    # loop builds the matching message; the vehicle file routes it to
    # actuators whose drivers consume that kind.
    output_kind: ClassVar[str] = "torque"

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> AttitudeController:
        """Build a controller from the vehicle file's control options.

        Args:
            options: This strategy's TYPED options message (its oneof
                block in vehicle.proto — see control_options.proto).

        Returns:
            A ready-to-run controller.
        """

    @abstractmethod
    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute one control step.

        Args:
            state: Current estimated state.
            reference: Target the vehicle should track.
            dt_s: Nominal time since the previous step, in seconds.

        Returns:
            The commanded torque and whether limits clipped it.
        """

    def reset(self) -> None:
        """Clear any internal state (integrators, filters, histories).

        Stateless strategies inherit this no-op; stateful ones override it.
        """
        return

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs/telemetry.

        Returns:
            Lines describing the strategy and its tuning.
        """
        return [f"controller: {type(self).__name__}"]


def clip_torque(torque: Vec3, limits: ControlLimits) -> ControlOutput:
    """Clip a torque vector to the actuator envelope.

    Args:
        torque: Unclipped per-axis torque.
        limits: Actuator limits to respect.

    Returns:
        The clipped command, flagged when clipping occurred.
    """
    limit = limits.max_torque_n_m
    clipped = tuple(max(-limit, min(limit, value)) for value in torque)
    saturated = any(abs(value) > limit for value in torque)
    return ControlOutput(torque_n_m=(clipped[0], clipped[1], clipped[2]), saturated=saturated)
