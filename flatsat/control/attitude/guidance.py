"""Guidance: what the vehicle is supposed to be doing right now.

Guidance answers "what should we track", control answers "how do we track
it". Separating them is what lets one control strategy serve every
objective: detumbling is a constant zero-rate reference, a slew is a
time-varying one, target tracking computes a reference from an estimated
relative geometry.

Only the constant reference exists today (detumble). Trajectory and
pointing sources arrive with the objectives that need them — the seam is
here so those additions do not touch controllers or the loop application.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeReference, Vec3


class ReferenceSource(ABC):
    """Produces the attitude reference the controller should track."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> ReferenceSource:
        """Build a reference source from vehicle-file guidance options.

        Args:
            options: This source's TYPED options message (its oneof block
                in vehicle.proto — see control_options.proto).

        Returns:
            A ready-to-use reference source.
        """

    @abstractmethod
    def reference_at(self, t_s: float) -> AttitudeReference:
        """Return the reference for a given mission-elapsed time.

        Args:
            t_s: Seconds since the loop started.

        Returns:
            The reference to track at that time.
        """

    def describe(self) -> list[str]:
        """Human-readable description for startup logs/telemetry.

        Returns:
            Lines describing the objective in force.
        """
        return [f"guidance: {type(self).__name__}"]


class ConstantRateReference(ReferenceSource):
    """A fixed body-rate target; all zeros is detumble."""

    def __init__(self, body_rates_rad_s: Vec3 = (0.0, 0.0, 0.0)) -> None:
        """Fix the target rates.

        Args:
            body_rates_rad_s: Target body rates held for all time.
        """
        self._reference = AttitudeReference(body_rates_rad_s=body_rates_rad_s)

    @classmethod
    def from_config(cls, options: control_options_pb2.ConstantRateOptions) -> ConstantRateReference:
        """Build from vehicle-file guidance options.

        Args:
            options: Optional ``target_rates_rad_s``; empty = detumble.

        Returns:
            The configured reference source.

        Raises:
            ValueError: If target rates are present but not 3 values.
        """
        values = list(options.target_rates_rad_s)
        if not values:
            return cls()
        if len(values) != 3:
            raise ValueError("target_rates_rad_s must have exactly 3 values")
        return cls((values[0], values[1], values[2]))

    def reference_at(self, t_s: float) -> AttitudeReference:
        """Return the fixed reference.

        Args:
            t_s: Ignored — this reference does not vary with time.

        Returns:
            The constant reference.
        """
        return self._reference

    def describe(self) -> list[str]:
        """Describe the target rates.

        Returns:
            One line naming the objective.
        """
        rates = self._reference.body_rates_rad_s
        objective = "detumble" if rates == (0.0, 0.0, 0.0) else "constant-rate hold"
        return [f"guidance: {objective} target={rates} rad/s"]
