"""What a state estimator IS.

An estimator sits between sensor topics and the controller: measurements
in, state estimate out. Separating it from the control law is what lets a
real filter (EKF, complementary, learned) replace raw-measurement
passthrough as a config swap — the controller keeps seeing the same
:class:`~flatsat.control.attitude.controller.AttitudeState` either way.

Like controllers, estimators are as close to PURE as their strategy
allows: no bus, no clock, no I/O. The loop application owns the clock, so
measurement age and freshness come IN as arguments; stateful estimators
(filters carry covariance) keep their state internal and expose
``reset()``.

Registry: estimators are looked up by the vehicle file's ``estimator``
key; ``passthrough`` is the default and is today's behavior made explicit.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from flatsat.control.attitude.controller import AttitudeState
from flatsat.msgs import hal_pb2


class StateEstimator(ABC):
    """Base class for state estimators feeding the attitude controller."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> StateEstimator:
        """Build an estimator from the vehicle file's estimator options.

        Args:
            options: This estimator's TYPED options message (its oneof
                block in vehicle.proto — see control_options.proto).

        Returns:
            A ready-to-run estimator.
        """

    @abstractmethod
    def update(
        self,
        measurement: hal_pb2.ImuSample,
        age_s: float,
        fresh: bool,
        dt_s: float,
        mag: hal_pb2.MagnetometerSample | None = None,
        sun: hal_pb2.SunSensorSample | None = None,
        star: hal_pb2.StarTrackerSample | None = None,
    ) -> AttitudeState:
        """Fold the measurements into the state estimate.

        Args:
            measurement: Latest IMU sample (its header carries the
                acquisition validity flags).
            age_s: Age of the measurement when this step ran.
            fresh: False when the measurement is older than the loop's
                staleness threshold — the loop owns the clock and the
                threshold; the estimator owns what staleness does to the
                estimate.
            dt_s: Nominal time since the previous step, in seconds.
            mag: Latest magnetometer sample, when the vehicle has one
                and it is fresh; None otherwise.
            sun: Latest sun sensor sample, same contract.
            star: Latest star tracker sample, same contract.

        Returns:
            The state estimate the controller should act on.
        """

    def reset(self) -> None:
        """Clear any internal state (covariance, histories).

        Stateless estimators inherit this no-op; stateful ones override it.
        """
        return

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs/telemetry.

        Returns:
            Lines describing the estimator in force.
        """
        return [f"estimator: {type(self).__name__}"]
