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
from collections.abc import Mapping

from flatsat.control.attitude.controller import AttitudeState
from flatsat.msgs import hal_pb2


class StateEstimator(ABC):
    """Base class for state estimators feeding the attitude controller."""

    @classmethod
    @abstractmethod
    def from_config(cls, options: Mapping[str, object]) -> StateEstimator:
        """Build an estimator from the vehicle file's estimator options.

        Args:
            options: Estimator-specific settings (filter tunings, models).

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
    ) -> AttitudeState:
        """Fold one measurement into the state estimate.

        Args:
            measurement: Latest sensor sample (its header carries the
                acquisition validity flags).
            age_s: Age of the measurement when this step ran.
            fresh: False when the measurement is older than the loop's
                staleness threshold — the loop owns the clock and the
                threshold; the estimator owns what staleness does to the
                estimate.
            dt_s: Nominal time since the previous step, in seconds.

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
