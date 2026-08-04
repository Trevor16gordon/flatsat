"""Passthrough estimator: the measurement IS the estimate.

Today's behavior made explicit. Body rates are read straight off the gyro
axes; the estimate is valid only when the measurement is fresh and carries
no acquisition flags. The HIL endgame floor — the loop chasing injected
gyro noise — is this estimator's signature, and the seam a real filter
replaces by config edit.
"""

from __future__ import annotations

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeState
from flatsat.control.attitude.estimators.estimator import StateEstimator
from flatsat.msgs import hal_pb2


class PassthroughEstimator(StateEstimator):
    """Forwards gyro measurements unfiltered as the state estimate."""

    @classmethod
    def from_config(cls, options: control_options_pb2.PassthroughOptions) -> PassthroughEstimator:
        """Build from vehicle-file estimator options.

        Args:
            options: Empty by design — passthrough has nothing to configure.

        Returns:
            The estimator.
        """
        return cls()

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
        """Map the measurement directly onto the state estimate.

        Args:
            measurement: Latest IMU sample.
            age_s: Age of the measurement when this step ran.
            fresh: False when the measurement exceeded the staleness
                threshold.
            dt_s: Unused — passthrough carries no dynamics.
            mag: Ignored — passthrough estimates nothing from the field.
            sun: Ignored likewise.
            star: Ignored likewise.

        Returns:
            The estimate: gyro rates verbatim, valid only if fresh and
            unflagged.
        """
        return AttitudeState(
            body_rates_rad_s=(
                measurement.gyro_x_rad_s,
                measurement.gyro_y_rad_s,
                measurement.gyro_z_rad_s,
            ),
            age_s=age_s,
            valid=fresh and measurement.header.validity == hal_pb2.VALIDITY_FLAG_VALID,
        )
