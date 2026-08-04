"""Star tracker attitude estimator, with TRIAD as the daylight fallback.

The measured MRP straight from the tracker when it has stars — a star
tracker is already an attitude solution, no fusion needed at this
fidelity. When the tracker is BLINDED (sun or Earth in its exclusion
cone) the estimator falls back to TRIAD if an orbit was configured for
the onboard models, and to rates-only otherwise. The ladder degrades
honestly at every rung: ``attitude_valid`` is False the moment nothing
can actually produce an attitude.

This is the eclipse-proof upgrade over plain TRIAD: a star tracker sees
BETTER in shadow, exactly when the sun/mag pair goes blind.
"""

from __future__ import annotations

from dataclasses import replace

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeState
from flatsat.control.attitude.estimators.estimator import StateEstimator
from flatsat.control.attitude.estimators.triad import TriadEstimator
from flatsat.msgs import hal_pb2


class StarAttitudeEstimator(StateEstimator):
    """Tracker attitude when valid, TRIAD fallback, rates always."""

    def __init__(self, fallback: TriadEstimator | None) -> None:
        """Bind the optional daylight fallback.

        Args:
            fallback: TRIAD estimator to consult when the tracker is
                blinded; None degrades straight to rates-only.
        """
        self._fallback = fallback

    @classmethod
    def from_config(cls, options: control_options_pb2.StarAttitudeOptions) -> StarAttitudeEstimator:
        """Build from vehicle-file estimator options.

        Args:
            options: Typed options; ``orbit`` enables the TRIAD fallback.

        Returns:
            The configured estimator.
        """
        fallback = None
        if options.HasField("orbit") and options.orbit:
            fallback = TriadEstimator.from_config(
                control_options_pb2.TriadOptions(orbit=options.orbit)
            )
        return cls(fallback)

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
        """Fold the best available attitude source into the estimate.

        Args:
            measurement: Latest IMU sample; rates pass through.
            age_s: Age of the IMU measurement.
            fresh: False when the IMU measurement is stale.
            dt_s: Nominal step (keeps the fallback's onboard clock).
            mag: Latest magnetometer sample, for the fallback.
            sun: Latest sun sensor sample, for the fallback.
            star: Latest star tracker sample; used when star_valid.

        Returns:
            The estimate; ``attitude_valid`` reflects whichever rung of
            the ladder actually produced an attitude this step.
        """
        # The fallback ALWAYS steps, valid or not — its onboard clock
        # must keep time even while the star tracker carries the load.
        if self._fallback is not None:
            state = self._fallback.update(
                measurement, age_s, fresh, dt_s, mag=mag, sun=sun, star=star
            )
        else:
            rates = (
                measurement.gyro_x_rad_s,
                measurement.gyro_y_rad_s,
                measurement.gyro_z_rad_s,
            )
            state = AttitudeState(
                body_rates_rad_s=rates,
                age_s=age_s,
                valid=fresh and measurement.header.validity == hal_pb2.VALIDITY_FLAG_VALID,
            )
        if star is not None and star.star_valid:
            return replace(
                state,
                sigma_bn=(star.sigma_x, star.sigma_y, star.sigma_z),
                attitude_valid=True,
            )
        return state

    def describe(self) -> list[str]:
        """Describe the ladder in force.

        Returns:
            One line per rung.
        """
        lines = ["estimator: star_attitude (tracker MRP when star_valid)"]
        if self._fallback is not None:
            lines += [f"estimator: fallback {line}" for line in self._fallback.describe()]
        else:
            lines.append("estimator: no fallback — blinded tracker degrades to rates-only")
        return lines
