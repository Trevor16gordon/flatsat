"""Basilisk-fed star tracker: truth attitude through the device model.

Runs under the ORDINARY sensor daemon on the flight computer, like every
basilisk_* sensor — the "device" it owns is the plant's truth topic. The
driver computes the body-frame nadir from the truth position + attitude
(for Earth blinding) and hands everything to the shared model.

Quiet-state behavior (no plant running) is CORRECT behavior: reads come
back STALE-flagged at full cadence, star_valid false. A BLINDED tracker
in a live run is different: fresh, unflagged, star_valid false — the
condition is real, the device is fine.
"""

from __future__ import annotations

import random
import threading
import time

import numpy as np
import zenoh

from flatsat.control.attitude.estimators.triad import mrp_to_dcm
from flatsat.core.bus import HalMessage, bus_config
from flatsat.core.config import Provenance, describe_star_tracker_spec, load_star_tracker_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.star_tracker import apply_star_model
from flatsat.hardware.sensor import SensorDriver
from flatsat.msgs import hal_pb2, sim_pb2

DEFAULT_TRUTH_TOPIC = "sim/truth/state"


class BasiliskStarTrackerDriver(SensorDriver):
    """Presents the plant's truth attitude as a star tracker."""

    def __init__(
        self,
        spec: devices_pb2.StarTrackerDevice,
        truth_topic: str = DEFAULT_TRUTH_TOPIC,
        stale_after_s: float = 0.5,
        seed: int | None = None,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Subscribe to the truth stream.

        Args:
            spec: Device spec (noise, boresight, exclusion cones).
            truth_topic: Bus key the plant publishes TruthState on.
            stale_after_s: Truth age beyond which reads are STALE-flagged.
            seed: RNG seed; None uses the shared generator.
            session: Zenoh session to reuse (tests); the driver opens its
                own when omitted — the bus IS this device's wire.
            provenance: The spec file's provenance, for describe().
        """
        self._spec = spec
        self._provenance = provenance
        self._truth_topic = truth_topic
        self._stale_after_ns = int(stale_after_s * 1e9)
        self._rng = random.Random(seed) if seed is not None else None
        self._owns_session = session is None
        self._session = session if session is not None else zenoh.open(bus_config())
        self._lock = threading.Lock()
        self._sigma: tuple[float, float, float] | None = None
        self._sun: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._nadir: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._recv_ns = 0
        self._sub = self._session.declare_subscriber(truth_topic, self._on_truth)

    def _on_truth(self, sample: zenoh.Sample) -> None:
        """Store the newest truth attitude and blinding geometry.

        Args:
            sample: Incoming TruthState.
        """
        msg = sim_pb2.TruthState.FromString(bytes(sample.payload.to_bytes()))
        sigma = (msg.sigma_x, msg.sigma_y, msg.sigma_z)
        position = np.array([msg.position_x_m, msg.position_y_m, msg.position_z_m])
        nadir = (0.0, 0.0, 0.0)
        radius = float(np.linalg.norm(position))
        if radius > 0.0:
            # Earth direction in the BODY frame — where the tracker's
            # exclusion cone lives.
            nadir_body = mrp_to_dcm(sigma) @ (-position / radius)
            nadir = (float(nadir_body[0]), float(nadir_body[1]), float(nadir_body[2]))
        with self._lock:
            self._sigma = sigma
            self._sun = (msg.sun_x, msg.sun_y, msg.sun_z)
            self._nadir = nadir
            self._recv_ns = time.monotonic_ns()

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskStarTrackerOptions
    ) -> BasiliskStarTrackerDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Typed options; every field has a default.

        Returns:
            The configured driver.
        """
        spec, prov = (
            load_star_tracker_spec(options.spec) if options.spec else load_star_tracker_spec()
        )
        return cls(
            spec=spec,
            truth_topic=options.truth_topic or DEFAULT_TRUTH_TOPIC,
            stale_after_s=(options.stale_after_s if options.HasField("stale_after_s") else 0.5),
            seed=options.seed if options.HasField("seed") else None,
            provenance=prov,
        )

    def read(self) -> tuple[HalMessage, int]:
        """Corrupt the latest truth attitude through the device model.

        Returns:
            Tuple of (StarTrackerSample, validity flags). No fresh truth
            is a STALE-flagged invalid sample; a fresh blinded read is an
            UNFLAGGED invalid sample — blinding is a condition, staleness
            a fault.
        """
        with self._lock:
            sigma = self._sigma
            sun = self._sun
            nadir = self._nadir
            age_ns = time.monotonic_ns() - self._recv_ns
        msg = hal_pb2.StarTrackerSample()
        if sigma is None or age_ns > self._stale_after_ns:
            return msg, int(hal_pb2.VALIDITY_FLAG_STALE)
        (sx, sy, sz), valid = apply_star_model(sigma, sun, nadir, self._spec, self._rng)
        msg.sigma_x, msg.sigma_y, msg.sigma_z = sx, sy, sz
        msg.star_valid = valid
        return msg, int(hal_pb2.VALIDITY_FLAG_VALID)

    def close(self) -> None:
        """Undeclare the truth subscription and any owned session."""
        self._sub.undeclare()
        if self._owns_session:
            self._session.close()

    def describe(self) -> list[str]:
        """Describe the truth source and the device spec in force.

        Returns:
            Lines naming the topic and spec provenance.
        """
        spec_lines = (
            describe_star_tracker_spec(self._spec, self._provenance) if self._provenance else []
        )
        return [
            f"driver: basilisk_star_tracker truth={self._truth_topic} "
            f"stale-after {self._stale_after_ns / 1e9:g} s",
            *spec_lines,
        ]
