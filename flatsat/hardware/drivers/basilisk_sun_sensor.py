"""Basilisk-fed sun sensor: the truth sun vector through the device model.

Runs under the ORDINARY sensor daemon on the flight computer, exactly
like the basilisk_imu and basilisk_magnetometer — the "device" it owns is
the plant's truth topic. The truth sun vector is already BODY-frame (the
plant rotates it, because that is where a sun sensor measures), so this
driver only corrupts direction through the shared device model and
reports eclipse as honest darkness.

Quiet-state behavior (no plant running) is CORRECT behavior: reads come
back STALE-flagged at full cadence, sun_visible false.
"""

from __future__ import annotations

import random
import threading
import time

import zenoh

from flatsat.core.bus import HalMessage, bus_config
from flatsat.core.config import Provenance, describe_sun_sensor_spec, load_sun_sensor_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.sun_sensor import apply_sun_model
from flatsat.hardware.sensor import SensorDriver
from flatsat.msgs import hal_pb2, sim_pb2

DEFAULT_TRUTH_TOPIC = "sim/truth/state"


class BasiliskSunSensorDriver(SensorDriver):
    """Presents the plant's truth sun vector as a coarse sun sensor."""

    def __init__(
        self,
        spec: devices_pb2.SunSensorDevice,
        truth_topic: str = DEFAULT_TRUTH_TOPIC,
        stale_after_s: float = 0.5,
        seed: int | None = None,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Subscribe to the truth stream.

        Args:
            spec: Device spec supplying the angular noise.
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
        self._sun: tuple[float, float, float] | None = None
        self._in_eclipse = False
        self._recv_ns = 0
        self._sub = self._session.declare_subscriber(truth_topic, self._on_truth)

    def _on_truth(self, sample: zenoh.Sample) -> None:
        """Store the newest truth sample (subscriber thread).

        Args:
            sample: Incoming TruthState.
        """
        msg = sim_pb2.TruthState.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._sun = (msg.sun_x, msg.sun_y, msg.sun_z)
            self._in_eclipse = msg.in_eclipse
            self._recv_ns = time.monotonic_ns()

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskSunSensorOptions
    ) -> BasiliskSunSensorDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Typed options; every field has a default.

        Returns:
            The configured driver.
        """
        spec, prov = load_sun_sensor_spec(options.spec) if options.spec else load_sun_sensor_spec()
        return cls(
            spec=spec,
            truth_topic=options.truth_topic or DEFAULT_TRUTH_TOPIC,
            stale_after_s=(options.stale_after_s if options.HasField("stale_after_s") else 0.5),
            seed=options.seed if options.HasField("seed") else None,
            provenance=prov,
        )

    def read(self) -> tuple[HalMessage, int]:
        """Corrupt the latest truth sun vector through the device model.

        Returns:
            Tuple of (SunSensorSample, validity flags). No fresh truth is
            a STALE-flagged dark sample; a fresh eclipse is an UNFLAGGED
            dark sample — darkness is a measurement, staleness a fault.
        """
        with self._lock:
            sun = self._sun
            in_eclipse = self._in_eclipse
            age_ns = time.monotonic_ns() - self._recv_ns
        msg = hal_pb2.SunSensorSample()
        if sun is None or age_ns > self._stale_after_ns:
            return msg, int(hal_pb2.VALIDITY_FLAG_STALE)
        # A run with no orbit publishes a zero sun vector: report darkness.
        if sun == (0.0, 0.0, 0.0):
            return msg, int(hal_pb2.VALIDITY_FLAG_VALID)
        (sx, sy, sz), visible = apply_sun_model(sun, in_eclipse, self._spec, self._rng)
        msg.sun_x, msg.sun_y, msg.sun_z = sx, sy, sz
        msg.sun_visible = visible
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
            describe_sun_sensor_spec(self._spec, self._provenance) if self._provenance else []
        )
        return [
            f"driver: basilisk_sun_sensor truth={self._truth_topic} "
            f"stale-after {self._stale_after_ns / 1e9:g} s",
            *spec_lines,
        ]
