"""Basilisk-fed magnetometer: the truth field through the device model.

Runs under the ORDINARY sensor daemon on the flight computer, exactly like
the basilisk_imu — the "device" it owns is the plant's truth topic. The
truth field is already BODY-frame (the plant rotates it, because that is
where a magnetometer measures), so this driver only corrupts it through
the shared device model.

Quiet-state behavior (no plant running) is CORRECT behavior: reads come
back STALE-flagged at full cadence. A run without an orbit publishes a
zero field, which the device reports honestly as noise around zero — a
B-dot law fed by it commands nothing, which is the right answer in a
universe with no field.
"""

from __future__ import annotations

import random
import threading
import time

import zenoh

from flatsat.core.bus import HalMessage, bus_config
from flatsat.core.config import Provenance, describe_magnetometer_spec, load_magnetometer_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.magnetometer import apply_mag_model
from flatsat.hardware.sensor import SensorDriver
from flatsat.msgs import hal_pb2, sim_pb2

DEFAULT_TRUTH_TOPIC = "sim/truth/state"


class BasiliskMagnetometerDriver(SensorDriver):
    """Presents the plant's truth field as a magnetometer device."""

    def __init__(
        self,
        spec: devices_pb2.MagnetometerDevice,
        truth_topic: str = DEFAULT_TRUTH_TOPIC,
        stale_after_s: float = 0.5,
        seed: int | None = None,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Subscribe to the truth stream.

        Args:
            spec: Device spec supplying noise, range, and resolution.
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
        self._field: tuple[float, float, float] | None = None
        self._recv_ns = 0
        self._sub = self._session.declare_subscriber(truth_topic, self._on_truth)

    def _on_truth(self, sample: zenoh.Sample) -> None:
        """Store the newest truth sample (subscriber thread).

        Args:
            sample: Incoming TruthState.
        """
        msg = sim_pb2.TruthState.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._field = (msg.mag_field_x_t, msg.mag_field_y_t, msg.mag_field_z_t)
            self._recv_ns = time.monotonic_ns()

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskMagnetometerOptions
    ) -> BasiliskMagnetometerDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Typed options; every field has a default.

        Returns:
            The configured driver.
        """
        spec, prov = (
            load_magnetometer_spec(options.spec) if options.spec else load_magnetometer_spec()
        )
        return cls(
            spec=spec,
            truth_topic=options.truth_topic or DEFAULT_TRUTH_TOPIC,
            stale_after_s=(options.stale_after_s if options.HasField("stale_after_s") else 0.5),
            seed=options.seed if options.HasField("seed") else None,
            provenance=prov,
        )

    def read(self) -> tuple[HalMessage, int]:
        """Corrupt the latest truth field through the device model.

        Returns:
            Tuple of (MagnetometerSample, validity flags). No fresh truth
            is a STALE-flagged zero sample: flag and forward, never
            silence, never a stale value passed off as a measurement.
        """
        with self._lock:
            field = self._field
            age_ns = time.monotonic_ns() - self._recv_ns
        msg = hal_pb2.MagnetometerSample()
        if field is None or age_ns > self._stale_after_ns:
            return msg, int(hal_pb2.VALIDITY_FLAG_STALE)
        (bx, by, bz), flags = apply_mag_model(field, self._spec, self._rng)
        msg.mag_x_t = bx
        msg.mag_y_t = by
        msg.mag_z_t = bz
        return msg, flags

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
            describe_magnetometer_spec(self._spec, self._provenance) if self._provenance else []
        )
        return [
            f"driver: basilisk_magnetometer truth={self._truth_topic} "
            f"stale-after {self._stale_after_ns / 1e9:g} s",
            *spec_lines,
        ]
