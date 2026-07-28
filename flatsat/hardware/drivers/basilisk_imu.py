"""Basilisk-fed IMU driver: the sim bridge's truth through the device model.

Runs under the ORDINARY sensor daemon on the flight computer — flight-clock
timestamps, sequence numbers, staleness flags, and health telemetry all stay
intact during HIL; no stopping services, no topic squatting. The "device" it
owns is the sim bridge's truth topic, reached over the bus the way a real
part is reached over I2C.

Truth is corrupted through the SAME :func:`~flatsat.hardware.models.imu.
apply_gyro_model` and device spec the local fake used, so a HIL run
exercises the sensor that exists.

Quiet-state behavior (no bridge running) is CORRECT behavior: reads come
back STALE-flagged at full cadence — the daemon keeps its timeline and
downstream sees an honest, degraded sensor, not silence.
"""

from __future__ import annotations

import random
import threading
import time

import zenoh

from flatsat.core.bus import HalMessage
from flatsat.core.config import Provenance, describe_imu_spec, load_imu_spec
from flatsat.hardware import devices_pb2
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.imu import apply_gyro_model
from flatsat.hardware.sensor import SensorDriver
from flatsat.msgs import hal_pb2, sim_pb2

DEFAULT_TRUTH_TOPIC = "sim/truth/state"


class BasiliskImuDriver(SensorDriver):
    """Presents the sim bridge's truth stream as an IMU device."""

    def __init__(
        self,
        spec: devices_pb2.ImuDevice,
        truth_topic: str = DEFAULT_TRUTH_TOPIC,
        stale_after_s: float = 0.5,
        seed: int | None = None,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Subscribe to the truth stream.

        Args:
            spec: Device spec supplying noise, range, and resolution.
            truth_topic: Bus key the bridge publishes TruthState on.
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
        self._session = session if session is not None else zenoh.open(zenoh.Config())
        self._lock = threading.Lock()
        self._truth: tuple[float, float, float] | None = None
        self._recv_ns = 0
        self._sub = self._session.declare_subscriber(truth_topic, self._on_truth)

    def _on_truth(self, sample: zenoh.Sample) -> None:
        """Store the newest truth sample (subscriber thread).

        Args:
            sample: Incoming TruthState.
        """
        msg = sim_pb2.TruthState.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            self._truth = (msg.omega_x_rad_s, msg.omega_y_rad_s, msg.omega_z_rad_s)
            self._recv_ns = time.monotonic_ns()

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskImuOptions
    ) -> BasiliskImuDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Typed options; every field has a default.

        Returns:
            The configured driver.
        """
        spec, prov = load_imu_spec(options.spec) if options.spec else load_imu_spec()
        return cls(
            spec=spec,
            truth_topic=options.truth_topic or DEFAULT_TRUTH_TOPIC,
            stale_after_s=(options.stale_after_s if options.HasField("stale_after_s") else 0.5),
            seed=options.seed if options.HasField("seed") else None,
            provenance=prov,
        )

    def read(self) -> tuple[HalMessage, int]:
        """Corrupt the latest truth through the device model.

        Returns:
            Tuple of (ImuSample, validity flags). No fresh truth — bridge
            not running, network stall — is a STALE-flagged zero sample:
            flag and forward, never silence, never a stale value passed
            off as a measurement.
        """
        with self._lock:
            truth = self._truth
            age_ns = time.monotonic_ns() - self._recv_ns
        msg = hal_pb2.ImuSample()
        msg.temperature_c = self._spec.temperature_c
        if truth is None or age_ns > self._stale_after_ns:
            return msg, int(hal_pb2.VALIDITY_FLAG_STALE)
        (gx, gy, gz), flags = apply_gyro_model(truth, self._spec, self._rng)
        msg.gyro_x_rad_s = gx
        msg.gyro_y_rad_s = gy
        msg.gyro_z_rad_s = gz
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
        spec_lines = describe_imu_spec(self._spec, self._provenance) if self._provenance else []
        return [
            f"driver: basilisk_imu truth={self._truth_topic} "
            f"stale-after {self._stale_after_ns / 1e9:g} s",
            *spec_lines,
        ]
