"""Sensor daemon base: the HAL contract mechanics, implemented once.

Every HAL daemon owns driver-level access to one device and publishes typed
messages on the bus (PLAN §4). This base class implements everything the
contract requires that is NOT device-specific:

  * header stamping — sample/publish timestamps (flight clock = wall clock
    for now, per PLAN §10), per-source monotonic sequence number;
  * validity propagation — subclasses report acquisition facts via flags,
    never repair or suppress (flag and forward);
  * a drift-free publish cadence (absolute-deadline loop, the same pattern
    as the RT control loops — lateness never compounds).

Subclasses implement exactly one thing: :meth:`SensorDaemon.read`, which
acquires one sample and MUST NOT raise — acquisition failures are reported
as validity flags on a publishable message, because a silent gap is
indistinguishable from daemon death and a repaired value is a lie FDIR can
never detect.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Protocol

import zenoh

from flight.msgs import hal_pb2


class HalMessage(Protocol):
    """Structural type of every generated sensor message: has a Header."""

    @property
    def header(self) -> hal_pb2.Header:
        """The embedded common envelope (mutable submessage)."""
        ...

    def SerializeToString(self) -> bytes:  # noqa: N802 — protobuf API name
        """Serialize to protobuf wire format."""
        ...


@dataclass(frozen=True)
class SensorConfig:
    """Per-sensor-instance configuration — the lightweight 'add a sensor' knob.

    Attributes:
        name: Daemon instance name; becomes ``Header.source`` (e.g. "thermal_tj").
        topic: Bus key expression to publish on (e.g. "hal/thermal_tj/sample").
        rate_hz: Publish cadence in samples per second.
    """

    name: str
    topic: str
    rate_hz: float


class SensorDaemon(ABC):
    """Base class implementing the PLAN §4 HAL daemon contract mechanics."""

    def __init__(self, config: SensorConfig, session: zenoh.Session) -> None:
        """Bind the daemon to its config and bus session.

        Args:
            config: Instance configuration (name, topic, rate).
            session: Open zenoh session; not owned — caller closes it.
        """
        self.config = config
        self._pub = session.declare_publisher(config.topic)
        self._seq = 0
        self._stop = threading.Event()

    @abstractmethod
    def read(self) -> tuple[HalMessage, int]:
        """Acquire one sample from the device.

        Returns:
            Tuple of (message with body fields filled, validity flags).
            The base class fills the header. Implementations MUST NOT raise:
            acquisition failures are reported via flags on an otherwise
            publishable message (flag and forward, never repair).
        """

    def publish_once(self) -> HalMessage:
        """Acquire, stamp, and publish one sample.

        Returns:
            The published message (header filled), for tests/introspection.
        """
        sample_time_ns = time.time_ns()
        msg, flags = self.read()
        self._seq += 1
        msg.header.source = self.config.name
        msg.header.sample_time_ns = sample_time_ns
        msg.header.seq = self._seq
        msg.header.validity = flags
        msg.header.publish_time_ns = time.time_ns()
        self._pub.put(msg.SerializeToString())
        return msg

    def run(self) -> None:
        """Publish at the configured cadence until :meth:`stop` is called.

        Uses the absolute-deadline pattern: the next wake time is advanced by
        exactly one period each cycle, so scheduling lateness never
        accumulates into drift.
        """
        period_ns = int(1_000_000_000 / self.config.rate_hz)
        next_wake = time.monotonic_ns() + period_ns
        while not self._stop.is_set():
            self.publish_once()
            delta_ns = next_wake - time.monotonic_ns()
            if delta_ns > 0:
                self._stop.wait(delta_ns / 1e9)
            next_wake += period_ns

    def stop(self) -> None:
        """Request the :meth:`run` loop to exit (idempotent, thread-safe)."""
        self._stop.set()
