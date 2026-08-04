"""The one implementation of the HAL publish contract (PLAN §4).

Any process that puts a sensor message on the bus — a hardware daemon on
the flight computer, a Basilisk-fed feed on the ground host, a replay tool —
stamps its header HERE. One implementation means source naming, sequence
numbering, and the two timestamps cannot drift apart between producers, and
a change to the contract lands everywhere at once.

The contract, per message:
  * ``source``           who published it
  * ``sample_time_ns``   when the quantity was measured
  * ``publish_time_ns``  when it went on the bus
  * ``seq``              per-source, strictly monotonic, starts at 1
  * ``validity``         acquisition flags; 0 == valid (flag and forward)
"""

from __future__ import annotations

import json
import os
import time
from typing import Protocol

import zenoh

from flatsat.msgs import hal_pb2

# Router endpoints for machines where multicast scouting cannot work —
# a laptop on a VPN (the default route hides the LAN), two Tailscale
# nodes in different networks, a datacenter. Comma-separated zenoh
# endpoints, e.g. "tcp/100.65.0.120:7447".
ENDPOINTS_ENV = "FLATSAT_ZENOH_CONNECT"


def bus_config() -> zenoh.Config:
    """Session config honoring the fleet's router endpoints, if any.

    With ``FLATSAT_ZENOH_CONNECT`` unset this is a default config —
    multicast peer scouting, the LAN behavior. With endpoints set the
    session runs in CLIENT mode against them: a zenoh PEER does not
    route third-party traffic, so two peers joined through a common
    router still cannot talk — a router routes for its CLIENTS. That
    property is the whole reason a zenohd exists in this system.

    Every production ``zenoh.open`` call goes through here, so pointing
    an entire machine at a router is one environment variable
    (systemd units read it from /etc/flatsat/bus.env).

    Returns:
        The config to pass to ``zenoh.open``.
    """
    config: zenoh.Config = zenoh.Config()
    raw = os.environ.get(ENDPOINTS_ENV, "").strip()
    if raw:
        endpoints = [item.strip() for item in raw.split(",") if item.strip()]
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", json.dumps(endpoints))
    return config


class HalMessage(Protocol):
    """Structural type of every generated sensor message: has a Header."""

    @property
    def header(self) -> hal_pb2.Header:
        """The embedded common envelope (mutable submessage)."""
        ...

    def SerializeToString(self) -> bytes:  # noqa: N802 — protobuf API name
        """Serialize to protobuf wire format."""
        ...


class SamplePublisher:
    """Stamps the common header and publishes sensor messages on one topic."""

    def __init__(self, session: zenoh.Session, topic: str, source: str) -> None:
        """Declare the publisher.

        Args:
            session: Open zenoh session; not owned — caller closes it.
            topic: Bus key expression to publish on.
            source: Value stamped into ``Header.source``.
        """
        self._pub = session.declare_publisher(topic)
        self._source = source
        self._seq = 0

    @property
    def seq(self) -> int:
        """Sequence number of the most recently published message."""
        return self._seq

    def publish(
        self,
        msg: HalMessage,
        validity: int = hal_pb2.VALIDITY_FLAG_VALID,
        sample_time_ns: int | None = None,
    ) -> HalMessage:
        """Stamp the header and put the message on the bus.

        Args:
            msg: Message with its body fields already filled.
            validity: Acquisition flags observed while reading (0 == valid).
            sample_time_ns: When the quantity was measured; defaults to now
                (correct only when acquisition is effectively instantaneous —
                callers that know better should pass the real sample time).

        Returns:
            The same message, now stamped, for tests and introspection.
        """
        self._seq += 1
        msg.header.source = self._source
        msg.header.sample_time_ns = sample_time_ns if sample_time_ns else time.time_ns()
        msg.header.seq = self._seq
        msg.header.validity = validity
        msg.header.publish_time_ns = time.time_ns()
        self._pub.put(msg.SerializeToString())
        return msg
