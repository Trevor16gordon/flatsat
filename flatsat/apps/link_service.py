#!/usr/bin/env python3
"""Link service: the managed, scheduled, store-and-forward pipe.

Subscribes the vehicle's named downlink topics, queues their messages
between passes, and transmits during contact windows; whatever arrives
over the air is republished onto the local bus. To every other process
the link is invisible — the correctness test is that everything keeps
working with the link down, just with stale ground data.

Deliberately NOT a middleware tunnel (PLAN §6): the topics carried are
an explicit allowlist in the vehicle file, so discovery chatter and
reliable-QoS retransmission never touch the RF path.

The same executable serves both ends. On the spacecraft it downlinks
telemetry and receives commands; on the ground mirror (``--ground``) it
republishes what it hears under a prefix, keeping flight and ground
topics distinguishable on one desk.

Usage:
  python -m flatsat.apps.link_service
  python -m flatsat.apps.link_service --ground --republish-prefix ground
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import zenoh

from flatsat.comms.link import (
    DEFAULT_QUEUE_LIMIT,
    DEFAULT_SEGMENT_BYTES,
    ContactSchedule,
    Link,
)
from flatsat.core.bus import SamplePublisher, bus_config
from flatsat.core.config import VehicleSpec, load_vehicle, which_impl
from flatsat.core.health import health_topic
from flatsat.core.registry import get_framer_class, get_modem_class
from flatsat.mode.client import ModeClient
from flatsat.msgs import health_pb2


def build_link(vehicle: VehicleSpec, endpoint: str) -> Link:
    """Compose the link stack declared by a vehicle.

    Args:
        vehicle: Loaded vehicle composition.
        endpoint: Endpoint name for modems that need one (loopback).

    Returns:
        The composed link, ready to run.

    Raises:
        ValueError: If the comms block selects no modem or framer.
    """
    comms = vehicle.comms
    modem_name = which_impl(comms, "modem", "comms")
    framer_name = which_impl(comms, "framer", "comms")
    modem = get_modem_class(modem_name).from_config(endpoint, getattr(comms, modem_name))
    framer = get_framer_class(framer_name).from_config(getattr(comms, framer_name))
    return Link(
        modem,
        framer,
        segment_bytes=(
            comms.segment_bytes if comms.HasField("segment_bytes") else DEFAULT_SEGMENT_BYTES
        ),
        queue_limit=(comms.queue_limit if comms.HasField("queue_limit") else DEFAULT_QUEUE_LIMIT),
        contact=ContactSchedule.from_config(comms.contact),
    )


class LinkService:
    """Bus ⇄ link: subscribe, queue, transmit on contact, republish."""

    def __init__(
        self,
        link: Link,
        session: zenoh.Session,
        downlink_topics: list[str],
        republish_prefix: str = "",
        app_name: str = "link",
    ) -> None:
        """Wire the link to the bus.

        Args:
            link: The composed link stack.
            session: Open zenoh session; not owned — caller closes it.
            downlink_topics: Key expressions whose messages are queued
                for transmission (explicit allowlist, never the bus).
            republish_prefix: Prefix applied to received topics before
                republishing — the ground mirror sets this so flight and
                ground traffic stay distinguishable on one desk.
            app_name: Name used for health telemetry.
        """
        self._link = link
        self._session = session
        self._prefix = republish_prefix
        self._health = SamplePublisher(session, health_topic(app_name), app_name)
        self._start_ns = time.monotonic_ns()
        self._stop = threading.Event()
        self._subs = [
            session.declare_subscriber(topic, self._make_handler(topic))
            for topic in downlink_topics
        ]

    def _make_handler(self, topic: str) -> object:
        """Build the subscriber callback that queues one topic.

        Args:
            topic: The subscribed key expression.

        Returns:
            The callback.
        """

        def on_message(sample: zenoh.Sample) -> None:
            """Queue one bus message for the next contact.

            Args:
                sample: The bus message; carried verbatim.
            """
            self._link.enqueue(str(sample.key_expr), bytes(sample.payload.to_bytes()))

        return on_message

    def elapsed_ns(self) -> int:
        """Nanoseconds since the service started (pass phase reference)."""
        return time.monotonic_ns() - self._start_ns

    def pump(self) -> int:
        """Run one service cycle: receive, republish, then transmit.

        Returns:
            Messages republished from the air this cycle.
        """
        now = time.monotonic_ns()
        delivered = self._link.poll(now)
        for topic, payload in delivered:
            self._session.put(f"{self._prefix}/{topic}" if self._prefix else topic, payload)
        self._link.send_pending(self.elapsed_ns())
        return len(delivered)

    def publish_health(self) -> health_pb2.LinkHealth:
        """Publish one window of link health.

        Returns:
            The published message, for tests and introspection.
        """
        msg = health_pb2.LinkHealth()
        msg.in_contact = self._link.in_contact(self.elapsed_ns())
        msg.queued_messages = self._link.queued
        msg.frames_sent = self._link.frames_sent
        msg.frames_received = self._link.frames_received
        msg.frames_dropped_crc = self._link.dropped_frames
        msg.messages_delivered = self._link.messages_delivered
        msg.messages_incomplete = self._link.messages_incomplete
        return self._health.publish(msg)  # type: ignore[return-value]

    def run(self, pump_hz: float = 20.0, health_every_s: float = 10.0) -> None:
        """Pump the link until :meth:`stop` is called.

        Args:
            pump_hz: Service cycle rate.
            health_every_s: Interval between LinkHealth publications.
        """
        period_s = 1.0 / pump_hz
        health_ns = int(health_every_s * 1e9)
        next_health = time.monotonic_ns() + health_ns
        while not self._stop.is_set():
            self.pump()
            if time.monotonic_ns() >= next_health:
                self.publish_health()
                next_health += health_ns
            self._stop.wait(period_s)

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Silence the transmitter, then undeclare bus resources.

        Keying down FIRST: a service that stops pumping leaves the
        carrier wherever the last window put it, and "up" is not an
        acceptable state to exit in (FSW-RADIO-002).
        """
        self._link.close_contact()
        for sub in self._subs:
            sub.undeclare()


def main() -> int:
    """Run the link service from the command line.

    Returns:
        0 on clean exit; 2 when the vehicle declares no link.
    """
    parser = argparse.ArgumentParser(description="Store-and-forward link service.")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    parser.add_argument(
        "--ground",
        action="store_true",
        help="run as the ground mirror (endpoint name 'ground')",
    )
    parser.add_argument(
        "--republish-prefix",
        default="",
        help="prefix received topics before republishing (ground mirror)",
    )
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    if vehicle.comms.WhichOneof("modem") is None:
        print("vehicle declares no comms modem; no link to run", file=sys.stderr)
        return 2

    endpoint = "ground" if args.ground else "flight"
    session = zenoh.open(bus_config())
    link = build_link(vehicle, endpoint)
    service = LinkService(
        link,
        session,
        downlink_topics=list(vehicle.comms.downlink_topics),
        republish_prefix=args.republish_prefix,
    )
    for line in (*vehicle.describe(), *link.describe()):
        print(f"[link] {line}", flush=True)
    print(f"[link] endpoint={endpoint} downlink {list(vehicle.comms.downlink_topics)}", flush=True)

    mode_client = ModeClient(session, "link")
    try:
        service.run()
    except KeyboardInterrupt:
        service.stop()
    finally:
        mode_client.close()
        service.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
