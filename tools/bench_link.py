#!/usr/bin/env python3
"""Bench space link: both ends of the store-and-forward pipe, one desk.

The two-bus rule (PLAN §6) says middleware never tunnels over the space
link: ground traffic and flight traffic live in separate namespaces,
and the ONLY bridge between them is a pair of link services carrying an
explicit, directional topic allowlist through framing, queues, and
contact windows. On a bench with no second radio, the loopback modem is
in-process — so this tool runs BOTH ends in one process, exactly like
the link-service test fixture, and stands where the RF hop will:

    mission control (Mac)            flight software (this box)
      publishes ground/<topic>          plain topics only
             |                                ^
             v                                |
      [ground link svc]  == loopback ==  [flight link svc]
      subscribes ground/<uplink>         subscribes <downlink>
      republishes ground/<downlink>      republishes <uplink> plain

Ground tools address the ground namespace (`uplink_send
--topic-prefix ground`, `mode_request --topic-prefix ground`); their
messages queue at the ground station and cross ONLY while a contact
window is open. Flight-native topics are produced solely by the flight
link service deframing what actually crossed.

Usage (on the flight computer, alongside the running stack):
  ~/venvs/flatsat-ml/bin/python tools/bench_link.py
  ~/venvs/flatsat-ml/bin/python tools/bench_link.py \
      --vehicle config/vehicles/flatsat_v1_mldemo.txtpb
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flatsat.apps.link_service import LinkService, build_link  # noqa: E402
from flatsat.core.bus import bus_config  # noqa: E402
from flatsat.core.config import load_vehicle  # noqa: E402


class GroundLinkService(LinkService):
    """Ground mirror that strips its namespace before transmission.

    The topic name rides the link verbatim, so the ground side must
    queue `uplink/artifact/chunk`, not `ground/uplink/artifact/chunk` —
    otherwise the flight side would republish a ground-namespace key
    onto the flight bus and no flight consumer would hear it.
    """

    def __init__(self, *args: object, strip_prefix: str = "", **kwargs: object) -> None:
        """Wire the mirror.

        Args:
            *args: Passed through to :class:`LinkService`.
            strip_prefix: Namespace to remove from subscribed keys
                before they are queued for transmission.
            **kwargs: Passed through to :class:`LinkService`.
        """
        self._strip = f"{strip_prefix}/" if strip_prefix else ""
        super().__init__(*args, **kwargs)  # type: ignore[arg-type]

    def _make_handler(self, topic: str) -> object:
        """Build a queueing callback that strips the ground namespace.

        Args:
            topic: The subscribed key expression.

        Returns:
            The callback.
        """
        del topic

        def on_message(sample: zenoh.Sample) -> None:
            """Queue one ground-bus message under its flight-native name.

            Args:
                sample: The bus message; payload carried verbatim.
            """
            key = str(sample.key_expr)
            key = key.removeprefix(self._strip)
            self._link.enqueue(key, bytes(sample.payload.to_bytes()))

        return on_message


def main() -> int:
    """Run both link ends until interrupted.

    Returns:
        0 on clean exit; 2 when the vehicle declares no link.
    """
    parser = argparse.ArgumentParser(description="Bench space link (both ends, one desk).")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    parser.add_argument("--prefix", default="ground", help="ground namespace prefix")
    parser.add_argument("--pump-hz", type=float, default=20.0)
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    comms = vehicle.comms
    if comms.WhichOneof("modem") is None:
        print("vehicle declares no comms modem; no link to run", file=sys.stderr)
        return 2
    if not comms.uplink_topics:
        print(
            "vehicle declares no uplink_topics; commands could never reach flight", file=sys.stderr
        )
        return 2

    flight_session = zenoh.open(bus_config())
    ground_session = zenoh.open(bus_config())
    flight = LinkService(
        build_link(vehicle, "flight"),
        flight_session,
        downlink_topics=list(comms.downlink_topics),
        app_name="link",
    )
    ground = GroundLinkService(
        build_link(vehicle, "ground"),
        ground_session,
        downlink_topics=[f"{args.prefix}/{topic}" for topic in comms.uplink_topics],
        republish_prefix=args.prefix,
        app_name="ground_link",
        strip_prefix=args.prefix,
    )
    contact = comms.contact
    window = (
        f"contact {contact.duration_s:g}s every {contact.period_s:g}s"
        if comms.HasField("contact")
        else "always in contact"
    )
    print(
        f"[bench-link] up: {len(comms.uplink_topics)} uplink topics "
        f"(ground namespace '{args.prefix}/'), "
        f"{len(comms.downlink_topics)} downlink topics; {window}",
        flush=True,
    )

    period_s = 1.0 / args.pump_hz
    was_in_contact = False
    next_health = time.monotonic() + 10.0
    try:
        while True:
            to_ground = ground.pump()  # downlink arrivals republished as ground/<topic>
            to_flight = flight.pump()  # uplink arrivals republished flight-native
            in_contact = flight._link.in_contact(flight.elapsed_ns())
            if in_contact != was_in_contact:
                state = "OPEN" if in_contact else "closed"
                print(
                    f"[bench-link] contact {state}  "
                    f"(queued: {ground._link.queued} up, {flight._link.queued} down)",
                    flush=True,
                )
                was_in_contact = in_contact
            if to_flight:
                print(f"[bench-link] {to_flight} message(s) delivered to FLIGHT", flush=True)
            if to_ground:
                print(f"[bench-link] {to_ground} message(s) delivered to GROUND", flush=True)
            if time.monotonic() >= next_health:
                flight.publish_health()
                ground.publish_health()
                next_health += 10.0
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        flight.close()
        ground.close()
        flight_session.close()
        ground_session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
