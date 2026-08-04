"""Suite-wide bus resilience: survive a machine whose multicast is broken.

The tests discover each other's zenoh sessions the same way flight
software does — multicast scouting on the LAN. That mechanism belongs to
the operating system, and it has real failure modes this repo has now
met in the field: overlay VPNs (Tailscale, AnyConnect) capturing the
multicast egress route, so two sessions in the SAME pytest process stop
hearing each other while remote peers stay visible. When that happens,
every cross-session test times out and the suite reads like a hundred
real regressions.

So: before any test runs, probe multicast health with two throwaway
sessions. Healthy — do nothing; the tests run exactly as production
does. Broken — open a loopback hub session and patch ``zenoh.open`` so
every test session also connects to it (tcp/127.0.0.1, which no VPN
captures), and SAY SO loudly. The fallback changes discovery topology,
not semantics, and only ever engages on a machine where the honest
configuration cannot work at all.
"""

from __future__ import annotations

import time
from collections.abc import Iterator
from typing import Any

import pytest
import zenoh

HUB_ENDPOINT = "tcp/127.0.0.1:7448"


def _multicast_healthy() -> bool:
    """Can two fresh local sessions exchange one message?

    Returns:
        True when multicast scouting delivers between local sessions.
    """
    received: list[int] = []
    a = zenoh.open(zenoh.Config())
    b = zenoh.open(zenoh.Config())
    sub = a.declare_subscriber(
        "test/conftest/mcast_probe", lambda _sample: received.append(1)
    )
    pub = b.declare_publisher("test/conftest/mcast_probe")
    deadline = time.monotonic() + 3.0
    while not received and time.monotonic() < deadline:
        pub.put(b"probe")
        time.sleep(0.1)
    sub.undeclare()
    a.close()
    b.close()
    return bool(received)


@pytest.fixture(scope="session", autouse=True)
def zenoh_bus_fallback() -> Iterator[None]:
    """Guarantee local delivery by ALSO meshing every session via loopback.

    The hub is unconditional, not probe-gated: a flapping bus (a VPN
    connecting mid-suite) would make a conditional fallback engage on
    some runs and not others, which reads as flaky tests. The loopback
    link is ADDITIVE — multicast scouting still runs and still forms
    links when the machine is healthy, so the production path stays
    exercised; the hub merely guarantees that two test sessions can
    always reach each other. The probe only decides whether to warn.

    Yields:
        Nothing; the fixture exists for its session-wide side effect.
    """
    if _multicast_healthy():
        yield
        return

    print(
        "\n[conftest] LOCAL MULTICAST IS BROKEN (VPN capturing the route?) — "
        f"test sessions will run as CLIENTS of a loopback hub at {HUB_ENDPOINT}, "
        "which routes between them. Production discovery is NOT being "
        "exercised in this run; fix the machine (disable the VPN) to get it back.",
        flush=True,
    )
    hub_cfg = zenoh.Config()
    hub_cfg.insert_json5("listen/endpoints", f'["{HUB_ENDPOINT}"]')
    hub = zenoh.open(hub_cfg)

    original_open = zenoh.open

    def hub_client_open(config: Any | None = None, **kwargs: Any) -> zenoh.Session:  # noqa: ANN401 — mirrors zenoh.open
        """Open the session as a client of the loopback hub.

        Client mode matters: a zenoh PEER does not route third-party
        traffic, so peers joined through a common hub still cannot talk;
        a peer DOES route for its clients.

        Args:
            config: The caller's config; a default one when omitted.
            **kwargs: Passed through to the real ``zenoh.open``.

        Returns:
            The open session.
        """
        cfg = config if config is not None else zenoh.Config()
        cfg.insert_json5("mode", '"client"')
        cfg.insert_json5("connect/endpoints", f'["{HUB_ENDPOINT}"]')
        return original_open(cfg, **kwargs)

    zenoh.open = hub_client_open
    try:
        yield
    finally:
        zenoh.open = original_open
        hub.close()
