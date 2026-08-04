#!/usr/bin/env python3
"""Ground commanding: request a system mode transition from the CLI.

The operator's side of the §7 asymmetry. Toward-safety requests need no
authority; away-from-safety requests must carry ``--ground`` — the same
rule the mode manager enforces, surfaced honestly here.

Usage:
  python -m flatsat.apps.mode_request SAFE --reason "operator safing"
  python -m flatsat.apps.mode_request RECOVERY --ground --reason "checkout"
  python -m flatsat.apps.mode_request NOMINAL --ground --reason "checkout complete"
  python -m flatsat.apps.mode_request --status          # just report the mode
"""

from __future__ import annotations

import argparse
import sys
import time

import zenoh

from flatsat.core.bus import bus_config
from flatsat.mode.manager import MODE_TOPIC, request_topic
from flatsat.msgs import mode_pb2


def current_state(
    session: zenoh.Session, base_topic: str, timeout_s: float = 3.0
) -> mode_pb2.ModeState | None:
    """Query the mode manager's latched state.

    Args:
        session: Open zenoh session.
        base_topic: Mode key base.
        timeout_s: Query timeout.

    Returns:
        The latched state, or None when no manager answered.
    """
    replies = session.get(base_topic, timeout=timeout_s)
    for reply in replies:
        if reply.ok is not None:
            state: mode_pb2.ModeState = mode_pb2.ModeState.FromString(
                bytes(reply.ok.payload.to_bytes())
            )
            return state  # one latched state is the answer — do not drain
    return None


def describe(state: mode_pb2.ModeState) -> str:
    """Render one latched mode state for the operator.

    Args:
        state: The state to render.

    Returns:
        A one-line summary.
    """
    name = mode_pb2.SystemMode.Name(state.mode).removeprefix("SYSTEM_MODE_")
    return f"{name} seq={state.mode_seq} ({state.reason})"


def send_request(
    session: zenoh.Session,
    base_topic: str,
    mode_name: str,
    reason: str,
    ground: bool,
    settle_s: float = 1.0,
) -> tuple[mode_pb2.ModeState | None, mode_pb2.ModeState | None]:
    """Publish one mode request and observe the outcome.

    Args:
        session: Open zenoh session.
        base_topic: Mode key base (production ``sys/mode``; tests pass a
            test-only base).
        mode_name: Bare target mode name (``SAFE``, ``NOMINAL``, ...).
        reason: Human-readable reason, recorded in the transition.
        ground: Whether this request carries ground authority.
        settle_s: How long to wait before reading back the result.

    Returns:
        Tuple of (state before, state after) — after is what the manager
        actually decided; an unchanged sequence means the request was
        refused.

    Raises:
        ValueError: If the mode name does not exist.
    """
    try:
        requested = mode_pb2.SystemMode.Value(f"SYSTEM_MODE_{mode_name}")
    except ValueError as exc:
        raise ValueError(f"unknown mode {mode_name!r}") from exc

    before = current_state(session, base_topic)
    request = mode_pb2.ModeRequest(
        source="ground",
        requested=requested,
        reason=reason,
        ground_authority=ground,
    )
    session.declare_publisher(request_topic(base_topic)).put(request.SerializeToString())
    time.sleep(settle_s)
    after = current_state(session, base_topic)
    return before, after


def main() -> int:
    """Send one mode request (or report the mode) from the command line.

    Returns:
        0 when the transition happened (or --status found a manager);
        1 when it was refused; 2 when no mode manager answered.
    """
    parser = argparse.ArgumentParser(description="Request a system mode transition.")
    parser.add_argument(
        "mode",
        nargs="?",
        default=None,
        help="target mode: INIT, NOMINAL, SAFE, RECOVERY (omit with --status)",
    )
    parser.add_argument("--reason", default="operator command", help="recorded reason")
    parser.add_argument(
        "--ground",
        action="store_true",
        help="carry ground authority (required to leave safety: SAFE->RECOVERY->NOMINAL)",
    )
    parser.add_argument("--status", action="store_true", help="only report the current mode")
    args = parser.parse_args()

    session = zenoh.open(bus_config())
    time.sleep(0.5)  # let scouting find the manager before querying
    try:
        if args.status or args.mode is None:
            state = current_state(session, MODE_TOPIC)
            if state is None:
                print("no mode manager answered", file=sys.stderr)
                return 2
            print(f"mode: {describe(state)}")
            return 0

        before, after = send_request(
            session, MODE_TOPIC, args.mode.upper(), args.reason, args.ground
        )
        if before is None or after is None:
            print("no mode manager answered", file=sys.stderr)
            return 2
        print(f"before: {describe(before)}")
        print(f"after:  {describe(after)}")
        if after.mode_seq == before.mode_seq:
            hint = "" if args.ground else " (leaving safety needs --ground)"
            print(f"request REFUSED{hint}", file=sys.stderr)
            return 1
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    sys.exit(main())
