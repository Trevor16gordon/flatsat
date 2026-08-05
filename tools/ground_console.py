#!/usr/bin/env python3
"""Mission-control console: the spacecraft as seen THROUGH the radio.

Subscribes the ground namespace — the only place downlinked telemetry
lands — and renders one line per meaningful change: mode transitions,
FDIR trips, uplink staging and activation state. Everything shown here
crossed the space link during a contact window; if the link is down,
this console goes quiet and stale, exactly like the real thing.

Run on the ground machine (mission control) with
FLATSAT_ZENOH_CONNECT set to the flight router:
  ~/venvs/flatsat-ground/bin/python tools/ground_console.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flatsat.core.bus import bus_config  # noqa: E402
from flatsat.msgs import health_pb2, mode_pb2, uplink_pb2  # noqa: E402


def _stamp() -> str:
    """Local wall-clock prefix for one console line."""
    return time.strftime("%H:%M:%S")


class Console:
    """Renders downlinked state changes, suppressing repeats."""

    def __init__(self) -> None:
        """Start with nothing known — the first pass paints everything."""
        self._mode: str | None = None
        self._tripped: tuple[str, ...] | None = None
        self._staged: tuple[str, ...] | None = None
        self._active: tuple[str, ...] | None = None
        self._refused: int | None = None
        self.last_rx_s = time.monotonic()

    def on_mode(self, payload: bytes) -> None:
        """Render a mode change.

        Args:
            payload: Downlinked ModeHealth bytes.
        """
        msg = health_pb2.ModeHealth.FromString(payload)
        self.last_rx_s = time.monotonic()
        mode = mode_pb2.SystemMode.Name(msg.mode).removeprefix("SYSTEM_MODE_")
        line = f"{mode} seq={msg.mode_seq}"
        if line != self._mode:
            print(f"{_stamp()}  MODE    {line}", flush=True)
            self._mode = line

    def on_fdir(self, payload: bytes) -> None:
        """Render FDIR trips appearing or clearing.

        Args:
            payload: Downlinked FdirHealth bytes.
        """
        msg = health_pb2.FdirHealth.FromString(payload)
        self.last_rx_s = time.monotonic()
        tripped = tuple(msg.tripped)
        if tripped != self._tripped:
            state = ", ".join(tripped) if tripped else "all rules quiet"
            print(f"{_stamp()}  FDIR    {state}  (safings: {msg.safe_requests})", flush=True)
            self._tripped = tripped

    def on_uplink(self, payload: bytes) -> None:
        """Render uplink staging/activation state changes.

        Args:
            payload: Downlinked UplinkStatus bytes.
        """
        msg = uplink_pb2.UplinkStatus.FromString(payload)
        self.last_rx_s = time.monotonic()
        staged = tuple(msg.staged)
        active = tuple(
            f"{slots.name}@{slots.active_version}" for slots in msg.slots if slots.active_version
        )
        if staged != self._staged:
            print(
                f"{_stamp()}  UPLINK  staged: {', '.join(staged) if staged else 'nothing'}",
                flush=True,
            )
            self._staged = staged
        if active != self._active:
            print(
                f"{_stamp()}  UPLINK  ACTIVE: {', '.join(active) if active else 'none'}",
                flush=True,
            )
            self._active = active
        if self._refused is not None and msg.refused_activations > self._refused:
            print(f"{_stamp()}  UPLINK  activation REFUSED by the flight side", flush=True)
        self._refused = msg.refused_activations


def main() -> int:
    """Run the console until interrupted.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Mission-control console (downlink view).")
    parser.add_argument("--prefix", default="ground", help="ground namespace prefix")
    parser.add_argument(
        "--stale-after-s",
        type=float,
        default=45.0,
        help="warn when nothing has crossed the link for this long",
    )
    args = parser.parse_args()

    console = Console()
    session = zenoh.open(bus_config())
    subs = [
        session.declare_subscriber(
            f"{args.prefix}/health/mode",
            lambda smp: console.on_mode(bytes(smp.payload.to_bytes())),
        ),
        session.declare_subscriber(
            f"{args.prefix}/health/fdir",
            lambda smp: console.on_fdir(bytes(smp.payload.to_bytes())),
        ),
        session.declare_subscriber(
            f"{args.prefix}/health/uplink",
            lambda smp: console.on_uplink(bytes(smp.payload.to_bytes())),
        ),
    ]
    print(
        f"[console] mission control view — everything below crossed the link "
        f"(namespace '{args.prefix}/', stale warning after {args.stale_after_s:g}s)",
        flush=True,
    )
    warned = False
    try:
        while True:
            time.sleep(1.0)
            quiet_s = time.monotonic() - console.last_rx_s
            if quiet_s > args.stale_after_s and not warned:
                print(
                    f"{_stamp()}  LINK    quiet for {quiet_s:.0f}s — no contact, or link down",
                    flush=True,
                )
                warned = True
            elif quiet_s <= args.stale_after_s:
                warned = False
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subs:
            sub.undeclare()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
