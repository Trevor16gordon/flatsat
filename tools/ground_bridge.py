#!/usr/bin/env python3
"""Ground bridge: the downlink as a live run the mission viewer can browse.

Subscribes the ground namespace — telemetry that CROSSED THE SPACE LINK,
nothing else — and assembles a growing in-memory mission blob: mode and
FDIR state as plottable series, contact passes as spans, transitions
and uplink events as annotations. Serves it over the exact HTTP
contract the viewer's ApiDataSource was written against:

    GET /api/runs                     -> [RunSummary]
    GET /api/runs/{id}                -> MissionBlob (without bulk series)
    GET /api/runs/{id}/series?channel=... -> Series

Open the viewer with ?api=http://localhost:8600 and it browses the
live session; the blob grows as passes land.

Run on the ground machine with FLATSAT_ZENOH_CONNECT set:
  ~/venvs/flatsat-ground/bin/python tools/ground_bridge.py
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flatsat.core.bus import bus_config  # noqa: E402
from flatsat.msgs import health_pb2, mode_pb2, uplink_pb2  # noqa: E402

MAX_POINTS_PER_SERIES = 100_000  # a long demo day at these rates is far below this


class LiveRun:
    """The growing record of everything that crossed the link."""

    def __init__(self, prefix: str) -> None:
        """Start an empty run stamped now.

        Args:
            prefix: Ground namespace being watched (recorded in the id).
        """
        self.lock = threading.Lock()
        self.run_id = f"ground-live-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
        self.started_wall_ns = time.time_ns()
        self.series: dict[str, dict[str, list[float]]] = {}
        self.annotations: list[dict[str, object]] = []
        self.spans: list[dict[str, object]] = []
        self._pass_open_ns: int | None = None
        self._pass_count = 0
        self._last_rx_ns: int | None = None
        self._mode: str | None = None
        self._staged: tuple[str, ...] = ()
        self._active: tuple[str, ...] = ()
        self._refused: int | None = None
        self.prefix = prefix

    # ------------------------------------------------------------ intake --

    def _point(self, channel: str, t_ns: int, value: float) -> None:
        """Append one sample to a channel (creating it on first use).

        Args:
            channel: Dotted channel key.
            t_ns: Wall-clock nanoseconds.
            value: Sample value.
        """
        entry = self.series.setdefault(channel, {"t_ns": [], "v": []})
        if len(entry["t_ns"]) < MAX_POINTS_PER_SERIES:
            entry["t_ns"].append(t_ns)
            entry["v"].append(value)

    def _annotate(self, t_ns: int, level: str, message: str) -> None:
        """Add one timeline event.

        Args:
            t_ns: Wall-clock nanoseconds.
            level: "info" or "error".
            message: What happened.
        """
        self.annotations.append(
            {
                "time_ns": t_ns,
                "span_id": "",
                "level": level,
                "source": "downlink",
                "message": message,
            }
        )

    def _track_pass(self, t_ns: int) -> None:
        """Group arrivals into contact-pass spans.

        Downlink arrives in bursts (a pass); a gap longer than a few
        seconds closes the current pass span. This infers geometry from
        arrivals rather than trusting anyone's schedule — the ground
        can only ever know what it heard.

        Args:
            t_ns: Arrival time of the current message.
        """
        gap_ns = 8_000_000_000  # > pump cadence, < the 20 s between passes
        if self._pass_open_ns is None or (
            self._last_rx_ns is not None and t_ns - self._last_rx_ns > gap_ns
        ):
            if self._pass_open_ns is not None and self._last_rx_ns is not None:
                self.spans.append(
                    {
                        "span_id": f"pass-{self._pass_count}",
                        "parent_span_id": "",
                        "kind": "SPAN_KIND_ACTIVITY",
                        "name": f"contact pass {self._pass_count}",
                        "start_ns": self._pass_open_ns,
                        "end_ns": self._last_rx_ns,
                        "outcome": "pass",
                        "detail": "downlink received",
                        "attributes": {},
                    }
                )
            self._pass_count += 1
            self._pass_open_ns = t_ns
        self._last_rx_ns = t_ns

    def on_mode(self, payload: bytes) -> None:
        """Ingest one downlinked ModeHealth.

        Args:
            payload: Serialized message.
        """
        msg = health_pb2.ModeHealth.FromString(payload)
        now = time.time_ns()
        with self.lock:
            self._track_pass(now)
            self._point("downlink/mode.mode", now, float(msg.mode))
            self._point("downlink/mode.mode_seq", now, float(msg.mode_seq))
            self._point("downlink/mode.safe_entries", now, float(msg.safe_entries))
            name = mode_pb2.SystemMode.Name(msg.mode).removeprefix("SYSTEM_MODE_")
            if name != self._mode:
                level = "error" if name == "SAFE" else "info"
                self._annotate(now, level, f"mode: {name} (seq {msg.mode_seq})")
                self._mode = name

    def on_fdir(self, payload: bytes) -> None:
        """Ingest one downlinked FdirHealth.

        Args:
            payload: Serialized message.
        """
        msg = health_pb2.FdirHealth.FromString(payload)
        now = time.time_ns()
        with self.lock:
            self._track_pass(now)
            self._point("downlink/fdir.tripped", now, float(len(msg.tripped)))
            self._point("downlink/fdir.safe_requests", now, float(msg.safe_requests))

    def on_uplink(self, payload: bytes) -> None:
        """Ingest one downlinked UplinkStatus.

        Args:
            payload: Serialized message.
        """
        msg = uplink_pb2.UplinkStatus.FromString(payload)
        now = time.time_ns()
        with self.lock:
            self._track_pass(now)
            self._point("downlink/uplink.staged", now, float(len(msg.staged)))
            self._point("downlink/uplink.refused_activations", now, float(msg.refused_activations))
            staged = tuple(msg.staged)
            active = tuple(f"{s.name}@{s.active_version}" for s in msg.slots if s.active_version)
            if staged != self._staged:
                self._annotate(now, "info", f"staged: {', '.join(staged) or 'nothing'}")
                self._staged = staged
            if active != self._active:
                self._annotate(now, "info", f"ACTIVE: {', '.join(active) or 'none'}")
                self._active = active
            if self._refused is not None and msg.refused_activations > self._refused:
                self._annotate(now, "error", "activation REFUSED by the flight side")
            self._refused = msg.refused_activations

    # ----------------------------------------------------------- serving --

    def summary(self) -> dict[str, object]:
        """RunSummary row for the browse list."""
        return {
            "run_id": self.run_id,
            "source_kind": "SOURCE_KIND_FLIGHT",
            "mission_name": f"live downlink ({self.prefix}/)",
            "started_wall_ns": self.started_wall_ns,
        }

    def blob(self) -> dict[str, object]:
        """MissionBlob with series inline (small at downlink health rates)."""
        with self.lock:
            spans = list(self.spans)
            if self._pass_open_ns is not None:
                spans.append(
                    {
                        "span_id": f"pass-{self._pass_count}",
                        "parent_span_id": "",
                        "kind": "SPAN_KIND_ACTIVITY",
                        "name": f"contact pass {self._pass_count}",
                        "start_ns": self._pass_open_ns,
                        "end_ns": self._last_rx_ns,
                        "outcome": "",
                        "detail": "receiving",
                        "attributes": {},
                    }
                )
            topics: dict[str, dict[str, float]] = {}
            for channel, data in self.series.items():
                topic = channel.split(".")[0]
                t_ns = data["t_ns"]
                if not t_ns:
                    continue
                stats = topics.setdefault(
                    topic, {"count": 0, "first_ns": t_ns[0], "last_ns": t_ns[-1], "rate_hz": 0.1}
                )
                stats["count"] += len(t_ns)
                stats["last_ns"] = max(stats["last_ns"], t_ns[-1])
            return {
                "run": {
                    "run_id": self.run_id,
                    "source_kind": "SOURCE_KIND_FLIGHT",
                    "mission_name": f"live downlink ({self.prefix}/)",
                    "vehicle_path": "config/vehicles/flatsat_v1_mldemo_rf.txtpb",
                    "vehicle_sha256": "",
                    "git_sha": "",
                    "git_dirty": False,
                    "plant": "",
                    "host": "ground",
                    "started_wall_ns": self.started_wall_ns,
                },
                "files": [],
                "spans": spans,
                "annotations": list(self.annotations),
                "topics": topics,
                # Inline, not lazy: the viewer reads blob.series directly
                # today, and downlinked health rates are tiny. Revisit
                # when a live run carries bulk channels.
                "series": {
                    name: {
                        "t_ns": data["t_ns"],
                        "v": data["v"],
                        "count": len(data["t_ns"]),
                        "decimated": False,
                    }
                    for name, data in self.series.items()
                },
            }

    def channel(self, name: str, max_points: int | None) -> dict[str, object] | None:
        """One channel as a Series, optionally decimated.

        Args:
            name: Dotted channel key.
            max_points: Resolution hint from the viewer.

        Returns:
            The Series dict, or None for an unknown channel.
        """
        with self.lock:
            data = self.series.get(name)
            if data is None:
                return None
            t_ns, v = data["t_ns"], data["v"]
            count = len(t_ns)
            step = max(1, count // max_points) if max_points else 1
            return {
                "t_ns": t_ns[::step],
                "v": v[::step],
                "count": count,
                "decimated": step > 1,
            }


def make_handler(run: LiveRun) -> type[BaseHTTPRequestHandler]:
    """Build the request handler bound to one live run.

    Args:
        run: The run being served.

    Returns:
        Handler class for ThreadingHTTPServer.
    """

    class Handler(BaseHTTPRequestHandler):
        """Serves the ApiDataSource contract for the live run."""

        def _send(self, obj: object, status: int = 200) -> None:
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            # The viewer fetches with credentials included, which
            # browsers refuse to pair with a wildcard — echo the origin.
            origin = self.headers.get("Origin", "*")
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Credentials", "true")
            self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 - http.server API
            """Route one request."""
            url = urlparse(self.path)
            parts = [p for p in url.path.split("/") if p]
            if parts == ["api", "runs"]:
                self._send([run.summary()])
            elif parts[:2] == ["api", "runs"] and len(parts) == 3:
                self._send(run.blob())
            elif parts[:2] == ["api", "runs"] and len(parts) == 4 and parts[3] == "series":
                query = parse_qs(url.query)
                channel = query.get("channel", [""])[0]
                max_points = int(query["max_points"][0]) if "max_points" in query else None
                series = run.channel(channel, max_points)
                self._send(series if series is not None else {}, 200 if series else 404)
            else:
                self._send({"error": "not found"}, 404)

        def log_message(self, fmt: str, *args: object) -> None:
            """Silence per-request logging."""
            del fmt, args

    return Handler


def main() -> int:
    """Run the bridge until interrupted.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Live downlink -> mission viewer bridge.")
    parser.add_argument("--prefix", default="ground", help="ground namespace prefix")
    parser.add_argument("--port", type=int, default=8600)
    args = parser.parse_args()

    run = LiveRun(args.prefix)
    session = zenoh.open(bus_config())
    subs = [
        session.declare_subscriber(
            f"{args.prefix}/health/mode",
            lambda smp: run.on_mode(bytes(smp.payload.to_bytes())),
        ),
        session.declare_subscriber(
            f"{args.prefix}/health/fdir",
            lambda smp: run.on_fdir(bytes(smp.payload.to_bytes())),
        ),
        session.declare_subscriber(
            f"{args.prefix}/health/uplink",
            lambda smp: run.on_uplink(bytes(smp.payload.to_bytes())),
        ),
    ]
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(run))
    print(
        f"[bridge] serving live run {run.run_id} on http://localhost:{args.port} "
        f"— open the viewer with ?api=http://localhost:{args.port}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        for sub in subs:
            sub.undeclare()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
