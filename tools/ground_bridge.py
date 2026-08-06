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
import base64
import bisect
import json
import math
import sys
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flatsat.apps.uplink_send import send_artifact  # noqa: E402
from flatsat.comms.uplink import CONTROL_TOPIC  # noqa: E402
from flatsat.core.bus import bus_config  # noqa: E402
from flatsat.mode.manager import MODE_TOPIC, request_topic  # noqa: E402
from flatsat.msgs import hal_pb2, health_pb2, mode_pb2, uplink_pb2  # noqa: E402

MAX_POINTS_PER_SERIES = 100_000
VEHICLE_SUMMARY: dict[str, object] = {}  # a long demo day at these rates is far below this


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
        # Per-source flight-clock anchor: offset = first arrival wall
        # time minus the sample's flight-clock stamp. Samples then plot
        # at ONBOARD time — a pass delivers 30 s of history into its
        # true place on the axis instead of compressing it into the
        # window. Crude clock correlation (anchor bias = first sample's
        # link latency); the real thing is a ground-system problem this
        # repo has deliberately not solved yet.
        self._clock_anchor: dict[str, int] = {}

    # ------------------------------------------------------------ intake --

    def _point(self, channel: str, t_ns: int, value: float) -> None:
        """Append one sample to a channel (creating it on first use).

        Args:
            channel: Dotted channel key.
            t_ns: Wall-clock nanoseconds.
            value: Sample value.
        """
        entry = self.series.setdefault(channel, {"t_ns": [], "v": []})
        times = entry["t_ns"]
        if len(times) >= MAX_POINTS_PER_SERIES:
            return
        # Arrival order is not sample order on a store-and-forward link
        # (and RF paths can duplicate): insert by timestamp, drop exact
        # repeats, so a plot line never doubles back over itself.
        if not times or t_ns > times[-1]:
            times.append(t_ns)
            entry["v"].append(value)
            return
        index = bisect.bisect_left(times, t_ns)
        if index < len(times) and times[index] == t_ns:
            return
        times.insert(index, t_ns)
        entry["v"].insert(index, value)

    def _wall(self, header: object) -> int:
        """Map a flight-clock header stamp to ground wall time.

        Args:
            header: Any message Header (source + sample_time_ns).

        Returns:
            Wall-clock nanoseconds for plotting.
        """
        source = header.source or "unknown"  # type: ignore[attr-defined]
        stamp = int(header.sample_time_ns)  # type: ignore[attr-defined]
        offset = self._clock_anchor.get(source)
        if offset is None:
            offset = time.time_ns() - stamp
            self._clock_anchor[source] = offset
        return stamp + offset

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

    def on_imu(self, payload: bytes) -> None:
        """Ingest one downlinked (sampled) IMU reading.

        Args:
            payload: Serialized ImuSample.
        """
        msg = hal_pb2.ImuSample.FromString(payload)
        now = time.time_ns()
        t_ns = self._wall(msg.header)
        omega = math.sqrt(msg.gyro_x_rad_s**2 + msg.gyro_y_rad_s**2 + msg.gyro_z_rad_s**2)
        with self.lock:
            self._track_pass(now)
            self._point("downlink/imu.omega_mrad_s", t_ns, omega * 1000.0)
            self._point("downlink/imu.gyro_x_mrad_s", t_ns, msg.gyro_x_rad_s * 1000.0)
            self._point("downlink/imu.gyro_y_mrad_s", t_ns, msg.gyro_y_rad_s * 1000.0)
            self._point("downlink/imu.gyro_z_mrad_s", t_ns, msg.gyro_z_rad_s * 1000.0)

    def on_wheel(self, wheel: str, payload: bytes) -> None:
        """Ingest one downlinked (sampled) wheel state.

        Args:
            wheel: Wheel name, e.g. "wheel0".
            payload: Serialized WheelState.
        """
        msg = hal_pb2.WheelState.FromString(payload)
        now = time.time_ns()
        t_ns = self._wall(msg.header)
        with self.lock:
            self._track_pass(now)
            self._point(f"downlink/{wheel}.speed_rad_s", t_ns, msg.speed_rad_s)
            self._point(f"downlink/{wheel}.momentum_n_m_s", t_ns, msg.momentum_n_m_s)

    def on_mtq(self, rod: str, payload: bytes) -> None:
        """Ingest one downlinked (sampled) magnetorquer state.

        Args:
            rod: Rod name, e.g. "mtq_x".
            payload: Serialized MagnetorquerState.
        """
        msg = hal_pb2.MagnetorquerState.FromString(payload)
        now = time.time_ns()
        t_ns = self._wall(msg.header)
        with self.lock:
            self._track_pass(now)
            self._point(f"downlink/{rod}.dipole_a_m2", t_ns, msg.dipole_a_m2)

    def on_mag(self, payload: bytes) -> None:
        """Ingest one downlinked (sampled) magnetometer reading.

        Args:
            payload: Serialized MagnetometerSample.
        """
        msg = hal_pb2.MagnetometerSample.FromString(payload)
        now = time.time_ns()
        t_ns = self._wall(msg.header)
        field = math.sqrt(msg.mag_x_t**2 + msg.mag_y_t**2 + msg.mag_z_t**2)
        with self.lock:
            self._track_pass(now)
            self._point("downlink/mag.field_ut", t_ns, field * 1e6)
            self._point("downlink/mag.mag_x_ut", t_ns, msg.mag_x_t * 1e6)
            self._point("downlink/mag.mag_y_ut", t_ns, msg.mag_y_t * 1e6)
            self._point("downlink/mag.mag_z_ut", t_ns, msg.mag_z_t * 1e6)

    def on_star(self, payload: bytes) -> None:
        """Ingest one downlinked (sampled) star tracker attitude.

        Args:
            payload: Serialized StarTrackerSample.
        """
        msg = hal_pb2.StarTrackerSample.FromString(payload)
        now = time.time_ns()
        t_ns = self._wall(msg.header)
        with self.lock:
            self._track_pass(now)
            self._point("downlink/att.star_valid", t_ns, 1.0 if msg.star_valid else 0.0)
            if msg.star_valid:
                self._point("downlink/att.sigma_x", t_ns, msg.sigma_x)
                self._point("downlink/att.sigma_y", t_ns, msg.sigma_y)
                self._point("downlink/att.sigma_z", t_ns, msg.sigma_z)

    def audit(self, message: str, level: str = "info") -> None:
        """Record one issued command in the console's own trail.

        Args:
            message: What mission control did.
            level: "info" or "error".
        """
        now = time.time_ns()
        with self.lock:
            self.annotations.append(
                {
                    "time_ns": now,
                    "span_id": "",
                    "level": level,
                    "source": "mission-control",
                    "message": message,
                }
            )

    def reset(self) -> None:
        """Start a fresh live run: console housekeeping, ground-only.

        Clears nothing on the spacecraft — only this console's record
        of what it has heard. The next pass repaints current state.
        """
        with self.lock:
            self.run_id = f"ground-live-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
            self.started_wall_ns = time.time_ns()
            self.series = {}
            self.annotations = []
            self.spans = []
            self._pass_open_ns = None
            self._pass_count = 0
            self._last_rx_ns = None
            self._mode = None
            self._staged = ()
            self._active = ()
            self._refused = None
            self._clock_anchor = {}

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
                    "vehicle_path": str(VEHICLE_SUMMARY.get("vehicle_path", "")),
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


class CommandStation:
    """Publishes mission-control commands into the ground namespace.

    Everything here rides the same path as the CLI tools: ground
    namespace -> ground link service -> contact window -> RF -> flight
    bus. The browser never talks to the spacecraft; it talks to the
    ground station, which waits for a pass like everybody else.
    """

    _KINDS = {
        "config": uplink_pb2.ARTIFACT_KIND_CONFIG,
        "component": uplink_pb2.ARTIFACT_KIND_COMPONENT,
        "model": uplink_pb2.ARTIFACT_KIND_MODEL,
    }

    def __init__(self, session: zenoh.Session, prefix: str) -> None:
        """Bind to the bus session and ground namespace.

        Args:
            session: Open zenoh session (shared with the subscribers).
            prefix: Ground namespace prefix.
        """
        self._session = session
        self._prefix = prefix

    def mode(self, mode_name: str, reason: str, ground: bool) -> str:
        """Queue one mode request for the next pass.

        Args:
            mode_name: Bare mode name (NOMINAL, SAFE, RECOVERY, INIT).
            reason: Operator-entered reason, recorded in the transition.
            ground: Whether the request carries ground authority.

        Returns:
            Human-readable confirmation.

        Raises:
            ValueError: On an unknown mode name.
        """
        requested = mode_pb2.SystemMode.Value(f"SYSTEM_MODE_{mode_name.upper()}")
        request = mode_pb2.ModeRequest(
            source="mission-control",
            requested=requested,
            reason=reason or "mission control",
            ground_authority=ground,
        )
        topic = f"{self._prefix}/{request_topic(MODE_TOPIC)}"
        self._session.put(topic, request.SerializeToString())
        return f"mode request {mode_name.upper()} queued for next pass"

    def artifact(self, action: str, name: str, version: str, ground: bool) -> str:
        """Queue an activate or rollback command.

        Args:
            action: "activate" or "rollback".
            name: Artifact slot name.
            version: Version to activate (ignored for rollback).
            ground: Ground authority flag (activation requires it).

        Returns:
            Human-readable confirmation.

        Raises:
            ValueError: On an unknown action.
        """
        actions = {
            "activate": uplink_pb2.ArtifactControl.ACTION_ACTIVATE,
            "rollback": uplink_pb2.ArtifactControl.ACTION_ROLLBACK,
        }
        if action not in actions:
            raise ValueError(f"unknown action {action!r}")
        control = uplink_pb2.ArtifactControl(
            action=actions[action],
            name=name,
            version=version,
            ground_authority=ground,
        )
        self._session.put(f"{self._prefix}/{CONTROL_TOPIC}", control.SerializeToString())
        target = f"{name}@{version}" if action == "activate" else name
        return f"{action} {target} queued for next pass"

    def upload(self, name: str, version: str, kind: str, payload: bytes) -> str:
        """Chunk and queue one artifact for uplink.

        Args:
            name: Artifact name.
            version: Version tag.
            kind: "model", "config" or "component".
            payload: The artifact bytes.

        Returns:
            Human-readable confirmation.

        Raises:
            ValueError: On an unknown kind.
        """
        if kind not in self._KINDS:
            raise ValueError(f"unknown kind {kind!r}")
        chunks = send_artifact(
            self._session,
            name,
            version,
            payload,
            self._KINDS[kind],
            topic_prefix=self._prefix,
        )
        return f"{name}@{version}: {len(payload)} B in {chunks} chunk(s) queued for next pass"


def vehicle_summary(path: str) -> dict[str, object]:
    """Describe the flying vehicle: composition, geometry, physics.

    Args:
        path: Vehicle file the flight stack is running.

    Returns:
        JSON-ready summary for the viewer's vehicle panel.
    """
    from flatsat.core.config import load_vehicle, which_impl
    from flatsat.sim.orbit_config import load_orbit  # noqa: F401  (may vary)

    spec = load_vehicle(path)
    cfg = spec.config

    def realness(kind: str) -> str:
        """Classify a driver implementation name.

        Args:
            kind: Registry key, e.g. "basilisk_imu".

        Returns:
            "simulated" or "real".
        """
        return "simulated" if kind.startswith(("basilisk_", "sim_")) else "real"

    sensors = []
    for s in cfg.sensors:
        kind = s.WhichOneof("options") or "?"
        sensors.append(
            {
                "name": s.name,
                "kind": kind,
                "realness": realness(kind),
                "rate_hz": s.rate_hz,
            }
        )
    actuators = []
    for a in cfg.actuators:
        kind = which_impl(a, "options", a.name)
        actuators.append(
            {
                "name": a.name,
                "kind": kind,
                "realness": realness(kind),
                "position_m": list(a.mounting.position_m),
                "axis": list(a.mounting.axis),
            }
        )
    control = cfg.control
    orbit: dict[str, float] = {}
    strategy = control.WhichOneof("strategy")
    if strategy == "nadir_point" and control.nadir_point.HasField("orbit"):
        from flatsat.core.config import load_textproto
        from flatsat.sim import mission_pb2

        msg = mission_pb2.OrbitConfig()
        load_textproto(control.nadir_point.orbit, msg)
        orbit = {
            "altitude_m": msg.altitude_m,
            "inclination_deg": msg.inclination_deg,
            "raan_deg": msg.raan_deg,
        }
    return {
        "name": cfg.name,
        "vehicle_path": path,
        "mass_kg": cfg.body.mass_kg,
        "inertia_kg_m2": list(cfg.body.inertia_kg_m2),
        "strategy": strategy,
        "orbit": orbit,
        "estimator": control.WhichOneof("estimator"),
        "rate_hz": control.rate_hz,
        "sensors": sensors,
        "actuators": actuators,
        # The platform facts the config cannot know it sits on.
        "platform": [
            {
                "name": "flight computer",
                "kind": "NVIDIA Jetson Orin Nano (JetPack 6.2.1)",
                "realness": "real",
            },
            {"name": "flight radio", "kind": "ADALM-Pluto SDR (915 MHz GMSK)", "realness": "real"},
            {"name": "ground radio", "kind": "ADALM-Pluto SDR (pluto-ground)", "realness": "real"},
            {
                "name": "space link",
                "kind": "CCSDS framing, 10 s pass / 30 s, 30 dB pads",
                "realness": "real",
            },
            {
                "name": "orbit + physics",
                "kind": "Basilisk (Mac): rigid body, orbit, field, sun",
                "realness": "simulated",
            },
        ],
    }


def make_handler(run: LiveRun, station: CommandStation) -> type[BaseHTTPRequestHandler]:
    """Build the request handler bound to one live run.

    Args:
        run: The run being served.
        station: Command publisher for the POST routes.

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
            if parts == ["api", "capabilities"]:
                self._send({"canBrowseRuns": True, "canCommand": True})
            elif parts == ["api", "vehicle"]:
                self._send(VEHICLE_SUMMARY)
            elif parts == ["api", "runs"]:
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

        def do_OPTIONS(self) -> None:  # noqa: N802 - http.server API
            """Answer the CORS preflight for POST routes."""
            origin = self.headers.get("Origin", "*")
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Credentials", "true")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Vary", "Origin")
            self.end_headers()

        def do_POST(self) -> None:  # noqa: N802 - http.server API
            """Route one command."""
            url = urlparse(self.path)
            parts = [p for p in url.path.split("/") if p]
            length = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._send({"ok": False, "detail": "malformed JSON"}, 400)
                return
            try:
                if parts == ["api", "command", "mode"]:
                    detail = station.mode(
                        str(body.get("mode", "")),
                        str(body.get("reason", "")),
                        bool(body.get("ground", True)),
                    )
                elif parts == ["api", "command", "artifact"]:
                    detail = station.artifact(
                        str(body.get("action", "")),
                        str(body.get("name", "")),
                        str(body.get("version", "")),
                        bool(body.get("ground", True)),
                    )
                elif parts == ["api", "command", "upload"]:
                    payload = base64.b64decode(str(body.get("content_b64", "")))
                    detail = station.upload(
                        str(body.get("name", "")),
                        str(body.get("version", "")),
                        str(body.get("kind", "model")),
                        payload,
                    )
                elif parts == ["api", "console", "clear"]:
                    run.reset()
                    detail = "console view cleared (ground-side only)"
                else:
                    self._send({"ok": False, "detail": "not found"}, 404)
                    return
            except (ValueError, KeyError) as exc:
                self._send({"ok": False, "detail": str(exc)}, 400)
                return
            run.audit(f"CMD: {detail}")
            print(f"[bridge] CMD: {detail}", flush=True)
            self._send({"ok": True, "detail": detail})

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
    parser.add_argument(
        "--flight-vehicle",
        default="config/vehicles/flatsat_v1.txtpb",
        help="vehicle file the FLIGHT stack is running (for the vehicle panel)",
    )
    args = parser.parse_args()
    global VEHICLE_SUMMARY
    VEHICLE_SUMMARY = vehicle_summary(args.flight_vehicle)

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
        session.declare_subscriber(
            f"{args.prefix}/hal/imu0/sample",
            lambda smp: run.on_imu(bytes(smp.payload.to_bytes())),
        ),
    ]

    def wheel_handler(wheel: str) -> Callable[[zenoh.Sample], None]:
        """Bind one wheel's name into its subscriber callback.

        Args:
            wheel: Wheel name.

        Returns:
            The callback.
        """

        def on_message(smp: zenoh.Sample) -> None:
            """Ingest one wheel state sample.

            Args:
                smp: The bus message.
            """
            run.on_wheel(wheel, bytes(smp.payload.to_bytes()))

        return on_message

    for wheel in ("wheel0", "wheel1", "wheel2"):
        subs.append(
            session.declare_subscriber(f"{args.prefix}/hal/{wheel}/state", wheel_handler(wheel))
        )

    def mtq_handler(rod: str) -> Callable[[zenoh.Sample], None]:
        """Bind one rod's name into its subscriber callback.

        Args:
            rod: Rod name.

        Returns:
            The callback.
        """

        def on_message(smp: zenoh.Sample) -> None:
            """Ingest one magnetorquer state sample.

            Args:
                smp: The bus message.
            """
            run.on_mtq(rod, bytes(smp.payload.to_bytes()))

        return on_message

    for rod in ("mtq_x", "mtq_y", "mtq_z"):
        subs.append(session.declare_subscriber(f"{args.prefix}/hal/{rod}/state", mtq_handler(rod)))
    subs.append(
        session.declare_subscriber(
            f"{args.prefix}/hal/mag0/sample",
            lambda smp: run.on_mag(bytes(smp.payload.to_bytes())),
        )
    )
    subs.append(
        session.declare_subscriber(
            f"{args.prefix}/hal/st0/sample",
            lambda smp: run.on_star(bytes(smp.payload.to_bytes())),
        )
    )
    station = CommandStation(session, args.prefix)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(run, station))
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
