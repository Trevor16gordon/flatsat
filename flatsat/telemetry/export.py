"""Turn a recorded run into one JSON blob a viewer can load.

The archive is deliberately dumb: verbatim bytes, no parsing at record
time, so a new message type needs no recorder change. That property is
worth keeping, and it pushes the interpretation here — this module is
where opaque payloads become plottable series, a span tree, and a
timeline of events.

Decoding needs a topic-to-type map, because protobuf is not
self-describing on the wire. That map (``TOPIC_PATTERNS``) is small,
explicit, ordered, and in one place. It is the same object a ground
system would call a mission database, and generating it from the .proto
descriptors is the obvious next step; hand-maintaining a dozen entries
is not yet worth the machinery. Patterns exist because a bare last-path
element stopped being enough the day two message kinds shared a suffix
AND field numbers (MagnetometerSample vs ImuSample) — a wrong-type
decode that yields plausible numbers under wrong names is worse than no
decode at all.

Anything unmapped still yields something useful rather than nothing:
every bus message embeds ``Header`` as field 1, so an unknown payload
parses as ``HeaderEnvelope`` and contributes timing, sequence numbers
and validity flags even when its fields are unreadable.

Series are decimated on the way out. A viewer plotting 100 Hz over an
hour wants shape, not 360000 points per channel; the full fidelity stays
in the archive, which is the point of keeping the archive.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from fnmatch import fnmatchcase
from pathlib import Path
from typing import Any

from google.protobuf.message import Message

from flatsat.msgs import adcs_pb2, hal_pb2, mission_log_pb2, mode_pb2, sim_pb2
from flatsat.telemetry import mission_log
from flatsat.telemetry.recorder import read_records

DEFAULT_MAX_POINTS = 2000

# Ordered (glob pattern, message type): FIRST match wins, so specific
# patterns sit above the generic suffix they would otherwise fall into.
# Patterns match the full topic; instance names (imu0, mag1) ride the
# wildcards.
TOPIC_PATTERNS: list[tuple[str, type[Message]]] = [
    # Physics truth — both the bridge's default key and the scenario
    # vehicles' test-scoped ones. Must precede "*/state" and the truth
    # suffix must precede nothing: TruthState carries the orbit.
    ("sim/truth/state", sim_pb2.TruthState),
    ("*/sim/truth", sim_pb2.TruthState),
    # Magnetometer before the generic sample rule: MagnetometerSample
    # REUSES ImuSample's field numbers, so the fallthrough would decode
    # into convincingly wrong gyro series.
    ("*/mag*/sample", hal_pb2.MagnetometerSample),
    # ImuSample is the 100 Hz sample that matters for plots; a
    # TemperatureSample decoded as one yields no numeric leaves rather
    # than wrong ones, because their field numbers do not overlap.
    ("*/sample", hal_pb2.ImuSample),
    # Magnetorquer rod state before the generic wheel-state rule.
    ("*/mtq*/state", hal_pb2.MagnetorquerState),
    ("*/state", hal_pb2.WheelState),
    # Per-wheel applied torque (bridge feedback) vs the body command.
    ("sim/wheel/*/torque", sim_pb2.WheelAxisTorque),
    ("*/sim/wheel/*/torque", sim_pb2.WheelAxisTorque),
    ("*/torque", adcs_pb2.WheelTorqueCommand),
    ("*/wheel_torque", adcs_pb2.WheelTorqueCommand),
    # Per-rod applied dipole (plant feedback) vs the body command.
    ("sim/mtq/*/dipole", sim_pb2.MagnetorquerDipole),
    ("*/sim/mtq/*/dipole", sim_pb2.MagnetorquerDipole),
    ("*/dipole", adcs_pb2.DipoleCommand),
    ("*/span", mission_log_pb2.Span),
    ("*/annotation", mission_log_pb2.Annotation),
    ("*/mode", mode_pb2.ModeState),
]


def _message_for(topic: str) -> type[Message]:
    """Pick the message type to decode a topic with.

    Args:
        topic: Concrete bus key the record arrived on.

    Returns:
        The first pattern match, or ``HeaderEnvelope`` when unmapped —
        which still yields the common header of any bus message.
    """
    for pattern, message_type in TOPIC_PATTERNS:
        if fnmatchcase(topic, pattern):
            return message_type
    return hal_pb2.HeaderEnvelope


def flatten(message: Message, prefix: str = "") -> dict[str, float]:
    """Project a message's numeric leaves onto a dotted namespace.

    This is the step that turns nested protobuf into the flat parameter
    space every plotting tool expects — ``gyro_x_rad_s`` rather than a
    message. Booleans count as numbers; strings and bytes are skipped,
    since a viewer cannot plot them.

    Args:
        message: Decoded message.
        prefix: Dotted path accumulated so far.

    Returns:
        Mapping of dotted field path to value.
    """
    out: dict[str, float] = {}
    for field, value in message.ListFields():
        path = f"{prefix}{field.name}"
        if field.is_repeated:
            continue  # a repeated field is not one channel
        if field.type == field.TYPE_MESSAGE:
            out.update(flatten(value, f"{path}."))
        elif isinstance(value, bool):
            out[path] = float(value)
        elif isinstance(value, (int, float)) and math.isfinite(float(value)):
            out[path] = float(value)
    return out


def _decimate(times: list[int], values: list[float], limit: int) -> tuple[list[int], list[float]]:
    """Stride a series down to at most ``limit`` points.

    Args:
        times: Timestamps, nanoseconds.
        values: Matching values.
        limit: Maximum points to keep.

    Returns:
        The strided (times, values). The last sample is always kept, so
        a series never appears to end early.
    """
    if len(times) <= limit:
        return times, values
    step = len(times) // limit + 1
    kept_t, kept_v = times[::step], values[::step]
    if kept_t[-1] != times[-1]:
        kept_t.append(times[-1])
        kept_v.append(values[-1])
    return kept_t, kept_v


def export_run(archive: Path, max_points: int = DEFAULT_MAX_POINTS) -> dict[str, Any]:
    """Read one run's archive and build the viewer blob.

    Args:
        archive: Directory of ``.rec`` files, or a single file.
        max_points: Cap per series before decimation kicks in.

    Returns:
        The blob: run identity, span tree, annotations, per-topic
        statistics and decimated series.

    Raises:
        FileNotFoundError: If the archive holds no records.
    """
    files = sorted(archive.glob("*.rec")) if archive.is_dir() else [archive]
    if not files:
        raise FileNotFoundError(f"no .rec files in {archive}")

    run: dict[str, Any] = {}
    spans: dict[str, dict[str, Any]] = {}
    annotations: list[dict[str, Any]] = []
    series_t: dict[str, list[int]] = defaultdict(list)
    series_v: dict[str, list[float]] = defaultdict(list)
    counts: dict[str, int] = defaultdict(int)
    first_ns: dict[str, int] = {}
    last_ns: dict[str, int] = {}

    for path in files:
        for record in read_records(path):
            if record.topic == mission_log.SESSION_TOPIC:
                if not run:
                    run = _session_to_dict(mission_log_pb2.SessionHeader.FromString(record.payload))
                continue

            counts[record.topic] += 1
            first_ns.setdefault(record.topic, record.recv_time_ns)
            last_ns[record.topic] = record.recv_time_ns

            try:
                decoded = _message_for(record.topic).FromString(record.payload)
            except Exception:  # noqa: BLE001 — a corrupt record is data, not a crash
                continue

            if isinstance(decoded, mission_log_pb2.Span):
                _absorb_span(spans, decoded)
                continue
            if isinstance(decoded, mission_log_pb2.Annotation):
                annotations.append(
                    {
                        "time_ns": decoded.time_ns,
                        "span_id": decoded.span_id,
                        "level": decoded.level,
                        "source": decoded.source,
                        "message": decoded.message,
                    }
                )
                continue

            for name, value in flatten(decoded).items():
                if name.startswith("header."):
                    continue  # sequence numbers and timestamps are not channels
                key = f"{record.topic}.{name}"
                series_t[key].append(record.recv_time_ns)
                series_v[key].append(value)

    series: dict[str, Any] = {}
    for key, times in series_t.items():
        kept_t, kept_v = _decimate(times, series_v[key], max_points)
        series[key] = {
            "t_ns": kept_t,
            "v": kept_v,
            "count": len(times),
            "decimated": len(kept_t) != len(times),
        }

    topics = {
        topic: {
            "count": count,
            "first_ns": first_ns[topic],
            "last_ns": last_ns[topic],
            "rate_hz": _rate(count, first_ns[topic], last_ns[topic]),
        }
        for topic, count in sorted(counts.items())
    }
    return {
        "run": run,
        "files": [f.name for f in files],
        "spans": sorted(spans.values(), key=lambda s: s.get("start_ns") or 0),
        "annotations": sorted(annotations, key=lambda a: a["time_ns"]),
        "topics": topics,
        "series": series,
    }


def _rate(count: int, first: int, last: int) -> float:
    """Average publish rate of a topic.

    Args:
        count: Records seen.
        first: First arrival, nanoseconds.
        last: Last arrival, nanoseconds.

    Returns:
        Hertz, or 0.0 when the span is degenerate.
    """
    span_s = (last - first) / 1e9
    return round(count / span_s, 2) if span_s > 0 else 0.0


def _absorb_span(spans: dict[str, dict[str, Any]], span: mission_log_pb2.Span) -> None:
    """Fold one span edge into the accumulating tree.

    Start and end arrive as separate records, so a span is built from
    both — and one that never closes keeps ``end_ns`` null, which is
    exactly the evidence that a run died inside it.

    Args:
        spans: Accumulator keyed by span id.
        span: The edge just read.
    """
    entry = spans.setdefault(
        span.span_id,
        {
            "span_id": span.span_id,
            "parent_span_id": "",
            "kind": "",
            "name": "",
            "start_ns": None,
            "end_ns": None,
            "outcome": "",
            "detail": "",
            "attributes": {},
        },
    )
    if span.open:
        entry["parent_span_id"] = span.parent_span_id
        entry["kind"] = mission_log_pb2.SpanKind.Name(span.kind)
        entry["name"] = span.name
        entry["start_ns"] = span.time_ns
        entry["attributes"] = dict(span.attributes)
    else:
        entry["end_ns"] = span.time_ns
        entry["outcome"] = span.outcome
        entry["detail"] = span.detail


def _session_to_dict(header: mission_log_pb2.SessionHeader) -> dict[str, Any]:
    """Render the run identity for the blob.

    Args:
        header: The session header read from the archive.

    Returns:
        Plain-JSON run metadata.
    """
    return {
        "run_id": header.run_id,
        "source_kind": mission_log_pb2.SourceKind.Name(header.source_kind),
        "mission_name": header.mission_name,
        "vehicle_path": header.vehicle_path,
        "vehicle_sha256": header.vehicle_sha256,
        "git_sha": header.git_sha,
        "git_dirty": header.git_dirty,
        "plant": header.plant,
        "host": header.host,
        "started_wall_ns": header.started_wall_ns,
    }


def main() -> int:
    """Export a recorded run from the command line.

    Returns:
        0 on success, 1 when the archive holds nothing.
    """
    parser = argparse.ArgumentParser(description="Export a recorded run as one JSON blob.")
    parser.add_argument("archive", type=Path, help="run directory or a single .rec file")
    parser.add_argument("-o", "--output", type=Path, default=None, help="default: <archive>.json")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    args = parser.parse_args()

    try:
        blob = export_run(args.archive, args.max_points)
    except FileNotFoundError as exc:
        print(f"export failed: {exc}", file=sys.stderr)
        return 1

    out = args.output or args.archive.with_suffix(".json")
    out.write_text(json.dumps(blob, indent=1))
    run = blob["run"]
    print(f"run       {run.get('run_id', '(none)')}  [{run.get('source_kind', '?')}]")
    print(f"spans     {len(blob['spans'])}   annotations {len(blob['annotations'])}")
    print(f"topics    {len(blob['topics'])}   series {len(blob['series'])}")
    print(f"wrote     {out}  ({out.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
