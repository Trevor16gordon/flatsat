#!/usr/bin/env python3
"""Ground tool: uplink a file, then activate or roll back — by command.

The operator side of the C1 path. Deliberately three separate verbs,
because deployment authority is earned step by step:

    send      chunk a local file onto the artifact topics (crosses the
              link like any other traffic; verified on arrival, staged,
              and NOT activated)
    activate  make a staged version live — requires --ground, and the
              flight side additionally refuses while the system is SAFE
    rollback  revert to the previous version; no authority needed,
              works in any mode

Usage:
  python -m flatsat.apps.uplink_send send ml_policy 2026-07-28a model.onnx --kind model
  python -m flatsat.apps.uplink_send activate ml_policy 2026-07-28a --ground
  python -m flatsat.apps.uplink_send rollback ml_policy
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import zenoh

from flatsat.comms.uplink import CHUNK_TOPIC, CONTROL_TOPIC, MANIFEST_TOPIC, chunk_artifact
from flatsat.core.bus import bus_config
from flatsat.msgs import uplink_pb2

_KINDS = {
    "config": uplink_pb2.ARTIFACT_KIND_CONFIG,
    "component": uplink_pb2.ARTIFACT_KIND_COMPONENT,
    "model": uplink_pb2.ARTIFACT_KIND_MODEL,
}


def send_artifact(
    session: zenoh.Session,
    name: str,
    version: str,
    payload: bytes,
    kind: int,
    chunk_bytes: int = 4096,
    pace_s: float = 0.0,
    topic_prefix: str = "",
) -> int:
    """Publish one artifact as a manifest followed by chunks.

    Args:
        session: Open zenoh session.
        name: Artifact name.
        version: Artifact version.
        payload: The complete artifact bytes.
        kind: ArtifactKind value.
        chunk_bytes: Bytes per chunk before link segmentation.
        pace_s: Delay between chunks — a real pass is not a firehose,
            and pacing keeps the link queue honest.
        topic_prefix: Prefix on the artifact topics.

    Returns:
        Number of chunks published.
    """
    prefix = f"{topic_prefix}/" if topic_prefix else ""
    manifest, chunks = chunk_artifact(name, version, payload, kind, chunk_bytes)
    session.put(f"{prefix}{MANIFEST_TOPIC}", manifest.SerializeToString())
    for chunk in chunks:
        session.put(f"{prefix}{CHUNK_TOPIC}", chunk.SerializeToString())
        if pace_s > 0:
            time.sleep(pace_s)
    return len(chunks)


def send_control(
    session: zenoh.Session,
    action: uplink_pb2.ArtifactControl.Action.ValueType,
    name: str,
    version: str,
    ground_authority: bool,
    reason: str,
    topic_prefix: str = "",
) -> None:
    """Publish one activate/rollback command.

    Args:
        session: Open zenoh session.
        action: ACTIVATE or ROLLBACK.
        name: Artifact name.
        version: Version to activate (ignored for rollback).
        ground_authority: Whether this command carries ground authority.
        reason: Recorded reason.
        topic_prefix: Prefix on the artifact topics.
    """
    prefix = f"{topic_prefix}/" if topic_prefix else ""
    control = uplink_pb2.ArtifactControl(
        action=action,
        name=name,
        version=version,
        ground_authority=ground_authority,
        reason=reason,
    )
    session.put(f"{prefix}{CONTROL_TOPIC}", control.SerializeToString())


def main() -> int:
    """Run one uplink verb from the command line.

    Returns:
        0 on success; 2 on a missing file.
    """
    parser = argparse.ArgumentParser(description="Uplink an artifact and manage its slots.")
    parser.add_argument("--topic-prefix", default="", help="prefix on the artifact topics")
    sub = parser.add_subparsers(dest="verb", required=True)

    send = sub.add_parser("send", help="chunk a file onto the artifact topics")
    send.add_argument("name")
    send.add_argument("version")
    send.add_argument("path", type=Path)
    send.add_argument("--kind", choices=sorted(_KINDS), default="model")
    send.add_argument("--chunk-bytes", type=int, default=4096)
    send.add_argument("--pace", type=float, default=0.0, help="seconds between chunks")

    activate = sub.add_parser("activate", help="make a staged version live")
    activate.add_argument("name")
    activate.add_argument("version")
    activate.add_argument("--ground", action="store_true", help="carry ground authority")
    activate.add_argument("--reason", default="ground activation")

    rollback = sub.add_parser("rollback", help="revert to the previous version")
    rollback.add_argument("name")
    rollback.add_argument("--reason", default="ground rollback")

    args = parser.parse_args()
    session = zenoh.open(bus_config())
    time.sleep(0.5)  # let scouting find the flight side before publishing
    try:
        if args.verb == "send":
            if not args.path.is_file():
                print(f"no such file: {args.path}", file=sys.stderr)
                return 2
            payload = args.path.read_bytes()
            count = send_artifact(
                session,
                args.name,
                args.version,
                payload,
                _KINDS[args.kind],
                chunk_bytes=args.chunk_bytes,
                pace_s=args.pace,
                topic_prefix=args.topic_prefix,
            )
            print(
                f"sent {args.name}@{args.version}: {len(payload)} B in {count} chunks "
                f"({args.kind}) — staged on arrival only if the digest matches; "
                f"activate separately"
            )
        elif args.verb == "activate":
            send_control(
                session,
                uplink_pb2.ArtifactControl.ACTION_ACTIVATE,
                args.name,
                args.version,
                ground_authority=args.ground,
                reason=args.reason,
                topic_prefix=args.topic_prefix,
            )
            hint = "" if args.ground else " (without --ground this will be REFUSED)"
            print(f"activation commanded for {args.name}@{args.version}{hint}")
        else:
            send_control(
                session,
                uplink_pb2.ArtifactControl.ACTION_ROLLBACK,
                args.name,
                "",
                ground_authority=False,
                reason=args.reason,
                topic_prefix=args.topic_prefix,
            )
            print(f"rollback commanded for {args.name}")
        time.sleep(0.5)  # let the last publication drain before closing
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
