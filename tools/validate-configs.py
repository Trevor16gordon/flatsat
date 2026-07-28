#!/usr/bin/env python3
"""Validate every config/**/*.txtpb against its declared proto schema.

Each config file names its schema in a header comment::

    # proto-file: flatsat/vehicle.proto
    # proto-message: flatsat.v1.VehicleConfig

This tool resolves the message type from the committed bindings and
parses the file STRICTLY: an unknown or misspelled field, a missing
header, or a type mismatch fails with file:line — the same guarantee the
flight loaders give at startup, moved to commit time (pre-commit + CI),
so a bad config never lands regardless of how schema-aware the editor
was.

Usage:
  python tools/validate-configs.py            # validate config/**/*.txtpb
  python tools/validate-configs.py FILE...    # validate specific files
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from google.protobuf import symbol_database, text_format  # noqa: E402
from google.protobuf.message import Message  # noqa: E402

# Importing the binding modules registers their messages in the default
# symbol database; config schemas are reachable from these roots.
from flatsat import vehicle_pb2  # noqa: E402, F401
from flatsat.hardware import devices_pb2  # noqa: E402, F401
from flatsat.sim import mission_pb2  # noqa: E402, F401

_HEADER = re.compile(r"^#\s*proto-message:\s*(\S+)\s*$", re.MULTILINE)


def validate(path: Path) -> str | None:
    """Validate one textproto file against its declared message type.

    Args:
        path: The ``.txtpb`` file.

    Returns:
        An error description, or None when the file is valid.
    """
    text = path.read_text()
    match = _HEADER.search(text)
    if match is None:
        return f"{path}: missing '# proto-message: <full.name>' header"
    type_name = match.group(1)
    try:
        message: Message = symbol_database.Default().GetSymbol(type_name)()
    except KeyError:
        return f"{path}: unknown message type {type_name!r} (is its binding imported here?)"
    try:
        text_format.Parse(text, message)
    except text_format.ParseError as exc:
        return f"{path}:{exc.GetLine()}: {exc}"
    return None


def main() -> int:
    """Validate the given files, or every config/**/*.txtpb.

    Returns:
        0 when every file is valid; 1 otherwise.
    """
    args = [Path(arg) for arg in sys.argv[1:]]
    targets = args or sorted((REPO_ROOT / "config").rglob("*.txtpb"))
    failures = [error for target in targets if (error := validate(target)) is not None]
    for error in failures:
        print(error, file=sys.stderr)
    if not failures:
        print(f"{len(targets)} config file(s) valid against their schemas")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
