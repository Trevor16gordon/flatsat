#!/usr/bin/env bash
# Regenerate Python protobuf bindings + mypy stubs from protos/.
#
# protos/*.proto are the single source of truth for inter-service interfaces.
# Generated files (flatsat/msgs/*_pb2.py + .pyi) are COMMITTED so CI and fresh
# clones need no protoc; re-run this script after any .proto change and commit
# the result. Generated files are excluded from ruff/mypy gates (pyproject +
# pre-commit hook excludes) — hand-written code importing them is still fully
# checked via the .pyi stubs.
#
# C++ codegen is deliberately deferred until a C++ consumer exists (~M2, the
# F´ bridge): same .proto files, add --cpp_out here when that day comes.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${FLATSAT_ML_VENV:-$HOME/venvs/flatsat-ml}"
OUT="$REPO/flatsat/msgs"

mkdir -p "$OUT"
"$VENV/bin/python" -m grpc_tools.protoc -I "$REPO/protos" \
  --plugin=protoc-gen-mypy="$VENV/bin/protoc-gen-mypy" \
  --python_out="$OUT" \
  --mypy_out="$OUT" \
  "$REPO"/protos/*.proto

# CONFIG schemas live COLOCATED with their owners (flatsat/**/*.proto) and
# generate colocated bindings: a proto at flatsat/hardware/devices.proto
# yields flatsat/hardware/devices_pb2.py right next to it, with correct
# package-absolute imports — no sed needed. Config files (config/*.txtpb)
# are textproto instances of these schemas; the editor resolves them via
# their `# proto-file:` headers.
CONFIG_PROTOS=$(find "$REPO/flatsat" -name '*.proto' | sort)
"$VENV/bin/python" -m grpc_tools.protoc -I "$REPO" \
  --plugin=protoc-gen-mypy="$VENV/bin/protoc-gen-mypy" \
  --python_out="$REPO" \
  --mypy_out="$REPO" \
  $CONFIG_PROTOS

# protoc emits flat sibling imports (`import hal_pb2`) that assume the output
# dir is on sys.path; rewrite them package-relative so the modules work as
# flatsat.msgs.* without path hacks.
sed -i -E 's/^import ([a-z0-9_]+_pb2) as/from flatsat.msgs import \1 as/' "$OUT"/*_pb2.py
sed -i -E 's/^import ([a-z0-9_]+_pb2)$/from flatsat.msgs import \1/' "$OUT"/*_pb2.pyi

echo "generated into $OUT:"
ls -la "$OUT"
