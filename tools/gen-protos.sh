#!/usr/bin/env bash
# Regenerate Python protobuf bindings + mypy stubs from protos/.
#
# protos/*.proto are the single source of truth for inter-service interfaces.
# Generated files (flight/msgs/*_pb2.py + .pyi) are COMMITTED so CI and fresh
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
OUT="$REPO/flight/msgs"

mkdir -p "$OUT"
"$VENV/bin/python" -m grpc_tools.protoc -I "$REPO/protos" \
  --plugin=protoc-gen-mypy="$VENV/bin/protoc-gen-mypy" \
  --python_out="$OUT" \
  --mypy_out="$OUT" \
  "$REPO"/protos/*.proto

# protoc emits flat sibling imports (`import hal_pb2`) that assume the output
# dir is on sys.path; rewrite them package-relative so the modules work as
# flight.msgs.* without path hacks.
sed -i -E 's/^import ([a-z0-9_]+_pb2) as/from flight.msgs import \1 as/' "$OUT"/*_pb2.py
sed -i -E 's/^import ([a-z0-9_]+_pb2)$/from flight.msgs import \1/' "$OUT"/*_pb2.pyi

echo "generated into $OUT:"
ls -la "$OUT"
