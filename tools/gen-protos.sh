#!/usr/bin/env bash
# Regenerate Python protobuf bindings + mypy stubs for every schema.
#
# Protos are the single source of truth for BOTH inter-service wire
# contracts (flatsat/msgs/*.proto) and configuration schemas (colocated
# with their owners: flatsat/vehicle.proto, flatsat/hardware/devices.proto,
# per-driver options, ...). All compile with the repo root as the import
# path, so imports read `import "flatsat/msgs/hal.proto";` and bindings
# land NEXT TO their proto with correct package-absolute imports — no
# postprocessing.
#
# Generated files are COMMITTED so CI and fresh clones need no protoc;
# re-run this script after any .proto change and commit the result (the
# CI proto-drift job fails if you forget). Generated files are excluded
# from ruff/mypy gates — hand-written code importing them is still fully
# checked via the .pyi stubs.
#
# C++ codegen is deliberately deferred until a C++ consumer exists (~M2,
# the F´ bridge): same .proto files, add --cpp_out here when that day comes.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${FLATSAT_ML_VENV:-$HOME/venvs/flatsat-ml}"

PROTOS=$(find "$REPO/flatsat" -name '*.proto' | sort)
"$VENV/bin/python" -m grpc_tools.protoc -I "$REPO" \
  --plugin=protoc-gen-mypy="$VENV/bin/protoc-gen-mypy" \
  --python_out="$REPO" \
  --mypy_out="$REPO" \
  $PROTOS

echo "generated bindings for:"
echo "$PROTOS" | sed "s|$REPO/||"
