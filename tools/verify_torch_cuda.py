#!/usr/bin/env python3
"""N2b acceptance gate: PyTorch must actually see and use the Orin GPU.

Run with the flatsat-ml venv python (jetson-setup.sh does this automatically):

  ~/venvs/flatsat-ml/bin/python tools/verify_torch_cuda.py

Checks, in order (exit code identifies the failed stage):
  1. torch imports, and it is a CUDA build (``torch.version.cuda`` set) —
     catches the identically-versioned CPU-only wheel from pypi.org.
  2. ``torch.cuda.is_available()`` is True and the device is an Orin.
  3. A real computation runs on the GPU and matches the CPU result.
  4. numpy is <2 (the system gnuradio bindings are built against numpy 1.x).
  5. gnuradio imports in the same process as torch — the Track B requirement
     (B5 live classifier wants both in one flowgraph process).
"""

from __future__ import annotations

import sys


def eprint(*args: object) -> None:
    """Print to stderr.

    Args:
        *args: Objects to print, passed through to ``print``.
    """
    print(*args, file=sys.stderr)


def main() -> int:
    """Run the N2b PyTorch/CUDA acceptance checks in order.

    Returns:
        0 on pass; 2 torch import/CPU-build failure; 3 CUDA unavailable;
        4 GPU compute mismatch; 5 numpy >=2; 6 gnuradio import failure.
    """
    # ---- 1: torch imports and is a CUDA build ------------------------------
    try:
        import torch
    except Exception as exc:  # noqa: BLE001 — report any import failure
        eprint("FAIL [1] import torch:", exc)
        return 2
    if torch.version.cuda is None:
        eprint("FAIL [1] torch is a CPU-only build — wrong wheel (pypi.org,")
        eprint("         not the jp6/cu126 Jetson index).")
        return 2
    print(f"[ok] 1  torch {torch.__version__} (CUDA build {torch.version.cuda})")

    # ---- 2: CUDA device present --------------------------------------------
    if not torch.cuda.is_available():
        eprint("FAIL [2] torch.cuda.is_available() is False")
        return 3
    name = torch.cuda.get_device_name(0)
    cap = torch.cuda.get_device_capability(0)
    print(f"[ok] 2  cuda available: {name} (compute {cap[0]}.{cap[1]})")

    # ---- 3: real GPU compute matches CPU ------------------------------------
    a = torch.randn(256, 256)
    b = torch.randn(256, 256)
    ref = a @ b
    got = (a.cuda() @ b.cuda()).cpu()
    if not torch.allclose(ref, got, atol=1e-3):
        eprint("FAIL [3] GPU matmul result does not match CPU reference")
        return 4
    print(f"[ok] 3  GPU matmul matches CPU (max err {float((ref - got).abs().max()):.2e})")

    # ---- 4: numpy stays <2 ---------------------------------------------------
    import numpy

    major = int(numpy.__version__.split(".")[0])
    if major >= 2:
        eprint(f"FAIL [4] numpy {numpy.__version__} >= 2 — breaks system gnuradio bindings")
        return 5
    print(f"[ok] 4  numpy {numpy.__version__} (<2)")

    # ---- 5: gnuradio coexists with torch in one process ----------------------
    try:
        from gnuradio import gr

        print(f"[ok] 5  gnuradio {gr.version()} imports alongside torch")
    except Exception as exc:  # noqa: BLE001 — report any import failure
        eprint("FAIL [5] gnuradio import alongside torch:", exc)
        eprint("         (venv must be created with --system-site-packages)")
        return 6

    print("PASS: N2b gate — PyTorch is CUDA-enabled on the Orin and coexists with GNU Radio.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
