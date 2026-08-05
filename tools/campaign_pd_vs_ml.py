#!/usr/bin/env python3
"""PD-vs-ML detumble campaign, exported as a mission-viewer blob.

Flies the classical PD law and the trained ml_policy artifact through
identical held-out initial conditions in the fastloop (same registry
composition, device models, and physics as flight), then writes a
single viewer-loadable JSON:

  * one time WINDOW per initial condition — the PD and ML traces of a
    condition share the window, so dropping both channels on one plot
    overlays them with no viewer changes;
  * a span tree in the mission lane — one phase per initial condition,
    one activity per controller, outcome pass/fail against the detumble
    floor — so the campaign reads like any recorded mission.

Usage:
  python tools/campaign_pd_vs_ml.py --artifact build/ml_detumble/2026-08-05a.json \
      [--out ~/hil-trace/campaign_pd_vs_ml.json]
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import train_ml_detumble as training  # noqa: E402

from flatsat.comms.slots import SlotManager  # noqa: E402
from flatsat.sim.fastloop import FastLoopResult, run_fast_loop  # noqa: E402

MAGNITUDES_MRAD_S = (30.0, 50.0, 70.0, 100.0, 150.0, 180.0)
WINDOW_S = training.GATE_DURATION_S
FLOOR_MRAD_S = training.GATE_FLOOR_RAD_S * 1000
DECIMATE = 5  # keep every 5th sample: dt 0.02 s -> 10 Hz in the blob


def _series(
    blob: dict[str, Any],
    topic: str,
    field_name: str,
    t0_ns: int,
    result_times: list[float],
    values: list[float],
) -> None:
    """Add one decimated series and keep its topic block consistent.

    Args:
        blob: The viewer blob under construction.
        topic: Topic name, e.g. "sim/ic050mrad/ml".
        field_name: Field within the topic.
        t0_ns: Wall-clock origin of this window.
        result_times: Sim times, seconds.
        values: Sample values, same length.
    """
    t_ns = [t0_ns + int(t * 1e9) for t in result_times[::DECIMATE]]
    v = values[::DECIMATE]
    blob["series"][f"{topic}.{field_name}"] = {
        "t_ns": t_ns,
        "v": v,
        "count": len(v),
        "decimated": True,
    }
    topic_entry = blob["topics"].setdefault(
        topic,
        {"count": 0, "first_ns": t_ns[0], "last_ns": t_ns[-1], "rate_hz": 1.0 / (0.02 * DECIMATE)},
    )
    topic_entry["count"] += len(v)
    topic_entry["last_ns"] = max(topic_entry["last_ns"], t_ns[-1])


def _span(
    blob: dict[str, Any],
    span_id: str,
    parent: str,
    kind: str,
    name: str,
    start_ns: int,
    end_ns: int,
    outcome: str = "",
    detail: str = "",
) -> None:
    """Append one closed span to the blob.

    Args:
        blob: The viewer blob under construction.
        span_id: Unique id within the blob.
        parent: Parent span id, empty for the root.
        kind: SpanKind name string.
        name: Display name.
        start_ns: Open edge, wall clock.
        end_ns: Close edge, wall clock.
        outcome: "pass" / "fail" / "" for structural spans.
        detail: Judge's one-liner.
    """
    blob["spans"].append(
        {
            "span_id": span_id,
            "parent_span_id": parent,
            "kind": kind,
            "name": name,
            "start_ns": start_ns,
            "end_ns": end_ns,
            "outcome": outcome,
            "detail": detail,
            "attributes": {},
        }
    )


def main() -> int:
    """Run the campaign and write the blob.

    Returns:
        0 on success.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument(
        "--out", type=Path, default=Path("~/hil-trace/campaign_pd_vs_ml.json").expanduser()
    )
    parser.add_argument("--seed", type=int, default=20260805)
    args = parser.parse_args()

    slots = Path(tempfile.mkdtemp(prefix="campaign-slots-"))
    artifact_bytes = args.artifact.read_bytes()
    version = args.artifact.stem
    staged = slots / "staged.json"
    staged.write_bytes(artifact_bytes)
    SlotManager(slots).activate("ml_detumble", version, staged, ground_authority=True)

    vehicles = {
        "pd": training._detumble_vehicle("rate_damping"),
        "ml": training._detumble_vehicle("ml_policy", slots_dir=str(slots)),
    }
    git_sha = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=False
    ).stdout.strip()

    t_start_ns = time.time_ns()
    blob: dict[str, Any] = {
        "run": {
            "run_id": f"campaign-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}",
            "source_kind": "SOURCE_KIND_SIM",
            "mission_name": f"pd-vs-ml-detumble ({version})",
            "vehicle_path": "config/vehicles/flatsat_v1.txtpb",
            "vehicle_sha256": "",
            "git_sha": git_sha,
            "git_dirty": False,
            "plant": "fastloop",
            "host": "ground",
            "started_wall_ns": t_start_ns,
        },
        "files": [],
        "spans": [],
        "annotations": [],
        "topics": {},
        "series": {},
    }

    rng = random.Random(args.seed + 1000)  # same stream as the training gate
    windows_end = t_start_ns
    for index, magnitude in enumerate(MAGNITUDES_MRAD_S):
        omega0 = training._random_omega(rng, magnitude / 1000.0)
        window_ns = t_start_ns + int(index * (WINDOW_S + 10.0) * 1e9)
        finals: dict[str, float] = {}
        results: dict[str, FastLoopResult] = {}
        for label, vehicle in vehicles.items():
            result = run_fast_loop(vehicle, WINDOW_S, omega0, dt_s=0.02, seed=args.seed)
            results[label] = result
            finals[label] = result.final_omega_mag_rad_s * 1000
            topic = f"sim/ic{int(magnitude):03d}mrad/{label}"
            _series(
                blob,
                topic,
                "omega_mrad_s",
                window_ns,
                result.times_s,
                [w * 1000 for w in result.omega_mag_rad_s],
            )
            _series(
                blob,
                topic,
                "wheel_momentum_n_m_s",
                window_ns,
                result.times_s,
                result.wheel_momentum_n_m_s,
            )
        window_close_ns = window_ns + int(WINDOW_S * 1e9)
        windows_end = window_close_ns
        phase_id = f"ic{int(magnitude):03d}"
        _span(
            blob,
            phase_id,
            "campaign",
            "SPAN_KIND_PHASE",
            f"release {magnitude:.0f} mrad/s",
            window_ns,
            window_close_ns,
        )
        for label in vehicles:
            final = finals[label]
            _span(
                blob,
                f"{phase_id}-{label}",
                phase_id,
                "SPAN_KIND_ACTIVITY",
                f"{label}-detumble",
                window_ns,
                window_close_ns,
                outcome="pass" if final < FLOOR_MRAD_S else "fail",
                detail=f"final {final:.2f} mrad/s (floor {FLOOR_MRAD_S:.0f})",
            )
        print(
            f"  ic {magnitude:6.1f} mrad/s  PD {finals['pd']:6.2f}  ML {finals['ml']:6.2f}",
            flush=True,
        )

    _span(
        blob,
        "campaign",
        "",
        "SPAN_KIND_MISSION",
        f"pd-vs-ml-detumble ({version})",
        t_start_ns,
        windows_end,
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(blob))
    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
