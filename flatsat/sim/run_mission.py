#!/usr/bin/env python3
r"""Run one mission profile from the command line — optionally in 3D.

Composes the vehicle's REAL flight chain in-process against the chosen
universe-fake and executes the mission's phases, printing the judgment.
With ``--plant basilisk`` the dynamics are real Basilisk (ground machine
only), and Vizard gives the 3D view:

  # quick, anywhere (Jetson, CI): rigid-body plant
  python -m flatsat.sim.run_mission config/missions/detumble_test.txtpb

  # the show: real dynamics, recorded 3D playback — open the .bin in Vizard
  python -m flatsat.sim.run_mission config/missions/detumble_3d.txtpb \\
      --plant basilisk --viz-save ~/detumble.bin

  # or live: START VIZARD FIRST (Direct Communication mode), then
  python -m flatsat.sim.run_mission config/missions/detumble_3d.txtpb \\
      --plant basilisk --viz
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

from flatsat.sim.scenario import ScenarioRunner, load_mission


def main() -> int:
    """Run a mission profile and report its judgment.

    Returns:
        0 when every phase passed; 1 otherwise.
    """
    parser = argparse.ArgumentParser(description="Run a mission profile.")
    parser.add_argument("mission", type=Path, help="mission file, e.g. config/missions/x.txtpb")
    parser.add_argument(
        "--plant",
        choices=("local", "basilisk"),
        default="local",
        help="universe-fake: local rigid body (anywhere) or Basilisk (ground machine)",
    )
    parser.add_argument(
        "--viz", action="store_true", help="Basilisk only: live Vizard 3D (start Vizard first)"
    )
    parser.add_argument(
        "--viz-save", default=None, help="Basilisk only: record a Vizard playback .bin"
    )
    args = parser.parse_args()

    mission = load_mission(args.mission)
    print(f"mission {mission.name}: {mission.description}", flush=True)
    print(f"vehicle {mission.vehicle_path}, plant {args.plant}", flush=True)
    with tempfile.TemporaryDirectory() as work_dir:
        runner = ScenarioRunner(
            mission,
            Path(work_dir),
            plant_kind=args.plant,
            viz_live=args.viz,
            viz_save=args.viz_save,
        )
        result = runner.run()
    print(result.describe(), flush=True)
    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
