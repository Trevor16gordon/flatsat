#!/usr/bin/env python3
"""FDIR service: run the vehicle's fault rulebook against the live bus.

The imperative shell around :class:`~flatsat.control.health.arbiter.Fdir`.
Watches the topics the rules name, and when a rule trips outside SAFE,
requests SAFE as a mode-manager client — the only escalation it knows.

Usage:
  python -m flatsat.apps.fdir
  python -m flatsat.apps.fdir --vehicle other.txtpb
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import zenoh

from flatsat.control.health.arbiter import Fdir
from flatsat.core.bus import bus_config
from flatsat.core.config import load_vehicle


def main() -> int:
    """Run the FDIR service from the command line.

    Returns:
        0 on clean exit; 2 when the vehicle declares no rules.
    """
    parser = argparse.ArgumentParser(description="FDIR limit checker.")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    if len(vehicle.fdir.rules) == 0:
        print("vehicle declares no fdir rules; nothing to watch", file=sys.stderr)
        return 2

    session = zenoh.open(bus_config())
    fdir = Fdir(vehicle.fdir, session)
    for line in (*vehicle.describe(), *fdir.describe()):
        print(f"[fdir] {line}", flush=True)
    try:
        fdir.run()
    except KeyboardInterrupt:
        fdir.stop()
    finally:
        fdir.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
