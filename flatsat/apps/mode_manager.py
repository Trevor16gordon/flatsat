#!/usr/bin/env python3
"""Mode manager app: run the mission state machine as a service.

The imperative shell around :class:`~flatsat.mode.manager.ModeManager`.
On SIGTERM/clean exit it writes the clean-shutdown marker, so the next
boot may go Init → Nominal; any crash skips that line and the next boot
lands in Safe — the boot policy is the default path, not a handler.

Usage:
  python -m flatsat.apps.mode_manager
  python -m flatsat.apps.mode_manager --vehicle other.txtpb
"""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path
from types import FrameType

import zenoh

from flatsat.core.bus import bus_config
from flatsat.core.config import load_vehicle
from flatsat.mode.manager import ModeManager


def main() -> int:
    """Run the mode manager from the command line.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="System mode manager.")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    session = zenoh.open(bus_config())
    manager = ModeManager(vehicle.mode, session)
    for line in (*vehicle.describe(), *manager.describe()):
        print(f"[mode] {line}", flush=True)

    def on_sigterm(signum: int, frame: FrameType | None) -> None:
        """Translate SIGTERM (systemd stop) into a clean stop.

        Args:
            signum: Delivered signal number.
            frame: Interrupted stack frame.
        """
        manager.stop()

    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        manager.run()
    except KeyboardInterrupt:
        manager.stop()
    finally:
        # Reaching this line IS the definition of a clean shutdown.
        manager.shutdown_clean()
        manager.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
