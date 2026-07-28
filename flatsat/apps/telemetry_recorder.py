#!/usr/bin/env python3
"""Telemetry recorder app: archive the vehicle's configured bus topics.

The imperative shell around :class:`~flatsat.telemetry.recorder.Recorder`.
Recording itself is subscription-driven (no cadence to own); this app owns
lifecycle, the periodic flush that bounds the crash-loss window, and
health telemetry — which is itself recorded, because the recorder's
vitals are telemetry like everything else's.

Usage:
  python -m flatsat.apps.telemetry_recorder
  python -m flatsat.apps.telemetry_recorder --vehicle other.toml
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import zenoh

from flatsat.core.bus import SamplePublisher
from flatsat.core.config import load_vehicle
from flatsat.core.health import health_topic
from flatsat.mode.client import ModeClient
from flatsat.msgs import health_pb2
from flatsat.telemetry.recorder import Recorder


class RecorderApp:
    """Runs one Recorder with periodic flush and health reporting."""

    def __init__(self, recorder: Recorder, session: zenoh.Session) -> None:
        """Bind the recorder to its health publisher.

        Args:
            recorder: The archive writer.
            session: Open zenoh session; not owned — caller closes it.
        """
        self._recorder = recorder
        self._health = SamplePublisher(session, health_topic("recorder"), "recorder")
        self._stop = threading.Event()

    def publish_health(self) -> health_pb2.RecorderHealth:
        """Publish one window of recorder health.

        Returns:
            The published message, for tests and introspection.
        """
        records, bytes_written = self._recorder.take_window()
        files, total_bytes = self._recorder.totals()
        msg = health_pb2.RecorderHealth()
        msg.window_records = records
        msg.window_bytes = bytes_written
        msg.files = files
        msg.total_bytes = total_bytes
        msg.current_file = self._recorder.current_file()
        return self._health.publish(msg)  # type: ignore[return-value]

    def run(self, flush_every_s: float = 1.0, health_every_s: float = 30.0) -> None:
        """Flush and report until :meth:`stop` is called.

        Args:
            flush_every_s: Interval between flushes (crash-loss bound).
            health_every_s: Interval between RecorderHealth publications;
                <= 0 disables health reporting.
        """
        health_ns = int(health_every_s * 1e9) if health_every_s > 0 else None
        next_health = time.monotonic_ns() + health_ns if health_ns else None
        while not self._stop.is_set():
            self._stop.wait(flush_every_s)
            self._recorder.flush()
            now = time.monotonic_ns()
            if next_health is not None and health_ns is not None and now >= next_health:
                self.publish_health()
                next_health += health_ns

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()


def main() -> int:
    """Run the telemetry recorder from the command line.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Telemetry recorder.")
    parser.add_argument("--vehicle", type=Path, default=None, help="vehicle composition file")
    args = parser.parse_args()

    vehicle = load_vehicle(args.vehicle)
    session = zenoh.open(zenoh.Config())
    recorder = Recorder(vehicle.telemetry, session)
    for line in (*vehicle.describe(), *recorder.describe()):
        print(f"[recorder] {line}", flush=True)

    app = RecorderApp(recorder, session)
    # Join the mode contract: late-join query + ack every transition.
    mode_client = ModeClient(session, "recorder")
    try:
        app.run()
    except KeyboardInterrupt:
        app.stop()
    finally:
        mode_client.close()
        recorder.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
