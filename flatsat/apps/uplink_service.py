#!/usr/bin/env python3
"""Uplink service: stage what arrives, activate only what is commanded.

The flight side of the C1 deployment path. Subscribes the artifact
topics (which reach this bus over RF like any other traffic), verifies
and stages complete artifacts, and applies activation and rollback
commands through the slot manager's gates:

    manifest + chunks --> verify sha256 --> STAGED
    ACTIVATE (ground authority, not in SAFE) --> live slot, previous kept
    ROLLBACK (no authority needed, any mode) --> previous slot

Nothing uplinked is ever executed by this service; activation moves a
pointer that consumers read when they next start.

Usage:
  python -m flatsat.apps.uplink_service
  python -m flatsat.apps.uplink_service --staging-dir ~/flatsat-uplink
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

import zenoh

from flatsat.comms.slots import ActivationRefused, SlotManager
from flatsat.comms.uplink import (
    CHUNK_TOPIC,
    CONTROL_TOPIC,
    MANIFEST_TOPIC,
    UplinkReceiver,
)
from flatsat.core.bus import SamplePublisher, bus_config
from flatsat.core.health import health_topic
from flatsat.mode.client import ModeClient
from flatsat.msgs import mode_pb2, uplink_pb2

DEFAULT_UPLINK_DIR = "~/flatsat-uplink"


class UplinkService:
    """Bus-facing shell around the receiver and the slot manager."""

    def __init__(
        self,
        receiver: UplinkReceiver,
        slots: SlotManager,
        session: zenoh.Session,
        topic_prefix: str = "",
        app_name: str = "uplink",
        mode_client: ModeClient | None = None,
    ) -> None:
        """Subscribe the artifact topics.

        Args:
            receiver: Staging receiver.
            slots: Slot manager applying the authority gates.
            session: Open zenoh session; not owned — caller closes it.
            topic_prefix: Prefix on the artifact topics (tests use one;
                the ground mirror's republish prefix uses another).
            app_name: Name used for status telemetry.
            mode_client: Mode follower supplying the current mode to the
                activation gate; None means the gate sees no mode and
                only the authority check applies.
        """
        self._receiver = receiver
        self._slots = slots
        self._mode_client = mode_client
        self._status = SamplePublisher(session, health_topic(app_name), app_name)
        self._stop = threading.Event()
        prefix = f"{topic_prefix}/" if topic_prefix else ""
        self._subs = [
            session.declare_subscriber(f"{prefix}{MANIFEST_TOPIC}", self._on_manifest),
            session.declare_subscriber(f"{prefix}{CHUNK_TOPIC}", self._on_chunk),
            session.declare_subscriber(f"{prefix}{CONTROL_TOPIC}", self._on_control),
        ]
        self.last_event = ""

    def _on_manifest(self, sample: zenoh.Sample) -> None:
        """Start a transfer (subscriber thread).

        Args:
            sample: Incoming ArtifactManifest.
        """
        manifest = uplink_pb2.ArtifactManifest.FromString(bytes(sample.payload.to_bytes()))
        self._receiver.on_manifest(manifest)
        self.last_event = f"manifest {manifest.name}@{manifest.version}"

    def _on_chunk(self, sample: zenoh.Sample) -> None:
        """Absorb one chunk (subscriber thread).

        Args:
            sample: Incoming ArtifactChunk.
        """
        chunk = uplink_pb2.ArtifactChunk.FromString(bytes(sample.payload.to_bytes()))
        staged = self._receiver.on_chunk(chunk)
        if staged is not None:
            self.last_event = f"staged {staged}"
            print(f"[uplink] staged {staged}", flush=True)

    def _on_control(self, sample: zenoh.Sample) -> None:
        """Apply an activation or rollback command (subscriber thread).

        Args:
            sample: Incoming ArtifactControl.
        """
        control = uplink_pb2.ArtifactControl.FromString(bytes(sample.payload.to_bytes()))
        self.apply_control(control)

    def apply_control(self, control: uplink_pb2.ArtifactControl) -> str:
        """Apply one control command through the gates.

        Args:
            control: The activate/rollback command.

        Returns:
            A human-readable outcome, also recorded as the last event.
        """
        try:
            if control.action == uplink_pb2.ArtifactControl.ACTION_ACTIVATE:
                slots = self._slots.activate(
                    control.name,
                    control.version,
                    self._receiver.staged_path(control.name, control.version),
                    ground_authority=control.ground_authority,
                    mode=self._current_mode(),
                )
                outcome = f"activated {slots.name}@{slots.active_version}"
            elif control.action == uplink_pb2.ArtifactControl.ACTION_ROLLBACK:
                slots = self._slots.rollback(control.name)
                outcome = f"rolled back {slots.name} to @{slots.active_version}"
            else:
                outcome = "ignored control with unspecified action"
        except ActivationRefused as exc:
            outcome = f"REFUSED: {exc}"
        self.last_event = outcome
        # The operator watching the journal must see every verdict — a
        # silent refusal reads as a dead service during a live pass.
        print(f"[uplink] {outcome}", flush=True)
        return outcome

    def _current_mode(self) -> mode_pb2.SystemMode.ValueType | None:
        """The latched system mode, when a mode client is following it.

        Returns:
            The mode, or None when unknown.
        """
        if self._mode_client is None or self._mode_client.current is None:
            return None
        return self._mode_client.current.mode

    def publish_status(self) -> uplink_pb2.UplinkStatus:
        """Publish what is staged, what is live, and what was refused.

        Returns:
            The published message, for tests and introspection.
        """
        msg = uplink_pb2.UplinkStatus()
        msg.staged.extend(self._receiver.staged())
        msg.slots.extend(self._slots.all_states())
        msg.receiving = self._receiver.receiving
        msg.rejected_checksum = self._receiver.rejected_checksum
        msg.refused_activations = self._slots.refused_activations
        return self._status.publish(msg)  # type: ignore[return-value]

    def run(self, status_every_s: float = 10.0) -> None:
        """Publish status until :meth:`stop` is called.

        Receiving is subscription-driven, so this loop only reports.

        Args:
            status_every_s: Interval between UplinkStatus publications.
        """
        while not self._stop.is_set():
            self._stop.wait(status_every_s)
            if not self._stop.is_set():
                self.publish_status()

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        for sub in self._subs:
            sub.undeclare()


def main() -> int:
    """Run the uplink service from the command line.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Artifact uplink service (C1 path).")
    parser.add_argument(
        "--staging-dir",
        default=DEFAULT_UPLINK_DIR,
        help="root for staged and activated artifacts",
    )
    args = parser.parse_args()

    root = Path(args.staging_dir).expanduser()
    receiver = UplinkReceiver(root / "staging")
    slots = SlotManager(root / "slots")
    session = zenoh.open(bus_config())
    mode_client = ModeClient(session, "uplink")
    service = UplinkService(receiver, slots, session, mode_client=mode_client)
    print(f"[uplink] staging {receiver.staging_dir}", flush=True)
    print(f"[uplink] slots   {slots.slots_dir}", flush=True)
    print(
        "[uplink] activation requires ground authority and is refused in SAFE; "
        "rollback is always permitted",
        flush=True,
    )
    try:
        service.run()
    except KeyboardInterrupt:
        service.stop()
    finally:
        service.close()
        mode_client.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
