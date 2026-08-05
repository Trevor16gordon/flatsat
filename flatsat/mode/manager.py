"""The mode manager service: latched distribution of the mission state.

The bus shell around :class:`~flatsat.mode.machine.ModeStateMachine`.
Distribution is hybrid broadcast-plus-ack:

  * the current :class:`ModeState` is BROADCAST on ``sys/mode`` at every
    transition, and a queryable on the same key answers late joiners —
    a restarted app learns the mode immediately instead of waiting for
    the next transition;
  * every registered app must ack the transition sequence on
    ``sys/mode/ack`` within a timeout; a missing ack is itself a fault,
    surfaced in ModeHealth (supervisor escalation arrives with FDIR).

Boot policy: any unexpected reset boots into SAFE; only a clean prior
shutdown (marker file written by :meth:`ModeManager.shutdown_clean`)
allows Init → Nominal. The marker is consumed at startup, so a crash
always leaves the next boot marker-less — and therefore Safe.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import zenoh

from flatsat.core.bus import SamplePublisher
from flatsat.core.health import health_topic
from flatsat.mode import mode_config_pb2
from flatsat.mode.machine import BOOT_SOURCE, Decision, ModeStateMachine
from flatsat.msgs import health_pb2, mode_pb2

MODE_TOPIC = "sys/mode"


def request_topic(base: str = MODE_TOPIC) -> str:
    """Key expression mode requests arrive on.

    Args:
        base: The mode base topic.

    Returns:
        The request key.
    """
    return f"{base}/request"


def ack_topic(base: str = MODE_TOPIC) -> str:
    """Key expression transition acks arrive on.

    Args:
        base: The mode base topic.

    Returns:
        The ack key.
    """
    return f"{base}/ack"


class ModeManager:
    """Owns the latched system mode and its distribution contract."""

    def __init__(
        self,
        entry: mode_config_pb2.ModeConfig,
        session: zenoh.Session,
        base_topic: str = MODE_TOPIC,
    ) -> None:
        """Apply boot policy and declare the distribution endpoints.

        Args:
            entry: Mode composition (registered apps, timeouts, marker).
            session: Open zenoh session; not owned — caller closes it.
            base_topic: Mode key base; production default ``sys/mode``.
                Tests MUST pass a test-only base — a production key here
                would read (and command!) a live mode manager.
        """
        self.entry = entry
        self._base_topic = base_topic
        self._machine = ModeStateMachine(
            min_dwell_ns=int(entry.min_dwell_s * 1e9), start_ns=time.monotonic_ns()
        )
        self._lock = threading.Lock()
        self._acked: set[str] = set()
        self._ack_deadline_ns = 0
        self._pub = session.declare_publisher(base_topic)
        self._health = SamplePublisher(session, health_topic("mode"), "mode_manager")
        self._queryable = session.declare_queryable(base_topic, self._on_query)
        self._req_sub = session.declare_subscriber(request_topic(base_topic), self._on_request)
        self._ack_sub = session.declare_subscriber(ack_topic(base_topic), self._on_ack)
        self._stop = threading.Event()
        self._apply_boot_policy()

    # ----------------------------------------------------------- boot --

    def _apply_boot_policy(self) -> None:
        """Unexpected reset boots into SAFE; a clean marker allows NOMINAL.

        The marker is consumed either way: only a deliberate
        :meth:`shutdown_clean` re-creates it, so a crash between now and
        then leaves the NEXT boot marker-less — and therefore Safe.
        """
        marker = Path(self.entry.clean_shutdown_marker).expanduser()
        clean = marker.exists()
        if clean:
            marker.unlink()
            request = mode_pb2.ModeRequest(
                source=BOOT_SOURCE,
                requested=mode_pb2.SYSTEM_MODE_NOMINAL,
                reason="clean prior shutdown",
            )
        else:
            request = mode_pb2.ModeRequest(
                source=BOOT_SOURCE,
                requested=mode_pb2.SYSTEM_MODE_SAFE,
                reason="unexpected reset (no clean-shutdown marker)",
            )
        with self._lock:
            self._machine.decide(request, time.monotonic_ns())
            self._broadcast_locked()

    def shutdown_clean(self) -> None:
        """Record a deliberate shutdown so the next boot may go Nominal."""
        marker = Path(self.entry.clean_shutdown_marker).expanduser()
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()

    # ----------------------------------------------------- distribution --

    def _on_query(self, query: zenoh.Query) -> None:
        """Answer a late joiner with the current latched state.

        Args:
            query: Incoming query on sys/mode.
        """
        with self._lock:
            state = self._machine.state_message(transition_time_ns=time.time_ns())
        query.reply(self._base_topic, state.SerializeToString())

    def _broadcast_locked(self) -> None:
        """Publish the latched state and arm the ack window (lock held)."""
        state = self._machine.state_message(transition_time_ns=time.time_ns())
        self._pub.put(state.SerializeToString())
        self._acked = set()
        self._ack_deadline_ns = time.monotonic_ns() + int(self.entry.ack_timeout_s * 1e9)

    def _on_request(self, sample: zenoh.Sample) -> None:
        """Decide one mode request (subscriber thread).

        Args:
            sample: Incoming ModeRequest.
        """
        request = mode_pb2.ModeRequest.FromString(bytes(sample.payload.to_bytes()))
        self.request(request)

    def request(self, request: mode_pb2.ModeRequest) -> Decision:
        """Apply a mode request and broadcast on acceptance.

        Args:
            request: The requested transition.

        Returns:
            The machine's decision, for in-process callers and tests.
        """
        with self._lock:
            before = mode_pb2.SystemMode.Name(self._machine.mode)
            decision = self._machine.decide(request, time.monotonic_ns())
            if decision.accepted:
                self._broadcast_locked()
                # The transition IS the story — an operator watching the
                # journal must see it happen, not discover it by query.
                print(
                    f"[mode] {before} -> {mode_pb2.SystemMode.Name(self._machine.mode)} "
                    f"seq={self._machine.mode_seq} ({request.source}: {request.reason})",
                    flush=True,
                )
            else:
                print(
                    f"[mode] refused {mode_pb2.SystemMode.Name(request.requested)} "
                    f"from {request.source}: {decision.reason}",
                    flush=True,
                )
        return decision

    def _on_ack(self, sample: zenoh.Sample) -> None:
        """Record one app's ack (subscriber thread).

        Args:
            sample: Incoming ModeAck.
        """
        ack = mode_pb2.ModeAck.FromString(bytes(sample.payload.to_bytes()))
        with self._lock:
            if ack.mode_seq == self._machine.mode_seq:
                self._acked.add(ack.app)

    def missing_acks(self) -> list[str]:
        """Registered apps that have not acked the current transition.

        Returns:
            App names, empty until the ack window has expired (apps are
            not "missing" while they still have time).
        """
        with self._lock:
            if time.monotonic_ns() < self._ack_deadline_ns:
                return []
            return sorted(set(self.entry.apps) - self._acked)

    def state(self) -> mode_pb2.ModeState:
        """Current latched state.

        Returns:
            The ModeState, freshly stamped.
        """
        with self._lock:
            return self._machine.state_message(transition_time_ns=time.time_ns())

    # ----------------------------------------------------------- health --

    def publish_health(self) -> health_pb2.ModeHealth:
        """Publish one window of mode-manager health.

        Returns:
            The published message, for tests and introspection.
        """
        missing = self.missing_acks()
        with self._lock:
            msg = health_pb2.ModeHealth()
            msg.mode = self._machine.mode
            msg.mode_seq = self._machine.mode_seq
            msg.transitions = self._machine.transitions
            msg.rejected_requests = self._machine.rejected_requests
            msg.safe_entries = self._machine.safe_entries
            msg.missing_acks.extend(missing)
        return self._health.publish(msg)  # type: ignore[return-value]

    # -------------------------------------------------------- lifecycle --

    def run(self, health_every_s: float = 10.0) -> None:
        """Report health until :meth:`stop` is called.

        Args:
            health_every_s: Interval between ModeHealth publications.
        """
        while not self._stop.is_set():
            self._stop.wait(health_every_s)
            if not self._stop.is_set():
                self.publish_health()

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        self._queryable.undeclare()
        self._req_sub.undeclare()
        self._ack_sub.undeclare()

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines naming the latched state and the ack contract.
        """
        state = self.state()
        return [
            f"mode: {mode_pb2.SystemMode.Name(state.mode)} seq={state.mode_seq} ({state.reason})",
            f"mode: acks expected from {list(self.entry.apps)} "
            f"within {self.entry.ack_timeout_s:g} s, dwell {self.entry.min_dwell_s:g} s",
        ]
