"""Mode client: how any app participates in the mode contract.

Late-join query (learn the current mode NOW, not at the next
transition), broadcast subscription, and the ack every registered app
owes the manager. Apps pass a callback for their local mode response —
the per-app mode table (app × mode → config) arrives with the parameter
database; until then most apps just ack for liveness visibility.
"""

from __future__ import annotations

from collections.abc import Callable

import zenoh

from flatsat.mode.manager import MODE_TOPIC, ack_topic
from flatsat.msgs import mode_pb2


class ModeClient:
    """Follows sys/mode for one app: query at start, ack every change."""

    def __init__(
        self,
        session: zenoh.Session,
        app: str,
        on_mode: Callable[[mode_pb2.ModeState], None] | None = None,
        query_timeout_s: float = 2.0,
        base_topic: str = MODE_TOPIC,
    ) -> None:
        """Join the mode contract.

        Args:
            session: Open zenoh session; not owned — caller closes it.
            app: This app's name, as registered with the manager.
            on_mode: Called with every ModeState (including the initial
                late-join answer). Runs on the subscriber thread — keep
                it short.
            query_timeout_s: How long to wait for the late-join answer;
                no manager running is not an error (the daemon works
                mode-less until one appears).
            base_topic: Mode key base; production default ``sys/mode``.
                Tests MUST pass a test-only base.
        """
        self._app = app
        self._on_mode = on_mode
        self._ack_pub = session.declare_publisher(ack_topic(base_topic))
        self.current: mode_pb2.ModeState | None = None
        self._sub = session.declare_subscriber(base_topic, self._on_state)
        # Late join: ask for the current latched state instead of waiting
        # for the next transition.
        for reply in session.get(base_topic, timeout=query_timeout_s):
            if reply.ok is not None:
                self._handle(mode_pb2.ModeState.FromString(bytes(reply.ok.payload.to_bytes())))

    def _on_state(self, sample: zenoh.Sample) -> None:
        """Handle a broadcast transition (subscriber thread).

        Args:
            sample: Incoming ModeState.
        """
        self._handle(mode_pb2.ModeState.FromString(bytes(sample.payload.to_bytes())))

    def _handle(self, state: mode_pb2.ModeState) -> None:
        """Latch, ack, and forward one mode state.

        Args:
            state: The received mode state.
        """
        self.current = state
        ack = mode_pb2.ModeAck(app=self._app, mode_seq=state.mode_seq)
        self._ack_pub.put(ack.SerializeToString())
        if self._on_mode is not None:
            self._on_mode(state)

    def close(self) -> None:
        """Undeclare bus resources."""
        self._sub.undeclare()
