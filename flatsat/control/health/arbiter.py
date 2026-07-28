"""The FDIR arbiter: watch the rules, and only ever make things safer.

Structurally one-way: the only mode request this module can emit is
SAFE, without ground authority — FDIR is a client of mode authority,
never its owner, and software may only ever make the system safer on
its own. Leaving safety is a human decision (``mode_request --ground``).

Deliberately small and dependency-minimal, like the mode manager below
it: the safing path must not depend on anything that can be the reason
for safing.
"""

from __future__ import annotations

import threading
import time

import zenoh

from flatsat.control.health import fdir_config_pb2
from flatsat.control.health.rules import RuleMonitor, build_monitors
from flatsat.core.bus import SamplePublisher
from flatsat.core.health import health_topic
from flatsat.mode.client import ModeClient
from flatsat.mode.manager import MODE_TOPIC, request_topic
from flatsat.msgs import hal_pb2, health_pb2, mode_pb2

_REREQUEST_INTERVAL_NS = int(1e9)  # don't spam the manager while un-safe


class Fdir:
    """Evaluates the rulebook against live telemetry and requests Safe."""

    def __init__(
        self,
        config: fdir_config_pb2.FdirConfig,
        session: zenoh.Session,
        base_topic: str = MODE_TOPIC,
        app_name: str = "fdir",
    ) -> None:
        """Subscribe every watched topic and join the mode contract.

        Args:
            config: The declarative rulebook (vehicle file fdir block).
            session: Open zenoh session; not owned — caller closes it.
            base_topic: Mode key base; tests pass a test-only base.
            app_name: Name used for mode acks and health telemetry.
        """
        now = time.monotonic_ns()
        self.monitors: list[RuleMonitor] = build_monitors(config, now)
        self._lock = threading.Lock()
        self._request_pub = session.declare_publisher(request_topic(base_topic))
        self._health = SamplePublisher(session, health_topic(app_name), app_name)
        self._mode_client = ModeClient(session, app_name, base_topic=base_topic)
        self.safe_requests = 0
        self._last_request_ns = 0
        self._stop = threading.Event()

        by_topic: dict[str, list[RuleMonitor]] = {}
        for monitor in self.monitors:
            by_topic.setdefault(monitor.topic, []).append(monitor)
        self._subs = [
            session.declare_subscriber(topic, self._make_handler(monitors))
            for topic, monitors in by_topic.items()
        ]

    def _make_handler(self, monitors: list[RuleMonitor]) -> object:
        """Build one subscriber callback feeding a topic's monitors.

        Args:
            monitors: The monitors watching this topic.

        Returns:
            The callback.
        """

        def on_message(sample: zenoh.Sample) -> None:
            """Feed one arrival's validity flags to the topic's monitors.

            Args:
                sample: Any bus message — only its Header is read.
            """
            envelope = hal_pb2.HeaderEnvelope.FromString(bytes(sample.payload.to_bytes()))
            now = time.monotonic_ns()
            with self._lock:
                for monitor in monitors:
                    monitor.on_message(now, envelope.header.validity)

        return on_message

    def tripped_rules(self, now_ns: int | None = None) -> list[str]:
        """Names of the rules currently tripped.

        Args:
            now_ns: Current monotonic time; defaults to now.

        Returns:
            Tripped rule names, in rulebook order.
        """
        now = now_ns if now_ns is not None else time.monotonic_ns()
        with self._lock:
            return [monitor.name for monitor in self.monitors if monitor.tripped(now)]

    def tick(self) -> list[str]:
        """Evaluate the rulebook once and escalate if warranted.

        The ONLY escalation this module knows is a Safe request without
        ground authority — the toward-safety path, structurally.

        Returns:
            The rules currently tripped, for callers and tests.
        """
        now = time.monotonic_ns()
        tripped = self.tripped_rules(now)
        current = self._mode_client.current
        in_safe = current is not None and current.mode == mode_pb2.SYSTEM_MODE_SAFE
        if tripped and not in_safe and now - self._last_request_ns > _REREQUEST_INTERVAL_NS:
            request = mode_pb2.ModeRequest(
                source="fdir",
                requested=mode_pb2.SYSTEM_MODE_SAFE,
                reason=", ".join(tripped),
                ground_authority=False,
            )
            self._request_pub.put(request.SerializeToString())
            self.safe_requests += 1
            self._last_request_ns = now
        return tripped

    def publish_health(self) -> health_pb2.FdirHealth:
        """Publish one window of FDIR health.

        Returns:
            The published message, for tests and introspection.
        """
        msg = health_pb2.FdirHealth()
        msg.rules = len(self.monitors)
        msg.tripped.extend(self.tripped_rules())
        msg.safe_requests = self.safe_requests
        return self._health.publish(msg)  # type: ignore[return-value]

    def run(self, tick_hz: float = 10.0, health_every_s: float = 10.0) -> None:
        """Evaluate on cadence until :meth:`stop` is called.

        Args:
            tick_hz: Rule-evaluation rate.
            health_every_s: Interval between FdirHealth publications.
        """
        period_s = 1.0 / tick_hz
        health_ns = int(health_every_s * 1e9)
        next_health = time.monotonic_ns() + health_ns
        while not self._stop.is_set():
            self.tick()
            if time.monotonic_ns() >= next_health:
                self.publish_health()
                next_health += health_ns
            self._stop.wait(period_s)

    def stop(self) -> None:
        """Request the run loop to exit (idempotent, thread-safe)."""
        self._stop.set()

    def close(self) -> None:
        """Undeclare bus resources."""
        for sub in self._subs:
            sub.undeclare()
        self._mode_client.close()

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines naming the rulebook in force.
        """
        return [
            f"fdir: {len(self.monitors)} rule(s): "
            + ", ".join(f"{m.name}@{m.topic}" for m in self.monitors),
            "fdir: escalation = request SAFE (toward-safety only, no authority carried)",
        ]
