"""Publish mission structure onto the bus, so the archive explains itself.

Spans and annotations are ordinary bus messages. That is the whole
design: the recorder already captures whatever is published, so mission
structure needs no new storage path, no sidecar, and no index. A run
recorded this way can be sliced by sub-mission from the archive alone,
including by a reader that has never seen this module.

Identity is per-run and hierarchical. A span id is unique within a run;
the run id is the join key across files and across sim/HIL/flight
recordings of the same profile.

Nothing here blocks or raises. A logging failure must never take down a
mission — losing the annotation is bad, losing the spacecraft is worse.
"""

from __future__ import annotations

import contextlib
import os
import socket
import subprocess
import threading
import time
from collections.abc import Iterator
from pathlib import Path

import zenoh

from flatsat.msgs import mission_log_pb2

SPAN_TOPIC = "mission/span"
ANNOTATION_TOPIC = "mission/annotation"
SESSION_TOPIC = "_meta/session"  # reserved: written into the file, never published


def git_revision(repo: Path | None = None) -> tuple[str, bool]:
    """Identify the code that produced a run.

    Args:
        repo: Repository root; None uses this file's repository.

    Returns:
        Tuple of (short sha, dirty). Empty sha when git is unavailable —
        a run outside version control is still worth recording, it is
        just not reproducible, and ``dirty`` says so.
    """
    root = repo or Path(__file__).resolve().parents[2]
    try:
        sha = subprocess.run(  # noqa: S603
            ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],  # noqa: S607
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        ).stdout.strip()
        status = subprocess.run(  # noqa: S603
            ["git", "-C", str(root), "status", "--porcelain"],  # noqa: S607
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "", True
    return sha, bool(status)


def build_session_header(
    run_id: str,
    source_kind: mission_log_pb2.SourceKind.ValueType,
    mission_name: str = "",
    vehicle_path: str = "",
    vehicle_sha256: str = "",
    plant: str = "",
) -> mission_log_pb2.SessionHeader:
    """Describe a run completely enough to interpret its data later.

    Args:
        run_id: Unique id for this execution.
        source_kind: Sim, HIL, flight or replay — the field that keeps
            two otherwise identical recordings apart.
        mission_name: Mission profile name, when the run flies one.
        vehicle_path: Vehicle composition file.
        vehicle_sha256: Digest of that file AS FLOWN, because the file
            on disk today is not necessarily the one that ran.
        plant: Which universe-fake supplied the dynamics.

    Returns:
        The populated header.
    """
    sha, dirty = git_revision()
    header = mission_log_pb2.SessionHeader(
        run_id=run_id,
        source_kind=source_kind,
        mission_name=mission_name,
        vehicle_path=vehicle_path,
        vehicle_sha256=vehicle_sha256,
        git_sha=sha,
        git_dirty=dirty,
        plant=plant,
        host=socket.gethostname(),
        started_wall_ns=time.time_ns(),
        started_mono_ns=time.monotonic_ns(),
    )
    header.header.source = "mission_log"
    header.header.publish_time_ns = header.started_wall_ns
    return header


class MissionLogger:
    """Emits spans and annotations for one run."""

    def __init__(self, session: zenoh.Session, run_id: str, source: str = "runner") -> None:
        """Bind a logger to an open bus session.

        Args:
            session: Open zenoh session; not owned.
            run_id: The run these spans belong to.
            source: Which organ is speaking, for the message header.
        """
        self._session = session
        self._run_id = run_id
        self._source = source
        self._lock = threading.Lock()
        self._next_id = 0
        self._stack: list[str] = []  # open spans, innermost last
        self._seq = 0

    @property
    def run_id(self) -> str:
        """The run these spans belong to."""
        return self._run_id

    @property
    def current_span(self) -> str:
        """Innermost open span id, or empty when none is open."""
        with self._lock:
            return self._stack[-1] if self._stack else ""

    def _new_span_id(self) -> str:
        """Allocate a span id unique within this run.

        Returns:
            The id.
        """
        with self._lock:
            self._next_id += 1
            return f"{self._run_id}-{self._next_id:04d}"

    def _publish(self, topic: str, payload: bytes) -> None:
        """Put one message on the bus, swallowing bus faults.

        Args:
            topic: Bus key.
            payload: Serialized message.
        """
        with contextlib.suppress(Exception):
            self._session.put(topic, payload)

    def start(
        self,
        name: str,
        kind: mission_log_pb2.SpanKind.ValueType = mission_log_pb2.SPAN_KIND_PHASE,
        attributes: dict[str, str] | None = None,
    ) -> str:
        """Open a span, nested inside whatever is currently open.

        Args:
            name: Span name.
            kind: Mission, phase or activity.
            attributes: Free-form key/values recorded with the span.

        Returns:
            The new span's id.
        """
        parent = self.current_span
        span_id = self._new_span_id()
        span = mission_log_pb2.Span(
            span_id=span_id,
            parent_span_id=parent,
            kind=kind,
            name=name,
            open=True,
            time_ns=time.time_ns(),
        )
        if attributes:
            span.attributes.update(attributes)
        self._stamp(span.header)
        with self._lock:
            self._stack.append(span_id)
        self._publish(SPAN_TOPIC, span.SerializeToString())
        return span_id

    def end(self, span_id: str, outcome: str = "", detail: str = "") -> None:
        """Close a span.

        Args:
            span_id: The span to close.
            outcome: pass, fail, aborted — empty when not a judgment.
            detail: One line of why.
        """
        span = mission_log_pb2.Span(
            span_id=span_id,
            kind=mission_log_pb2.SPAN_KIND_UNSPECIFIED,
            open=False,
            time_ns=time.time_ns(),
            outcome=outcome,
            detail=detail,
        )
        self._stamp(span.header)
        with self._lock:
            if span_id in self._stack:
                self._stack.remove(span_id)
        self._publish(SPAN_TOPIC, span.SerializeToString())

    def annotate(self, message: str, level: str = "info", source: str = "") -> None:
        """Record a point event against the innermost open span.

        Args:
            message: What happened.
            level: info, warn, error.
            source: Which organ; defaults to this logger's source.
        """
        note = mission_log_pb2.Annotation(
            time_ns=time.time_ns(),
            span_id=self.current_span,
            level=level,
            source=source or self._source,
            message=message,
        )
        self._stamp(note.header)
        self._publish(ANNOTATION_TOPIC, note.SerializeToString())

    def _stamp(self, header: object) -> None:
        """Fill the common message header.

        Args:
            header: The Header submessage to populate.
        """
        with self._lock:
            self._seq += 1
            seq = self._seq
        now = time.time_ns()
        header.source = self._source  # type: ignore[attr-defined]
        header.sample_time_ns = now  # type: ignore[attr-defined]
        header.publish_time_ns = now  # type: ignore[attr-defined]
        header.seq = seq  # type: ignore[attr-defined]

    @contextlib.contextmanager
    def span(
        self,
        name: str,
        kind: mission_log_pb2.SpanKind.ValueType = mission_log_pb2.SPAN_KIND_PHASE,
        attributes: dict[str, str] | None = None,
    ) -> Iterator[str]:
        """Open a span for the duration of a block.

        An exception closes the span with outcome ``aborted`` and
        re-raises: a run that dies mid-phase must leave evidence of what
        it was doing.

        Args:
            name: Span name.
            kind: Mission, phase or activity.
            attributes: Free-form key/values.

        Yields:
            The span id.
        """
        span_id = self.start(name, kind, attributes)
        try:
            yield span_id
        except BaseException as exc:
            self.end(span_id, outcome="aborted", detail=f"{type(exc).__name__}: {exc}")
            raise
        else:
            self.end(span_id)


def default_run_id(prefix: str = "run") -> str:
    """Build a run id that sorts chronologically and reads plainly.

    Args:
        prefix: Leading token, usually the mission name.

    Returns:
        An id like ``detumble-20260731T140233Z-4131``.
    """
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return f"{prefix}-{stamp}-{os.getpid()}"
