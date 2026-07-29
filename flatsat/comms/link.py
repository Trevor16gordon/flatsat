"""The link layer: named messages ⇄ segments ⇄ frames, gated by contact.

Sits between the bus and the framing/PHY pair and owns everything the
modem must not know:

  * **Segmentation.** A logical message (a telemetry sample, a slice of
    an uplinked model) is split into segments sized for the channel, so
    one corrupted frame costs one segment rather than the whole message.
  * **Reassembly.** Segments are collected by message id; a message
    completes when every index has arrived, and ages out (counted) when
    they do not. Partial messages are NEVER delivered — a half a model
    is not a model.
  * **Contact windows.** Simulated pass geometry gates transmission even
    though both radios sit on one desk: telemetry accumulates between
    passes and dumps on contact (PLAN §6). The queue is bounded; a link
    that never opens must not exhaust memory.

Time comes in as arguments; the caller owns the clock, so windows and
aging are testable without sleeping.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing.framer import Framer
from flatsat.comms.modem import Modem
from flatsat.msgs import link_pb2

DEFAULT_SEGMENT_BYTES = 512
DEFAULT_QUEUE_LIMIT = 10000
REASSEMBLY_TIMEOUT_NS = int(60e9)


class ContactSchedule:
    """Simulated pass geometry: when the radio may transmit."""

    def __init__(self, period_s: float = 0.0, duration_s: float = 0.0) -> None:
        """Configure the pass cadence.

        Args:
            period_s: Pass repeat interval; 0 means always in contact.
            duration_s: Contact length within each period.
        """
        self._period_ns = int(period_s * 1e9)
        self._duration_ns = int(duration_s * 1e9)

    @classmethod
    def from_config(cls, options: comms_config_pb2.ContactScheduleConfig) -> ContactSchedule:
        """Build from vehicle-file contact options.

        Args:
            options: Typed options; an absent block means always in
                contact.

        Returns:
            The schedule.
        """
        return cls(period_s=options.period_s, duration_s=options.duration_s)

    @property
    def always_in_contact(self) -> bool:
        """Whether this schedule never gates the radio."""
        return self._period_ns <= 0

    def in_contact(self, elapsed_ns: int) -> bool:
        """Whether a pass is open at this point in the run.

        Args:
            elapsed_ns: Monotonic nanoseconds since the link started.

        Returns:
            True when transmission is permitted.
        """
        if self.always_in_contact:
            return True
        return (elapsed_ns % self._period_ns) < self._duration_ns

    def describe(self) -> str:
        """Render the schedule for logs.

        Returns:
            One line describing the pass cadence.
        """
        if self.always_in_contact:
            return "contact: continuous (no pass gating)"
        return f"contact: {self._duration_ns / 1e9:g} s pass every {self._period_ns / 1e9:g} s"


@dataclass
class _Assembly:
    """Segments collected so far for one inbound message."""

    total: int
    topic: str
    first_seen_ns: int
    chunks: dict[int, bytes] = field(default_factory=dict)

    def complete(self) -> bool:
        """Whether every segment has arrived."""
        return len(self.chunks) == self.total

    def payload(self) -> bytes:
        """Concatenate the segments in index order.

        Returns:
            The reassembled payload.
        """
        return b"".join(self.chunks[index] for index in range(self.total))


class Link:
    """Segments outbound messages, reassembles inbound ones, gates by pass."""

    def __init__(
        self,
        modem: Modem,
        framer: Framer,
        segment_bytes: int = DEFAULT_SEGMENT_BYTES,
        queue_limit: int = DEFAULT_QUEUE_LIMIT,
        contact: ContactSchedule | None = None,
    ) -> None:
        """Wire the layers together.

        Args:
            modem: The PHY; not owned — caller closes it.
            framer: Framing for every segment.
            segment_bytes: Payload bytes per segment.
            queue_limit: Messages stored between passes; the OLDEST are
                dropped beyond it (bounded memory beats an OOM in orbit).
            contact: Pass schedule; None means always in contact.
        """
        self._modem = modem
        self._framer = framer
        self._segment_bytes = segment_bytes
        self._contact = contact or ContactSchedule()
        self._queue: deque[tuple[str, bytes]] = deque(maxlen=queue_limit)
        self._assemblies: dict[int, _Assembly] = {}
        self._lock = threading.Lock()
        self._next_message_id = 0
        self.frames_sent = 0
        self.frames_received = 0
        self.messages_delivered = 0
        self.messages_incomplete = 0
        self.messages_dropped_queue = 0

    # ------------------------------------------------------------ outbound --

    def enqueue(self, topic: str, payload: bytes) -> None:
        """Store one message for the next contact.

        A full queue drops the OLDEST message, not this one — the deque's
        maxlen does it implicitly, so the only work here is counting it.
        That direction is deliberate: when a pass is missed, stale
        telemetry is the least valuable thing on board, and refusing new
        data to preserve old would hand the ground a picture that is
        already wrong.

        Args:
            topic: Destination bus key on the far side.
            payload: Serialized bus message.
        """
        with self._lock:
            if len(self._queue) == self._queue.maxlen:
                self.messages_dropped_queue += 1
            self._queue.append((topic, payload))

    @property
    def queued(self) -> int:
        """Messages waiting for a contact window."""
        with self._lock:
            return len(self._queue)

    def _segments(self, topic: str, payload: bytes) -> list[link_pb2.LinkSegment]:
        """Split one payload into wire segments.

        Args:
            topic: Destination bus key.
            payload: The bytes to carry.

        Returns:
            Segments covering the payload, at least one.
        """
        size = self._segment_bytes
        chunks = [payload[i : i + size] for i in range(0, len(payload), size)] or [b""]
        message_id = self._next_message_id
        self._next_message_id += 1
        return [
            link_pb2.LinkSegment(
                message_id=message_id,
                index=index,
                total=len(chunks),
                topic=topic,
                chunk=chunk,
            )
            for index, chunk in enumerate(chunks)
        ]

    def send_pending(self, elapsed_ns: int, max_messages: int = 64) -> int:
        """Transmit queued messages if a pass is open.

        Args:
            elapsed_ns: Monotonic nanoseconds since the link started.
            max_messages: Cap on messages drained per call, so one
                enormous backlog cannot monopolize a cycle.

        Returns:
            Messages transmitted (0 when out of contact — the queue is
            the point of store-and-forward).
        """
        if not self._contact.in_contact(elapsed_ns):
            return 0
        sent = 0
        for _ in range(max_messages):
            with self._lock:
                if not self._queue:
                    break
                topic, payload = self._queue.popleft()
                segments = self._segments(topic, payload)
            for segment in segments:
                self._modem.send(self._framer.frame(segment.SerializeToString()))
                self.frames_sent += 1
            sent += 1
        return sent

    # ------------------------------------------------------------- inbound --

    def poll(self, now_ns: int) -> list[tuple[str, bytes]]:
        """Drain the modem and return whatever messages completed.

        Args:
            now_ns: Current monotonic time (for reassembly aging).

        Returns:
            (topic, payload) pairs, only for messages received in full.
        """
        delivered: list[tuple[str, bytes]] = []
        for block in self._modem.receive():
            for framed in self._framer.feed(block):
                self.frames_received += 1
                segment = link_pb2.LinkSegment()
                try:
                    segment.ParseFromString(framed)
                except Exception:  # noqa: BLE001 — a CRC-valid frame can still be nonsense
                    continue
                assembly = self._assemblies.setdefault(
                    segment.message_id,
                    _Assembly(total=segment.total, topic=segment.topic, first_seen_ns=now_ns),
                )
                assembly.chunks[segment.index] = segment.chunk
                if assembly.complete():
                    delivered.append((assembly.topic, assembly.payload()))
                    self.messages_delivered += 1
                    del self._assemblies[segment.message_id]
        self._age_assemblies(now_ns)
        return delivered

    def _age_assemblies(self, now_ns: int) -> None:
        """Discard partial messages whose missing segments never came.

        Args:
            now_ns: Current monotonic time.
        """
        expired = [
            message_id
            for message_id, assembly in self._assemblies.items()
            if now_ns - assembly.first_seen_ns > REASSEMBLY_TIMEOUT_NS
        ]
        for message_id in expired:
            del self._assemblies[message_id]
            self.messages_incomplete += 1

    # -------------------------------------------------------------- status --

    def in_contact(self, elapsed_ns: int) -> bool:
        """Whether a pass is currently open.

        Args:
            elapsed_ns: Monotonic nanoseconds since the link started.

        Returns:
            True when transmission is permitted.
        """
        return self._contact.in_contact(elapsed_ns)

    @property
    def dropped_frames(self) -> int:
        """Frames the framer rejected for failing their integrity check."""
        return self._framer.dropped_frames

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines describing every layer in force.
        """
        return [
            *self._modem.describe(),
            *self._framer.describe(),
            f"link: {self._segment_bytes} B segments, queue limit "
            f"{self._queue.maxlen}, {self._contact.describe()}",
        ]
