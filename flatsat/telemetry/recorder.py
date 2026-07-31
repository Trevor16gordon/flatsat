"""Bus recorder: subscribe, append verbatim, rotate, never fill the disk.

Design rules:
  * **Verbatim.** The recorder never parses payloads — topic, arrival
    time, raw bytes. A new message type or topic needs no recorder
    change, and the archive is byte-exact evidence.
  * **Bounded.** Files rotate at a size/time bound; the archive prunes
    its OLDEST files past a total-size cap. An indefinite service run
    must never fill the flight computer's disk — losing the oldest data
    is a decision, taken here, visibly, not an accident.
  * **Honest tails.** Writes are buffered (flushed on rotation and by
    the app's periodic flush); a crash can truncate the final frame.
    :func:`read_records` treats a truncated tail as end-of-file — the
    frames before it are intact evidence.

On disk: ``telemetry-<UTCstamp>-<seq>.rec`` files, each a stream of
``[4-byte little-endian length][RecordedSample]`` frames.
"""

from __future__ import annotations

import struct
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from typing import BinaryIO

import zenoh

from flatsat.msgs import mission_log_pb2, telemetry_pb2
from flatsat.telemetry import mission_log, telemetry_config_pb2

_LEN = struct.Struct("<I")
FILE_SUFFIX = ".rec"


def read_records(path: Path | str) -> Iterator[telemetry_pb2.RecordedSample]:
    """Iterate the frames of one archive file.

    Args:
        path: Archive file to read.

    Yields:
        Each recorded sample, in write order. A truncated final frame
        (crash or power loss mid-write) ends iteration silently — the
        preceding frames are intact.
    """
    with Path(path).expanduser().open("rb") as fh:
        while True:
            prefix = fh.read(_LEN.size)
            if len(prefix) < _LEN.size:
                return
            (length,) = _LEN.unpack(prefix)
            payload = fh.read(length)
            if len(payload) < length:
                return  # truncated tail: end of intact evidence
            yield telemetry_pb2.RecordedSample.FromString(payload)


class Recorder:
    """Archives configured bus topics to rotating, size-capped files."""

    def __init__(
        self,
        entry: telemetry_config_pb2.TelemetryConfig,
        session: zenoh.Session,
        session_header: mission_log_pb2.SessionHeader | None = None,
    ) -> None:
        """Open the archive and subscribe to every configured topic.

        Args:
            entry: Recorder composition (topics, directory, bounds).
            session: Open zenoh session; not owned — caller closes it.
            session_header: Run identity written as the first record of
                EVERY file. Files rotate and the oldest are pruned, so
                metadata kept only at the start of a run is the first
                thing lost; repeating it per file costs a few hundred
                bytes and makes each file interpretable alone.
        """
        self.entry = entry
        self._session_header = session_header
        self._dir = Path(entry.output_dir).expanduser()
        self._dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._seq = 0
        self._path = Path()
        self._file: BinaryIO = self._open_next()
        self._file_bytes = 0
        self._file_opened_monotonic = time.monotonic()
        self.window_records = 0
        self.window_bytes = 0
        self._subs = [session.declare_subscriber(t, self._on_sample) for t in entry.topics]

    # ------------------------------------------------------------- files --

    def _open_next(self) -> BinaryIO:
        """Open a fresh archive file (unique name, monotonic seq).

        Returns:
            The open binary file handle.
        """
        self._seq += 1
        stamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
        self._path = self._dir / f"telemetry-{stamp}-{self._seq:04d}{FILE_SUFFIX}"
        handle = self._path.open("ab")
        if self._session_header is not None:
            self._write_frame(
                handle,
                telemetry_pb2.RecordedSample(
                    topic=mission_log.SESSION_TOPIC,
                    recv_time_ns=time.time_ns(),
                    payload=self._session_header.SerializeToString(),
                ),
            )
        return handle

    @staticmethod
    def _write_frame(handle: BinaryIO, sample: telemetry_pb2.RecordedSample) -> int:
        """Append one length-prefixed record.

        Args:
            handle: Open archive file.
            sample: The record to append.

        Returns:
            Bytes written.
        """
        blob = sample.SerializeToString()
        handle.write(_LEN.pack(len(blob)))
        handle.write(blob)
        return _LEN.size + len(blob)

    def current_file(self) -> str:
        """Basename of the file currently being written.

        Returns:
            The basename, for health telemetry.
        """
        with self._lock:
            return self._path.name

    def archive_files(self) -> list[Path]:
        """List archive files, oldest first.

        Returns:
            Sorted paths (name order == time order by construction).
        """
        return sorted(self._dir.glob(f"telemetry-*{FILE_SUFFIX}"))

    def _rotate_locked(self) -> None:
        """Close the current file and open the next (lock held)."""
        self._file.close()
        self._file = self._open_next()
        self._file_bytes = 0
        self._file_opened_monotonic = time.monotonic()
        self._prune_locked()

    def _prune_locked(self) -> None:
        """Delete oldest files until the archive fits its cap (lock held)."""
        files = self.archive_files()
        current = self._path
        total = sum(f.stat().st_size for f in files)
        for oldest in files:
            if total <= self.entry.max_total_bytes:
                return
            if oldest == current:
                return  # never delete the live file
            total -= oldest.stat().st_size
            oldest.unlink()

    # ----------------------------------------------------------- recording --

    def _on_sample(self, sample: zenoh.Sample) -> None:
        """Append one bus message to the archive (subscriber thread).

        Args:
            sample: Incoming message on any subscribed topic.
        """
        record = telemetry_pb2.RecordedSample()
        record.topic = str(sample.key_expr)
        record.recv_time_ns = time.time_ns()
        record.payload = bytes(sample.payload.to_bytes())
        with self._lock:
            written = self._write_frame(self._file, record)
            self._file_bytes += written
            self.window_records += 1
            self.window_bytes += written
            file_age_s = time.monotonic() - self._file_opened_monotonic
            if self._file_bytes >= self.entry.max_file_bytes or (
                file_age_s >= self.entry.rotate_every_s
            ):
                self._rotate_locked()

    def flush(self) -> None:
        """Push buffered frames to the OS (bounds the crash-loss window)."""
        with self._lock:
            self._file.flush()

    def take_window(self) -> tuple[int, int]:
        """Return and reset the (records, bytes) counters for one window.

        Returns:
            Tuple of (records recorded, bytes appended) since last call.
        """
        with self._lock:
            window = (self.window_records, self.window_bytes)
            self.window_records = 0
            self.window_bytes = 0
            return window

    def totals(self) -> tuple[int, int]:
        """Current archive footprint.

        Returns:
            Tuple of (file count, total bytes on disk).
        """
        files = self.archive_files()
        return len(files), sum(f.stat().st_size for f in files)

    def close(self) -> None:
        """Undeclare subscriptions and close the current file."""
        for sub in self._subs:
            sub.undeclare()
        with self._lock:
            self._file.close()

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines naming topics, directory, and bounds.
        """
        return [
            f"recorder: topics {list(self.entry.topics)} -> {self._dir}",
            f"recorder: rotate at {self.entry.max_file_bytes} B / "
            f"{self.entry.rotate_every_s:g} s, cap {self.entry.max_total_bytes} B",
        ]
