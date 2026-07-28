"""Recorder: verbatim capture, rotation, retention, honest tails."""

import struct
import time
from pathlib import Path

import pytest
import zenoh

from flatsat.msgs import hal_pb2, telemetry_pb2
from flatsat.telemetry import telemetry_config_pb2
from flatsat.telemetry.recorder import Recorder, read_records

RECV_TIMEOUT_S = 5.0


def _entry(
    tmp_path: Path, topic_root: str, **overrides: object
) -> telemetry_config_pb2.TelemetryConfig:
    defaults: dict[str, object] = {
        "topics": [f"{topic_root}/**"],
        "output_dir": str(tmp_path),
        "max_file_bytes": 64 * 1024 * 1024,
        "rotate_every_s": 3600.0,
        "max_total_bytes": 1024 * 1024 * 1024,
    }
    defaults.update(overrides)
    return telemetry_config_pb2.TelemetryConfig(**defaults)  # type: ignore[arg-type]


def _wait_until(predicate: object, timeout_s: float = RECV_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():  # type: ignore[operator]
            return True
        time.sleep(0.02)
    return False


@pytest.mark.verifies("FSW-TLM-001")
def test_records_are_byte_exact_with_topic_and_time(tmp_path: Path) -> None:
    pub_session = zenoh.open(zenoh.Config())
    rec_session = zenoh.open(zenoh.Config())
    recorder = Recorder(_entry(tmp_path, "test/rec/exact"), rec_session)
    time.sleep(0.5)  # discovery

    msg = hal_pb2.ImuSample()
    msg.header.source = "rec_test"
    msg.header.seq = 42
    msg.gyro_x_rad_s = 0.125
    wire = msg.SerializeToString()
    before_ns = time.time_ns()
    pub_session.put("test/rec/exact/imu", wire)

    assert _wait_until(lambda: recorder.window_records >= 1), "nothing recorded"
    recorder.flush()
    files = recorder.archive_files()
    recorder.close()
    pub_session.close()
    rec_session.close()

    records = [r for f in files for r in read_records(f)]
    assert len(records) == 1
    record = records[0]
    assert record.topic == "test/rec/exact/imu"
    assert record.payload == wire, "payload must be byte-exact"
    assert before_ns <= record.recv_time_ns <= time.time_ns()
    # And the evidence is decodable downstream:
    back = hal_pb2.ImuSample.FromString(record.payload)
    assert back.header.seq == 42


@pytest.mark.verifies("FSW-TLM-002")
def test_rotation_bounds_file_size(tmp_path: Path) -> None:
    pub_session = zenoh.open(zenoh.Config())
    rec_session = zenoh.open(zenoh.Config())
    # Tiny bound: every few records forces a rotation.
    recorder = Recorder(_entry(tmp_path, "test/rec/rotate", max_file_bytes=256), rec_session)
    time.sleep(0.5)

    payload = bytes(64)
    for _ in range(20):
        pub_session.put("test/rec/rotate/x", payload)
    assert _wait_until(lambda: recorder.window_records >= 20), "records missing"
    recorder.flush()
    files = recorder.archive_files()
    recorder.close()
    pub_session.close()
    rec_session.close()

    assert len(files) > 1, "size bound never rotated the file"
    for f in files[:-1]:  # every closed file respects the bound
        assert f.stat().st_size <= 256 + (4 + 64 + 64)  # bound + one frame of slack
    total = sum(1 for f in files for _ in read_records(f))
    assert total == 20, "rotation must not lose records"


@pytest.mark.verifies("FSW-TLM-003")
def test_retention_prunes_oldest_first(tmp_path: Path) -> None:
    pub_session = zenoh.open(zenoh.Config())
    rec_session = zenoh.open(zenoh.Config())
    recorder = Recorder(
        _entry(tmp_path, "test/rec/prune", max_file_bytes=256, max_total_bytes=1024),
        rec_session,
    )
    time.sleep(0.5)

    payload = bytes(64)
    for _ in range(60):
        pub_session.put("test/rec/prune/x", payload)
    assert _wait_until(lambda: recorder.window_records >= 60), "records missing"
    recorder.flush()
    files = recorder.archive_files()
    live = tmp_path / recorder.current_file()
    _, total_bytes = recorder.totals()
    recorder.close()
    pub_session.close()
    rec_session.close()

    assert total_bytes <= 1024 + 256, "archive exceeded its cap by more than one live file"
    assert live in files, "the live file must never be pruned"
    # Oldest-first: the earliest sequence numbers are the ones missing.
    names = sorted(f.name for f in files)
    assert "0001" not in names[0], "oldest file should have been pruned first"


@pytest.mark.verifies("FSW-TLM-004")
def test_truncated_tail_reads_cleanly(tmp_path: Path) -> None:
    """A crash mid-write must cost the last frame, not the archive."""
    path = tmp_path / "telemetry-fake-0001.rec"
    frames = []
    for index in range(3):
        record = telemetry_pb2.RecordedSample()
        record.topic = f"test/rec/tail/{index}"
        record.recv_time_ns = 1000 + index
        record.payload = b"x" * 10
        frames.append(record.SerializeToString())
    with path.open("wb") as fh:
        for frame in frames:
            fh.write(struct.pack("<I", len(frame)))
            fh.write(frame)
        # The crash: a length prefix promising more than was written.
        fh.write(struct.pack("<I", 999))
        fh.write(b"partial")

    records = list(read_records(path))
    assert len(records) == 3, "intact frames before the truncation must survive"
    assert [r.topic for r in records] == [f"test/rec/tail/{i}" for i in range(3)]


def test_empty_archive_reads_as_nothing(tmp_path: Path) -> None:
    path = tmp_path / "telemetry-empty-0001.rec"
    path.touch()
    assert list(read_records(path)) == []
