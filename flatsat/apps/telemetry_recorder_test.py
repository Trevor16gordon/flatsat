"""Recorder app: health telemetry over the real bus."""

import threading
import time
from pathlib import Path

import pytest
import zenoh

from flatsat.apps.telemetry_recorder import RecorderApp
from flatsat.core.config import TelemetryEntry
from flatsat.core.health import health_topic
from flatsat.msgs import health_pb2
from flatsat.telemetry.recorder import Recorder

RECV_TIMEOUT_S = 5.0


@pytest.mark.verifies("FSW-TLM-005")
def test_recorder_publishes_health(tmp_path: Path) -> None:
    pub_session = zenoh.open(zenoh.Config())
    rec_session = zenoh.open(zenoh.Config())
    entry = TelemetryEntry(
        topics=("test/recapp/**",),
        output_dir=str(tmp_path),
        max_file_bytes=64 * 1024 * 1024,
        rotate_every_s=3600.0,
        max_total_bytes=1024 * 1024 * 1024,
    )
    recorder = Recorder(entry, rec_session)
    app = RecorderApp(recorder, rec_session)

    received: list[health_pb2.RecorderHealth] = []
    got = threading.Event()

    def on_health(sample: zenoh.Sample) -> None:
        received.append(health_pb2.RecorderHealth.FromString(bytes(sample.payload.to_bytes())))
        got.set()

    sub = pub_session.declare_subscriber(health_topic("recorder"), on_health)
    time.sleep(0.5)  # discovery

    for index in range(5):
        pub_session.put("test/recapp/x", bytes([index]) * 8)
    deadline = time.monotonic() + RECV_TIMEOUT_S
    while recorder.window_records < 5 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert recorder.window_records >= 5, "records never arrived"

    app.publish_health()
    got.wait(RECV_TIMEOUT_S)
    recorder.close()
    sub.undeclare()
    pub_session.close()
    rec_session.close()

    assert received, "no RecorderHealth published"
    health = received[0]
    assert health.window_records >= 5
    assert health.window_bytes > 0
    assert health.files >= 1
    assert health.total_bytes >= 0
    assert health.current_file.startswith("telemetry-")
    assert health.header.source == "recorder"
