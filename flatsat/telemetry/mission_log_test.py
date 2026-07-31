"""Mission structure: nesting, closure on failure, and self-describing files."""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.msgs import mission_log_pb2
from flatsat.telemetry import mission_log


class _Recorder:
    """Collects what a logger publishes, standing in for the bus."""

    def __init__(self) -> None:
        """Start with nothing captured."""
        self.spans: list[mission_log_pb2.Span] = []
        self.notes: list[mission_log_pb2.Annotation] = []

    def put(self, topic: str, payload: bytes) -> None:
        """Absorb one published message.

        Args:
            topic: Bus key.
            payload: Serialized message.
        """
        if topic == mission_log.SPAN_TOPIC:
            self.spans.append(mission_log_pb2.Span.FromString(payload))
        else:
            self.notes.append(mission_log_pb2.Annotation.FromString(payload))


@pytest.fixture(name="rig")
def fixture_rig() -> Iterator[tuple[mission_log.MissionLogger, _Recorder]]:
    """A logger writing into a capture buffer.

    Yields:
        Tuple of (logger, recorder).
    """
    sink = _Recorder()
    yield mission_log.MissionLogger(sink, "run-1"), sink


def test_spans_nest_by_enclosure(rig: tuple[mission_log.MissionLogger, _Recorder]) -> None:
    """A span opened inside another records it as its parent."""
    logger, sink = rig
    outer = logger.start("mission", mission_log_pb2.SPAN_KIND_MISSION)
    inner = logger.start("detumble", mission_log_pb2.SPAN_KIND_PHASE)
    logger.end(inner)
    logger.end(outer)

    opens = {s.span_id: s for s in sink.spans if s.open}
    assert opens[outer].parent_span_id == "", "the mission span is the root"
    assert opens[inner].parent_span_id == outer


def test_span_ids_are_unique_within_a_run(
    rig: tuple[mission_log.MissionLogger, _Recorder],
) -> None:
    logger, sink = rig
    for index in range(20):
        logger.end(logger.start(f"phase{index}"))
    ids = [s.span_id for s in sink.spans if s.open]
    assert len(set(ids)) == 20
    assert all(i.startswith("run-1-") for i in ids)


def test_annotations_attach_to_the_innermost_open_span(
    rig: tuple[mission_log.MissionLogger, _Recorder],
) -> None:
    """An event belongs to what was happening when it happened."""
    logger, sink = rig
    outer = logger.start("mission", mission_log_pb2.SPAN_KIND_MISSION)
    inner = logger.start("safe entry")
    logger.annotate("fdir tripped", level="error", source="fdir")
    logger.end(inner)
    logger.annotate("mission ending", source="runner")
    logger.end(outer)

    assert sink.notes[0].span_id == inner
    assert sink.notes[0].level == "error"
    assert sink.notes[1].span_id == outer, "closing a child returns to its parent"


def test_a_failing_block_still_closes_its_span(
    rig: tuple[mission_log.MissionLogger, _Recorder],
) -> None:
    """A run that dies mid-phase must leave evidence of what it was doing."""
    logger, sink = rig
    with pytest.raises(RuntimeError, match="plant died"), logger.span("detumble"):
        raise RuntimeError("plant died")

    closes = [s for s in sink.spans if not s.open]
    assert len(closes) == 1
    assert closes[0].outcome == "aborted"
    assert "plant died" in closes[0].detail


def test_span_edges_are_separate_messages(
    rig: tuple[mission_log.MissionLogger, _Recorder],
) -> None:
    """Start and end are distinct records, so an unclosed span is visible."""
    logger, sink = rig
    logger.start("never closed")
    assert [s.open for s in sink.spans] == [True]


def test_bus_faults_never_reach_the_caller() -> None:
    """Losing an annotation is bad; losing the spacecraft is worse."""

    class _Broken:
        def put(self, topic: str, payload: bytes) -> None:
            raise OSError("bus down")

    logger = mission_log.MissionLogger(_Broken(), "run-x")
    span = logger.start("phase")  # must not raise
    logger.annotate("something")
    logger.end(span)


# ------------------------------------------------------------- provenance --


def test_session_header_records_what_produced_the_data() -> None:
    """Sim, HIL and flight publish identical topics — the header separates them."""
    header = mission_log.build_session_header(
        run_id="run-9",
        source_kind=mission_log_pb2.SOURCE_KIND_HIL,
        mission_name="detumble",
        vehicle_path="config/vehicles/flatsat_v1.txtpb",
        plant="basilisk",
    )
    assert header.source_kind == mission_log_pb2.SOURCE_KIND_HIL
    assert header.host and header.started_wall_ns > 0
    assert header.plant == "basilisk"


def test_git_revision_reports_dirtiness() -> None:
    """A run with uncommitted changes is not reproducible, and must say so."""
    sha, dirty = mission_log.git_revision()
    assert isinstance(dirty, bool)
    assert sha == "" or len(sha) >= 7


def test_run_ids_sort_chronologically() -> None:
    first = mission_log.default_run_id("detumble")
    time.sleep(1.05)
    assert first < mission_log.default_run_id("detumble")


@pytest.mark.verifies("FSW-TLM-004")
def test_every_archive_file_starts_with_the_session_header(tmp_path: object) -> None:
    """Files rotate and prune, so per-run metadata must repeat per file."""
    from flatsat.telemetry import telemetry_config_pb2
    from flatsat.telemetry.recorder import Recorder, read_records

    session = zenoh.open(zenoh.Config())
    try:
        entry = telemetry_config_pb2.TelemetryConfig(
            topics=["test/mission_log/**"],
            output_dir=str(tmp_path),
            max_file_bytes=1 << 20,
            rotate_every_s=3600.0,
            max_total_bytes=1 << 24,
        )
        header = mission_log.build_session_header(
            "run-42", mission_log_pb2.SOURCE_KIND_SIM, mission_name="m"
        )
        recorder = Recorder(entry, session, session_header=header)
        try:
            recorder._rotate_locked()  # noqa: SLF001 — force a second file
        finally:
            recorder.close()

        files = recorder.archive_files()
        assert len(files) >= 2, "expected a rotation"
        for path in files:
            first = next(read_records(path))
            assert first.topic == mission_log.SESSION_TOPIC, f"{path.name} is not self-describing"
            recovered = mission_log_pb2.SessionHeader.FromString(first.payload)
            assert recovered.run_id == "run-42"
    finally:
        session.close()
