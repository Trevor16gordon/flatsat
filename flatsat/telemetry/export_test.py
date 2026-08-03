"""Export: a run becomes one blob, and an interrupted run still says so."""

import json
from pathlib import Path

import pytest

from flatsat.msgs import hal_pb2, mission_log_pb2, telemetry_pb2
from flatsat.telemetry import export, mission_log
from flatsat.telemetry.recorder import Recorder


def _write(path: Path, records: list[tuple[str, bytes]]) -> None:
    """Write a synthetic archive file.

    Args:
        path: File to create.
        records: (topic, payload) pairs, in order.
    """
    with path.open("wb") as handle:
        for index, (topic, payload) in enumerate(records):
            Recorder._write_frame(  # noqa: SLF001 — the archive's own framing
                handle,
                telemetry_pb2.RecordedSample(
                    topic=topic, recv_time_ns=1_000_000_000 + index * 10_000_000, payload=payload
                ),
            )


def _span(span_id: str, name: str, *, open_: bool, parent: str = "", outcome: str = "") -> bytes:
    """Build a span edge.

    Args:
        span_id: Span identity.
        name: Span name.
        open_: True for the opening edge.
        parent: Parent span id.
        outcome: Closing outcome.

    Returns:
        Serialized span.
    """
    span = mission_log_pb2.Span(
        span_id=span_id,
        parent_span_id=parent,
        kind=mission_log_pb2.SPAN_KIND_PHASE,
        name=name,
        open=open_,
        time_ns=1_000 if open_ else 2_000,
        outcome=outcome,
    )
    return bytes(span.SerializeToString())


@pytest.fixture(name="archive")
def fixture_archive(tmp_path: Path) -> Path:
    """An archive holding a session header, a span pair and telemetry.

    Returns:
        The archive directory.
    """
    header = mission_log.build_session_header(
        "run-7", mission_log_pb2.SOURCE_KIND_HIL, mission_name="demo", plant="basilisk"
    )
    records: list[tuple[str, bytes]] = [
        (mission_log.SESSION_TOPIC, header.SerializeToString()),
        (mission_log.SPAN_TOPIC, _span("s1", "detumble", open_=True)),
        (
            mission_log.ANNOTATION_TOPIC,
            mission_log_pb2.Annotation(
                time_ns=1_500, span_id="s1", level="warn", source="fdir", message="rate high"
            ).SerializeToString(),
        ),
    ]
    for step in range(50):
        records.append(
            (
                "hal/imu0/sample",
                hal_pb2.ImuSample(gyro_x_rad_s=0.1 * step, gyro_y_rad_s=-0.05).SerializeToString(),
            )
        )
    records.append((mission_log.SPAN_TOPIC, _span("s1", "detumble", open_=True, outcome="pass")))
    records[-1] = (mission_log.SPAN_TOPIC, _span("s1", "", open_=False, outcome="pass"))
    _write(tmp_path / "telemetry-0001.rec", records)
    return tmp_path


def test_run_identity_survives_the_round_trip(archive: Path) -> None:
    """Provenance is the point: a blob must say what produced it."""
    blob = export.export_run(archive)
    assert blob["run"]["run_id"] == "run-7"
    assert blob["run"]["source_kind"] == "SOURCE_KIND_HIL"
    assert blob["run"]["plant"] == "basilisk"


def test_span_is_assembled_from_both_edges(archive: Path) -> None:
    """Start and end arrive as separate records and must rejoin."""
    span = export.export_run(archive)["spans"][0]
    assert span["name"] == "detumble"
    assert span["start_ns"] == 1_000
    assert span["end_ns"] == 2_000
    assert span["outcome"] == "pass"


def test_an_unclosed_span_is_visible_as_unclosed(tmp_path: Path) -> None:
    """A run that died mid-phase must not look like one that finished."""
    _write(tmp_path / "a.rec", [(mission_log.SPAN_TOPIC, _span("s9", "cut short", open_=True))])
    span = export.export_run(tmp_path)["spans"][0]
    assert span["start_ns"] is not None
    assert span["end_ns"] is None, "an open span must stay open in the blob"


def test_annotations_are_carried_with_their_span(archive: Path) -> None:
    note = export.export_run(archive)["annotations"][0]
    assert note["span_id"] == "s1"
    assert note["level"] == "warn"
    assert note["message"] == "rate high"


def test_numeric_leaves_become_plottable_series(archive: Path) -> None:
    """Nested protobuf projects onto the flat namespace a plot needs."""
    series = export.export_run(archive)["series"]
    assert "hal/imu0/sample.gyro_x_rad_s" in series
    assert series["hal/imu0/sample.gyro_x_rad_s"]["count"] == 49  # step 0 leaves the field unset
    assert len(series["hal/imu0/sample.gyro_x_rad_s"]["t_ns"]) == 49


def test_header_fields_are_not_treated_as_channels(archive: Path) -> None:
    """Sequence numbers and timestamps are metadata, not telemetry."""
    assert not [k for k in export.export_run(archive)["series"] if ".header." in k]


def test_series_decimate_and_admit_it(archive: Path) -> None:
    """A viewer wants shape; the archive keeps the fidelity."""
    blob = export.export_run(archive, max_points=10)
    entry = blob["series"]["hal/imu0/sample.gyro_x_rad_s"]
    assert entry["decimated"] is True
    assert len(entry["t_ns"]) <= 12
    assert entry["count"] == 49, "the true count must survive decimation"


def test_decimation_keeps_the_last_sample(archive: Path) -> None:
    """Striding must not make a series appear to stop early."""
    full = export.export_run(archive)["series"]["hal/imu0/sample.gyro_x_rad_s"]
    small = export.export_run(archive, max_points=5)["series"]["hal/imu0/sample.gyro_x_rad_s"]
    assert small["t_ns"][-1] == full["t_ns"][-1]


def test_topic_statistics_are_reported(archive: Path) -> None:
    stats = export.export_run(archive)["topics"]["hal/imu0/sample"]
    assert stats["count"] == 50
    assert stats["rate_hz"] > 0


def test_unmapped_topics_still_count(tmp_path: Path) -> None:
    """An unknown message must contribute presence, not vanish."""
    payload = hal_pb2.ImuSample(gyro_x_rad_s=1.0).SerializeToString()
    _write(tmp_path / "a.rec", [("something/entirely/unknown", payload)])
    blob = export.export_run(tmp_path)
    assert blob["topics"]["something/entirely/unknown"]["count"] == 1


def test_corrupt_records_do_not_abort_the_export(tmp_path: Path) -> None:
    """One bad record must not cost the whole run."""
    _write(
        tmp_path / "a.rec",
        [("hal/imu0/sample", b"\xff\xff\xff\xff not a protobuf")]
        + [("hal/imu0/sample", hal_pb2.ImuSample(gyro_x_rad_s=2.0).SerializeToString())],
    )
    blob = export.export_run(tmp_path)
    assert blob["topics"]["hal/imu0/sample"]["count"] == 2


def test_empty_archive_fails_loudly(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="no .rec files"):
        export.export_run(tmp_path)


def test_blob_is_json_serializable(archive: Path) -> None:
    """The whole point is handing this to a web app."""
    json.dumps(export.export_run(archive))


def test_topic_patterns_resolve_every_kind_distinctly() -> None:
    """Each recorded topic shape decodes with ITS OWN message type.

    The dangerous pair is magnetometer-vs-imu: MagnetometerSample reuses
    ImuSample's field numbers, so a fallthrough would produce plausible
    numbers under wrong names — worse than no decode.
    """
    from flatsat.msgs import adcs_pb2, hal_pb2, sim_pb2
    from flatsat.telemetry.export import _message_for

    cases = {
        "sim/truth/state": sim_pb2.TruthState,
        "test/scn/sim/truth": sim_pb2.TruthState,
        "hal/mag0/sample": hal_pb2.MagnetometerSample,
        "test/bdt/hal/mag/sample": hal_pb2.MagnetometerSample,
        "hal/imu0/sample": hal_pb2.ImuSample,
        "hal/mtq_x/state": hal_pb2.MagnetorquerState,
        "hal/wheel0/state": hal_pb2.WheelState,
        "sim/wheel/wheel0/torque": sim_pb2.WheelAxisTorque,
        "test/scn/adcs/torque": adcs_pb2.WheelTorqueCommand,
        "adcs/wheel_torque": adcs_pb2.WheelTorqueCommand,
        "sim/mtq/mtq_x/dipole": sim_pb2.MagnetorquerDipole,
        "adcs/dipole": adcs_pb2.DipoleCommand,
        "health/imu0": hal_pb2.HeaderEnvelope,
    }
    for topic, expected in cases.items():
        assert _message_for(topic) is expected, f"{topic} decoded as the wrong type"
