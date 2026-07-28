"""Artifact staging: reassemble, verify, and refuse anything that fails."""

import hashlib
from pathlib import Path

import pytest

from flatsat.comms.uplink import UplinkReceiver, chunk_artifact
from flatsat.msgs import uplink_pb2

PAYLOAD = bytes(range(256)) * 20  # 5 KiB


def _receiver(tmp_path: Path) -> UplinkReceiver:
    return UplinkReceiver(tmp_path / "staging")


@pytest.mark.verifies("FSW-UPL-001")
def test_chunked_artifact_stages_byte_exact(tmp_path: Path) -> None:
    receiver = _receiver(tmp_path)
    manifest, chunks = chunk_artifact(
        "ml_policy", "v1", PAYLOAD, uplink_pb2.ARTIFACT_KIND_MODEL, chunk_bytes=512
    )
    receiver.on_manifest(manifest)
    staged = [receiver.on_chunk(chunk) for chunk in chunks]

    assert staged[-1] == "ml_policy@v1", "completing chunk reports the staged key"
    assert all(result is None for result in staged[:-1]), "no early staging"
    path = receiver.staged_path("ml_policy", "v1")
    assert path is not None
    assert path.read_bytes() == PAYLOAD, "staged bytes must be byte-exact"
    assert receiver.staged() == ["ml_policy@v1"]


@pytest.mark.verifies("FSW-UPL-002")
def test_corrupted_artifact_is_rejected_not_staged(tmp_path: Path) -> None:
    """A digest mismatch means the bytes never reach the disk."""
    receiver = _receiver(tmp_path)
    manifest, chunks = chunk_artifact(
        "ml_policy", "v1", PAYLOAD, uplink_pb2.ARTIFACT_KIND_MODEL, chunk_bytes=512
    )
    receiver.on_manifest(manifest)
    corrupted = uplink_pb2.ArtifactChunk()
    corrupted.CopyFrom(chunks[3])
    corrupted.data = bytes(len(chunks[3].data))  # same length, wrong content
    for chunk in chunks[:3] + [corrupted] + chunks[4:]:
        receiver.on_chunk(chunk)

    assert receiver.staged() == [], "unverified bytes must never be staged"
    assert receiver.staged_path("ml_policy", "v1") is None
    assert receiver.rejected_checksum == 1


@pytest.mark.verifies("FSW-UPL-002")
def test_truncated_artifact_never_completes(tmp_path: Path) -> None:
    receiver = _receiver(tmp_path)
    manifest, chunks = chunk_artifact(
        "ml_policy", "v1", PAYLOAD, uplink_pb2.ARTIFACT_KIND_MODEL, chunk_bytes=512
    )
    receiver.on_manifest(manifest)
    for chunk in chunks[:-1]:  # the pass ended early
        receiver.on_chunk(chunk)
    assert receiver.staged() == []
    assert receiver.receiving == 1, "still awaiting the remainder"


def test_retransmitted_manifest_restarts_the_transfer(tmp_path: Path) -> None:
    """A ground retry after a failed pass is the normal case."""
    receiver = _receiver(tmp_path)
    manifest, chunks = chunk_artifact(
        "ml_policy", "v1", PAYLOAD, uplink_pb2.ARTIFACT_KIND_MODEL, chunk_bytes=512
    )
    receiver.on_manifest(manifest)
    receiver.on_chunk(chunks[0])
    receiver.on_manifest(manifest)  # retry from the top
    for chunk in chunks:
        receiver.on_chunk(chunk)
    assert receiver.staged() == ["ml_policy@v1"]


def test_unannounced_chunks_are_ignored(tmp_path: Path) -> None:
    """Chunks for an artifact nobody announced are not a transfer."""
    receiver = _receiver(tmp_path)
    _manifest, chunks = chunk_artifact("rogue", "v1", b"payload", uplink_pb2.ARTIFACT_KIND_MODEL)
    assert receiver.on_chunk(chunks[0]) is None
    assert receiver.staged() == []


def test_manifest_digest_matches_the_payload() -> None:
    manifest, _chunks = chunk_artifact("ml_policy", "v1", PAYLOAD, uplink_pb2.ARTIFACT_KIND_MODEL)
    assert manifest.sha256 == hashlib.sha256(PAYLOAD).hexdigest()
    assert manifest.size_bytes == len(PAYLOAD)


def test_multiple_versions_stage_side_by_side(tmp_path: Path) -> None:
    receiver = _receiver(tmp_path)
    for version, payload in (("v1", b"first"), ("v2", b"second")):
        manifest, chunks = chunk_artifact(
            "ml_policy", version, payload, uplink_pb2.ARTIFACT_KIND_MODEL
        )
        receiver.on_manifest(manifest)
        for chunk in chunks:
            receiver.on_chunk(chunk)
    assert receiver.staged() == ["ml_policy@v1", "ml_policy@v2"]
