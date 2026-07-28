"""Artifact uplink: receive bytes over the link, verify, stage — nothing more.

The receive half of the C1 path. Chunks arrive as ordinary bus messages
(having crossed the RF link like any other traffic), are reassembled
against their manifest, and are written to a staging directory ONLY if
the SHA-256 matches. A digest mismatch is not a warning: the bytes are
discarded, because a partially-correct model is indistinguishable from a
malicious one and neither belongs on a flight computer.

This module never activates, never executes, and never imports what it
receives. Staging and activation are deliberately separate steps with
separate authority (see :mod:`flatsat.comms.slots`) — trust is earned by
measurement, not by arrival.
"""

from __future__ import annotations

import hashlib
import threading
from dataclasses import dataclass, field
from pathlib import Path

from flatsat.msgs import uplink_pb2

MANIFEST_TOPIC = "uplink/artifact/manifest"
CHUNK_TOPIC = "uplink/artifact/chunk"
CONTROL_TOPIC = "uplink/artifact/control"


def artifact_key(name: str, version: str) -> str:
    """Render the ``name@version`` identity used in telemetry and paths.

    Args:
        name: Artifact name.
        version: Artifact version.

    Returns:
        The combined key.
    """
    return f"{name}@{version}"


@dataclass
class _Transfer:
    """Chunks collected so far for one inbound artifact."""

    manifest: uplink_pb2.ArtifactManifest
    chunks: dict[int, bytes] = field(default_factory=dict)

    def complete(self) -> bool:
        """Whether every declared chunk has arrived."""
        return len(self.chunks) == self.manifest.chunk_count

    def assembled(self) -> bytes:
        """Concatenate the chunks in index order.

        Returns:
            The complete artifact bytes.
        """
        return b"".join(self.chunks[index] for index in range(self.manifest.chunk_count))


class UplinkReceiver:
    """Reassembles uplinked artifacts and stages the ones that verify."""

    def __init__(self, staging_dir: Path | str) -> None:
        """Bind the receiver to its staging directory.

        Args:
            staging_dir: Where verified artifacts land. Created if
                absent; one subdirectory per ``name@version``.
        """
        self.staging_dir = Path(staging_dir).expanduser()
        self.staging_dir.mkdir(parents=True, exist_ok=True)
        self._transfers: dict[str, _Transfer] = {}
        self._lock = threading.Lock()
        self.rejected_checksum = 0

    def on_manifest(self, manifest: uplink_pb2.ArtifactManifest) -> None:
        """Begin (or restart) a transfer.

        A repeated manifest resets the transfer: the ground retrying a
        failed uplink is the normal case, not an error.

        Args:
            manifest: The artifact's declared identity and digest.
        """
        with self._lock:
            self._transfers[artifact_key(manifest.name, manifest.version)] = _Transfer(
                manifest=manifest
            )

    def on_chunk(self, chunk: uplink_pb2.ArtifactChunk) -> str | None:
        """Absorb one chunk, staging the artifact when it completes.

        Args:
            chunk: One slice of an artifact already announced.

        Returns:
            The ``name@version`` key when this chunk completed a
            VERIFIED artifact; None while incomplete, unannounced, or
            when the completed bytes failed their digest.
        """
        key = artifact_key(chunk.name, chunk.version)
        with self._lock:
            transfer = self._transfers.get(key)
            if transfer is None:
                return None  # chunk for an artifact we were never told about
            transfer.chunks[chunk.index] = chunk.data
            if not transfer.complete():
                return None
            payload = transfer.assembled()
            del self._transfers[key]

        digest = hashlib.sha256(payload).hexdigest()
        if digest != transfer.manifest.sha256 or len(payload) != transfer.manifest.size_bytes:
            with self._lock:
                self.rejected_checksum += 1
            return None  # discarded: unverified bytes never reach the disk
        self._write(transfer.manifest, payload)
        return key

    def _write(self, manifest: uplink_pb2.ArtifactManifest, payload: bytes) -> None:
        """Write a verified artifact and its manifest to staging.

        Args:
            manifest: The verified artifact's manifest.
            payload: The verified bytes.
        """
        target = self.staging_dir / artifact_key(manifest.name, manifest.version)
        target.mkdir(parents=True, exist_ok=True)
        (target / "artifact.bin").write_bytes(payload)
        (target / "manifest.pb").write_bytes(manifest.SerializeToString())

    def staged(self) -> list[str]:
        """Artifacts fully received and verified.

        Returns:
            ``name@version`` keys present in staging, sorted.
        """
        return sorted(
            path.name for path in self.staging_dir.iterdir() if (path / "artifact.bin").is_file()
        )

    def staged_path(self, name: str, version: str) -> Path | None:
        """Locate one staged artifact's bytes.

        Args:
            name: Artifact name.
            version: Artifact version.

        Returns:
            Path to the staged bytes, or None when not staged.
        """
        candidate = self.staging_dir / artifact_key(name, version) / "artifact.bin"
        return candidate if candidate.is_file() else None

    @property
    def receiving(self) -> int:
        """Artifacts with chunks still arriving."""
        with self._lock:
            return len(self._transfers)


def chunk_artifact(
    name: str, version: str, payload: bytes, kind: int, chunk_bytes: int = 4096
) -> tuple[uplink_pb2.ArtifactManifest, list[uplink_pb2.ArtifactChunk]]:
    """Prepare an artifact for uplink (the GROUND side of the transfer).

    Args:
        name: Artifact name.
        version: Artifact version.
        payload: The complete artifact bytes.
        kind: ArtifactKind value.
        chunk_bytes: Bytes per chunk before link segmentation.

    Returns:
        Tuple of (manifest, chunks) ready to publish.
    """
    chunks = [payload[i : i + chunk_bytes] for i in range(0, len(payload), chunk_bytes)] or [b""]
    manifest = uplink_pb2.ArtifactManifest(
        name=name,
        version=version,
        kind=kind,  # type: ignore[arg-type]
        size_bytes=len(payload),
        sha256=hashlib.sha256(payload).hexdigest(),
        chunk_count=len(chunks),
    )
    messages = [
        uplink_pb2.ArtifactChunk(name=name, version=version, index=index, data=data)
        for index, data in enumerate(chunks)
    ]
    return manifest, messages
