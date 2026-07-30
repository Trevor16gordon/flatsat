#!/usr/bin/env python3
"""⚠ TRANSMITS RF ⚠ — send a real file over the radio and stage it, verified.

This is the C1 uplink path over an actual radio rather than a loopback
fake. A file is chunked, segmented, framed, randomized, GMSK-modulated,
pushed out the Pluto, received back through 30 dB of cabled attenuation,
demodulated, deframed, reassembled, checked against the manifest's
sha256, and written to the staging directory. Every layer is the flight
implementation; nothing here reimplements the stack.

    file -> chunk_artifact -> Link -> CcsdsFramer -> PlutoGmskModem
         -> RF -> PlutoGmskModem -> CcsdsFramer -> Link -> UplinkReceiver

ONE RADIO, TALKING TO ITSELF. TX and RX are cabled together, so a single
Link instance hears its own transmissions. That makes this a genuine RF
transfer over coax rather than a simulation, but it is NOT two
independent stations: there is one oscillator, one clock, and no path
loss beyond the pads.

RETRIES ARE BY REPETITION, and that is the honest part. The link has no
ARQ: a frame that fails CRC is simply gone, and the Link delivers a
message only when every one of its segments arrived. With a few percent
frame loss, a multi-chunk artifact will not survive a single pass. So
missing chunks are re-sent in rounds until the transfer completes. Note
that the MANIFEST is sent exactly once — re-sending it resets the
transfer by design, which would discard every chunk already collected.

Staged is not deployed: the artifact lands inert. Activation is a
separate, explicitly authorized command (see uplink_service.py).

RF SAFETY: refuses without --transmit; cabled path with 30 dB pads only;
transmitter silenced on every exit path.

Usage:
  ~/venvs/flatsat-ml/bin/python radio/uplink_file_over_rf.py --transmit --file FILE
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from flatsat.comms.framing.ccsds import CcsdsFramer  # noqa: E402
from flatsat.comms.link import Link  # noqa: E402
from flatsat.comms.phy.pluto_gmsk import PlutoGmskModem  # noqa: E402
from flatsat.comms.uplink import (  # noqa: E402
    CHUNK_TOPIC,
    MANIFEST_TOPIC,
    UplinkReceiver,
    artifact_key,
    chunk_artifact,
)
from flatsat.msgs import uplink_pb2  # noqa: E402


def eprint(*args: object) -> None:
    """Print to stderr.

    Args:
        args: Values to print.
    """
    print(*args, file=sys.stderr, flush=True)


def missing_chunks(receiver: UplinkReceiver, name: str, version: str, total: int) -> list[int]:
    """Which chunk indices have not arrived yet.

    Args:
        receiver: The staging receiver.
        name: Artifact name.
        version: Artifact version.
        total: Declared chunk count.

    Returns:
        Sorted indices still outstanding.
    """
    # A completed transfer is REMOVED from _transfers once staged, so
    # "no transfer" is ambiguous: it means either nothing has arrived yet
    # or everything has. Check the staging area first, or a finished
    # upload looks like a failed one and gets re-sent forever.
    if receiver.staged_path(name, version) is not None:
        return []
    transfer = receiver._transfers.get(artifact_key(name, version))  # noqa: SLF001
    if transfer is None:
        return list(range(total))
    return [index for index in range(total) if index not in transfer.chunks]


def main() -> int:
    """Uplink one file over the cabled radio and verify it on arrival.

    Returns:
        0 when the file stages with a matching digest; 1 without
        --transmit; 2 if the radio would not open; 3 if the transfer did
        not complete within the round budget.
    """
    parser = argparse.ArgumentParser(description="Uplink a file over the Pluto link.")
    parser.add_argument("--transmit", action="store_true", help="REQUIRED: this run radiates")
    parser.add_argument("--file", type=Path, required=True, help="file to uplink")
    parser.add_argument("--name", default=None, help="artifact name (defaults to the filename)")
    parser.add_argument("--version", default="rf-1", help="artifact version")
    parser.add_argument(
        "--staging",
        type=Path,
        default=Path.home() / "uplink-staging",
        help="where verified artifacts land",
    )
    parser.add_argument("--uri", default="ip:192.168.2.1")
    parser.add_argument("--freq", type=float, default=915e6)
    parser.add_argument("--rate", type=float, default=2.084e6)
    parser.add_argument("--tx-atten", type=float, default=20.0)
    parser.add_argument("--rx-gain", type=float, default=30.0)
    parser.add_argument("--chunk-bytes", type=int, default=1024, help="artifact bytes per chunk")
    parser.add_argument("--segment-bytes", type=int, default=512, help="link segment size")
    parser.add_argument("--rounds", type=int, default=40, help="retry rounds before giving up")
    parser.add_argument("--warmup", type=float, default=1.0, help="idle carrier before first frame")
    args = parser.parse_args()

    if not args.transmit:
        eprint("REFUSED: this script transmits. Cabled path with 30 dB pads only.")
        return 1
    if not args.file.is_file():
        eprint(f"no such file: {args.file}")
        return 1

    payload = args.file.read_bytes()
    name = args.name or args.file.name
    digest = hashlib.sha256(payload).hexdigest()
    manifest, chunks = chunk_artifact(
        name, args.version, payload, uplink_pb2.ARTIFACT_KIND_CONFIG, args.chunk_bytes
    )

    print("=" * 72)
    print(" ⚠  UPLINK OVER RF — THIS RUN TRANSMITS (cabled + 30 dB pads)  ⚠")
    print("=" * 72)
    print(f"  file      : {args.file}  ({len(payload)} bytes)")
    print(f"  sha256    : {digest}")
    print(f"  artifact  : {name}@{args.version}  in {len(chunks)} chunks")
    print(f"  staging   : {args.staging}", flush=True)

    modem = PlutoGmskModem(
        "uplink_radio",
        uri=args.uri,
        center_freq_hz=args.freq,
        sample_rate_hz=args.rate,
        tx_attenuation_db=args.tx_atten,
        rx_gain_db=args.rx_gain,
        transmit_ack=True,
    )
    # One Link for both directions: TX and RX are the same radio, cabled,
    # so what this sends is what it hears.
    link = Link(modem, CcsdsFramer(), segment_bytes=args.segment_bytes)
    receiver = UplinkReceiver(args.staging)

    started = time.monotonic()

    def elapsed_ns() -> int:
        """Monotonic nanoseconds since the link started.

        Returns:
            Nanoseconds since start.
        """
        return int((time.monotonic() - started) * 1e9)

    def pump(seconds: float) -> None:
        """Run the link both ways for a while, dispatching what arrives.

        Args:
            seconds: How long to keep pumping.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            link.send_pending(elapsed_ns(), max_messages=2)
            for topic, data in link.poll(elapsed_ns()):
                if topic == MANIFEST_TOPIC:
                    receiver.on_manifest(uplink_pb2.ArtifactManifest.FromString(data))
                elif topic == CHUNK_TOPIC:
                    receiver.on_chunk(uplink_pb2.ArtifactChunk.FromString(data))
            time.sleep(0.02)

    staged_path = None
    try:
        modem.receive()
        if modem.start_error:
            eprint(f"FAIL: could not open the radio: {modem.start_error}")
            return 2
        print(f"[ok] radio open; carrier up, warming {args.warmup:g} s", flush=True)
        pump(args.warmup)

        # The manifest goes out ONCE. Re-sending it would reset the
        # transfer and discard every chunk collected so far.
        link.enqueue(MANIFEST_TOPIC, manifest.SerializeToString())
        pump(1.0)
        print(
            f"  manifest: sent {link.frames_sent} frame(s), "
            f"receiver tracking {receiver.receiving} transfer(s), "
            f"already staged: {receiver.staged_path(name, args.version) is not None}",
            flush=True,
        )

        for round_index in range(args.rounds):
            outstanding = missing_chunks(receiver, name, args.version, len(chunks))
            if not outstanding:
                # Say so. Breaking silently here is how a run that never
                # sent a chunk looked identical to one that succeeded.
                print(f"  round {round_index + 1:2d}: nothing outstanding", flush=True)
                break
            for index in outstanding:
                link.enqueue(CHUNK_TOPIC, chunks[index].SerializeToString())
            # Air time for what was just queued, plus the pipeline depth.
            pump(1.5 + 0.05 * len(outstanding))
            still = missing_chunks(receiver, name, args.version, len(chunks))
            if not still:
                print(f"  round {round_index + 1:2d}: complete", flush=True)
                break
            print(
                f"  round {round_index + 1:2d}: sent {len(outstanding)}, "
                f"have {len(chunks) - len(still)}/{len(chunks)} chunks",
                flush=True,
            )

        staged_path = receiver.staged_path(name, args.version)
    finally:
        link.close_contact()  # silence before anything else
        modem.close()
        print("[ok] transmitter silenced, radio released", flush=True)

    print("-" * 72)
    print(f"  frames sent      : {link.frames_sent}")
    print(f"  frames received  : {link.frames_received}")
    print(f"  frames dropped   : {link.dropped_frames} (CRC)")
    print(f"  messages incomplete: {link.messages_incomplete}")

    if staged_path is None:
        remaining = missing_chunks(receiver, name, args.version, len(chunks))
        eprint(f"FAIL: transfer incomplete — {len(remaining)}/{len(chunks)} chunks never arrived")
        return 3

    landed = staged_path.read_bytes()
    ok = hashlib.sha256(landed).hexdigest() == digest
    print(f"  staged at        : {staged_path}")
    print(f"  bytes            : {len(landed)} (sent {len(payload)})")
    print(f"  sha256 matches   : {ok}")
    print("=" * 72)
    if not ok:
        eprint("FAIL: digest mismatch — the receiver should never have staged this")
        return 3
    print("The file crossed the radio and was staged INERT. Activation is a")
    print("separate authorized command; arriving over RF deploys nothing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
