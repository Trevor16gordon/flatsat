#!/usr/bin/env python3
"""⚠ TRANSMITS RF ⚠ — capture the raw demodulated bitstream and analyse it offline.

A diagnostic, not a verification. ``pluto_driver_ber_test.py`` measures
what the driver RECOVERS; this measures what actually ARRIVES, by
draining the modem's demodulated bits without letting the driver's
synchronizer consume them. The difference between the two answers the
question three bench runs could not: when a frame goes missing, was it
never on the air, or did the receiver code lose it?

The transmit side is the real driver, unchanged. Only the receive side
is bypassed, and only to observe it.

RF SAFETY: identical to the BER test — refuses without ``--transmit``,
cabled path with 30 dB pads only, transmitter silenced on every exit.

Usage:
  ~/venvs/flatsat-ml/bin/python radio/pluto_bitstream_probe.py --transmit
"""

from __future__ import annotations

import argparse
import fcntl
import struct
import sys
import termios
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from flatsat.comms.framing.ccsds import SYNC, CcsdsFramer  # noqa: E402
from flatsat.comms.phy.pluto_gmsk import PREAMBLE, PlutoGmskModem, _sync_bits  # noqa: E402


def eprint(*args: object) -> None:
    """Print to stderr.

    Args:
        args: Values to print.
    """
    print(*args, file=sys.stderr, flush=True)


def payload_for(index: int, size: int) -> bytes:
    """Build one deterministic, self-identifying test payload.

    Args:
        index: Frame number.
        size: Payload length in bytes.

    Returns:
        The payload.
    """
    head = f"FLATSAT-PROBE frame={index:04d} ".encode()
    return (head + bytes(range(256)) * 4)[:size]


def analyse(
    bits: np.ndarray,
    frames: int,
    payload_len: int,
    baud: float,
    sent_at: dict[int, float],
) -> None:
    """Report what the captured bitstream actually contains.

    Args:
        bits: The full demodulated bit stream, one bit per element.
        frames: How many frames were transmitted.
        payload_len: Payload bytes per frame.
        baud: Symbol rate, for turning bit offsets into arrival times.
        sent_at: MEASURED send time of each frame, seconds since the
            capture began. Deriving this from index*gap instead silently
            adds the warmup to every latency — which is exactly how a
            0.55 s pipeline was once reported as 1.65 s.
    """
    pattern = np.asarray(_sync_bits("1acffc1d"), dtype=np.int8) * 2 - 1
    correlation = np.correlate(bits.astype(np.int8) * 2 - 1, pattern, mode="valid")

    print("-" * 72)
    print(f"  bits captured    : {bits.size}")
    # Sweep the error budget: how tolerant must the hunt be to see them all?
    for budget in (0, 1, 2, 3, 4, 6, 8):
        hits = int(np.count_nonzero(correlation >= 32 - 2 * budget))
        print(f"  markers at <={budget} bit errors : {hits}")

    # Byte phase of every marker. If they all share one residue mod 8,
    # the demodulator never slipped and any "resync" the driver performs
    # is destroying a working alignment rather than repairing a broken one.
    strong = np.flatnonzero(correlation >= 32 - 2 * 2)
    if strong.size:
        phases = np.bincount(strong % 8, minlength=8)
        print(f"  marker phase mod 8: {list(phases)}")

    # Deframe from every candidate, independent of the driver's logic.
    framer = CcsdsFramer()
    hits = np.flatnonzero(correlation >= 32 - 2 * 2)
    good = 0
    seen: set[int] = set()
    arrival: dict[int, float] = {}
    for hit in hits:
        start = int(hit)
        need = (len(SYNC) + 2 + payload_len + 4) * 8
        if start + need > bits.size:
            break
        block = np.packbits(bits[start : start + need]).tobytes()
        for payload in framer.feed(block):
            text = payload[:24].decode("ascii", "replace")
            if "frame=" in text:
                index = int(text.split("frame=")[1][:4])
                seen.add(index)
                arrival[index] = start / baud
                good += 1
        framer._buffer.clear()  # noqa: SLF001 — each candidate judged independently

    print("-" * 72)
    print(f"  frames sent      : {frames}")
    print(f"  frames deframed  : {good}  (distinct indices: {len(seen)})")
    if seen:
        missing = sorted(set(range(frames)) - seen)
        print(f"  missing indices  : {len(missing)} -> {missing[:30]}")
        # Consecutive runs of loss mean bursts; scattered means per-frame.
        runs = 1 + sum(1 for a, b in zip(missing, missing[1:], strict=False) if b != a + 1)
        print(f"  loss runs        : {runs} (1 run = one burst; ~N runs = scattered)")
        print(f"  ALL missing      : {missing}")
    # Latency: arrival time in the capture minus the time this frame was
    # offered to send(). A CONSTANT offset is pipeline depth; a GROWING
    # one means the transmitter is falling behind the sender.
    if arrival:
        order = sorted(arrival)
        print("-" * 72)
        print("  index  arrival_s  send_s  latency_s")
        for i in order[:3] + order[len(order) // 2 : len(order) // 2 + 2] + order[-3:]:
            if i in sent_at:
                print(
                    f"  {i:5d}  {arrival[i]:9.3f}  {sent_at[i]:6.3f}  "
                    f"{arrival[i] - sent_at[i]:9.3f}"
                )
        span = [arrival[i] - sent_at[i] for i in order if i in sent_at]
        if span:
            print(f"  pipeline latency : {min(span):.3f}..{max(span):.3f} s")
    print("-" * 72)


def main() -> int:
    """Transmit frames and analyse the raw received bitstream.

    Returns:
        0 on a clean capture, 1 without --transmit, 2 if the radio failed.
    """
    parser = argparse.ArgumentParser(description="Raw bitstream probe over the cabled loopback.")
    parser.add_argument("--transmit", action="store_true", help="REQUIRED: this run radiates")
    parser.add_argument("--uri", default="ip:192.168.2.1")
    parser.add_argument("--freq", type=float, default=915e6)
    parser.add_argument("--rate", type=float, default=2.084e6)
    parser.add_argument("--tx-atten", type=float, default=20.0)
    parser.add_argument("--rx-gain", type=float, default=30.0)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--payload", type=int, default=64)
    parser.add_argument("--gap", type=float, default=0.02)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=2.0)
    args = parser.parse_args()

    if not args.transmit:
        eprint("REFUSED: this script transmits. Cabled path with 30 dB pads only.")
        return 1

    eprint("⚠  BITSTREAM PROBE — THIS RUN TRANSMITS RF (cabled + pads only)")
    modem = PlutoGmskModem(
        "probe_radio",
        uri=args.uri,
        center_freq_hz=args.freq,
        sample_rate_hz=args.rate,
        tx_attenuation_db=args.tx_atten,
        rx_gain_db=args.rx_gain,
        transmit_ack=True,
    )
    framer = CcsdsFramer()
    captured = bytearray()

    pending_peak = [0]

    def harvest() -> None:
        """Drain demodulated bits WITHOUT running the driver's synchronizer.

        Also records the peak bytes waiting in the RX pipe: if the
        receive side were hoarding, that is where it would show, and it
        would mean the measured latency is not all transmit-side.
        """
        if modem._rx_read_fd is not None:  # noqa: SLF001
            waiting = struct.unpack(
                "i", fcntl.ioctl(modem._rx_read_fd, termios.FIONREAD, b"\0" * 4)
            )[0]  # noqa: SLF001
            pending_peak[0] = max(pending_peak[0], waiting)
        modem._drain_rx_pipe()  # noqa: SLF001
        with modem._rx_lock:  # noqa: SLF001
            captured.extend(modem._rx_bits)  # noqa: SLF001
            modem._rx_bits.clear()  # noqa: SLF001

    sent_at: dict[int, float] = {}
    try:
        modem.receive()  # opens the radio (and starts the carrier)
        if modem.start_error:
            eprint(f"FAIL: could not open the radio: {modem.start_error}")
            return 2
        # Time zero is the first harvest: the capture and the clock must
        # share an origin or every latency is offset by the warmup.
        capture_start = time.monotonic()
        deadline = capture_start + args.warmup
        while time.monotonic() < deadline:
            harvest()
            time.sleep(0.05)

        for index in range(args.frames):
            sent_at[index] = time.monotonic() - capture_start
            modem.send(framer.frame(payload_for(index, args.payload)))
            time.sleep(args.gap)
            harvest()

        deadline = time.monotonic() + args.settle
        while time.monotonic() < deadline:
            harvest()
            time.sleep(0.05)
    finally:
        modem.close()
        eprint("[ok] transmitter silenced, radio released")

    print(f"  frames fed to mod: {modem.frames_transmitted} of {args.frames}")
    # One RX pipe byte is one bit, so divide by the bit rate for seconds.
    print(
        f"  RX pipe peak     : {pending_peak[0]} bits "
        f"({pending_peak[0] / (args.rate / 8.0):.3f} s of receive-side latency)"
    )
    analyse(
        np.frombuffer(bytes(captured), dtype=np.uint8),
        args.frames,
        args.payload,
        args.rate / 8.0,
        sent_at,
    )
    print(f"  (a frame is {len(PREAMBLE) + 4 + 2 + args.payload + 4} bytes on the wire)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
