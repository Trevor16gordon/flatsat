#!/usr/bin/env python3
"""⚠ TRANSMITS RF ⚠ — BER verification of the FLIGHT pluto_gmsk driver.

Closes the open item from the PHY graduation: the driver's transmit path
has never been exercised. This script drives the REAL flight code —
``flatsat.comms.phy.pluto_gmsk.PlutoGmskModem`` and
``flatsat.comms.framing.ccsds.CcsdsFramer`` — through the cabled
single-radio loopback and measures what actually comes back. It is
deliberately NOT a reimplementation: a bench script that re-derives the
modulation would prove the bench script works, not the driver.

    TX_pluto_1 -> 30 dB pads -> RX_pluto_1   (one radio, cabled, no antenna)

What it verifies:
  * frames sent through ``Modem.send()`` come back through
    ``Modem.receive()`` — the TX chain, the bit-level ASM sync, and the
    byte alignment, on real hardware;
  * the framer above accepts them (length + CRC), so recovered payloads
    are byte-exact;
  * the frame recovery rate and payload BER, comparable to the
    2026-07-23 bench baseline (113/113 frames, BER 0.00).

RF SAFETY (PLAN §2, FSW-RADIO-001/002):
  * refuses to run without ``--transmit``;
  * cabled path with pads inline only — never an antenna;
  * TX attenuation defaults high, and the transmitter is silenced on
    EVERY exit path including exceptions, because the Pluto's DMA can
    replay a stale buffer after a flowgraph stops.

Usage (from the repo root, after Trevor's per-instance go-ahead):
  ~/venvs/flatsat-ml/bin/python radio/pluto_driver_ber_test.py --transmit
  ... --frames 200 --tx-atten 50      # stronger signal, still padded
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from flatsat.comms.framing.ccsds import CcsdsFramer  # noqa: E402
from flatsat.comms.phy.pluto_gmsk import PlutoGmskModem  # noqa: E402


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
        The payload, recognizable on arrival even if frames are lost.
    """
    head = f"FLATSAT-DRIVER-BER frame={index:04d} ".encode()
    return (head + bytes(range(256)) * 4)[:size]


def bit_errors(sent: bytes, received: bytes) -> int:
    """Count differing bits between two equal-length payloads.

    Args:
        sent: The transmitted payload.
        received: The recovered payload.

    Returns:
        Number of differing bits.
    """
    return sum(bin(a ^ b).count("1") for a, b in zip(sent, received, strict=True))


def main() -> int:
    """Run the cabled BER verification of the flight driver.

    Returns:
        0 on a clean run; 1 without --transmit; 2 if the radio would not
        open; 3 if nothing was recovered.
    """
    parser = argparse.ArgumentParser(description="Driver BER test over the cabled loopback.")
    parser.add_argument(
        "--transmit",
        action="store_true",
        help="REQUIRED acknowledgement that this run radiates (cabled + pads only)",
    )
    parser.add_argument("--uri", default="ip:192.168.2.1", help="Pluto IIO URI")
    parser.add_argument("--freq", type=float, default=915e6, help="LO frequency [Hz]")
    # Defaults are the 2026-07-23 bench point that measured BER 0.00, not
    # freshly invented "safe-looking" numbers: changing two link
    # parameters at once is how a working configuration gets lost.
    parser.add_argument("--rate", type=float, default=2.084e6, help="sample rate [Sa/s]")
    parser.add_argument("--tx-atten", type=float, default=60.0, help="TX attenuation [dB]")
    parser.add_argument("--rx-gain", type=float, default=30.0, help="RX manual gain [dB]")
    parser.add_argument("--frames", type=int, default=100, help="frames to transmit")
    parser.add_argument("--payload", type=int, default=64, help="payload bytes per frame")
    parser.add_argument("--gap", type=float, default=0.02, help="seconds between frames")
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="seconds of idle carrier before the first frame, to let clock recovery lock",
    )
    parser.add_argument("--settle", type=float, default=2.0, help="seconds to drain at the end")
    args = parser.parse_args()

    if not args.transmit:
        eprint("REFUSED: this script transmits. Re-run with --transmit once the cabled")
        eprint("         path (TX -> 30 dB pads -> RX) is verified. No antennas, ever.")
        return 1

    print("=" * 72)
    print(" ⚠  PLUTO DRIVER BER TEST — THIS RUN TRANSMITS RF  ⚠")
    print(" cabled loopback only: TX -> 30 dB pads -> RX, no antenna attached")
    print("=" * 72, flush=True)

    modem = PlutoGmskModem(
        "bench_radio",
        uri=args.uri,
        center_freq_hz=args.freq,
        sample_rate_hz=args.rate,
        tx_attenuation_db=args.tx_atten,
        rx_gain_db=args.rx_gain,
        transmit_ack=True,  # the acknowledgement --transmit stands for
    )
    framer = CcsdsFramer()
    for line in (*modem.describe(), *framer.describe()):
        print(f"  {line}")

    sent: dict[bytes, int] = {}
    recovered: list[bytes] = []
    try:
        # Opening RX first lets the receiver settle before anything radiates.
        modem.receive()
        if modem.start_error:
            eprint(f"FAIL: could not open the radio: {modem.start_error}")
            return 2
        print("[ok] radio open; carrier up (idle fill)", flush=True)

        # Let the idle carrier run before the first frame: the
        # demodulator's clock recovery locks on 0x55's transitions, and
        # a frame sent into an unlocked receiver is simply wasted.
        deadline = time.monotonic() + args.warmup
        while time.monotonic() < deadline:
            modem.receive()
            time.sleep(0.05)
        print(f"[ok] warm; transmitting {args.frames} frames", flush=True)

        for index in range(args.frames):
            payload = payload_for(index, args.payload)
            sent[payload] = index
            flags = modem.send(framer.frame(payload))
            if flags:
                eprint(f"WARN: send flagged 0x{flags:x} on frame {index}")
            time.sleep(args.gap)
            recovered.extend(_drain(modem, framer))

        print(f"[ok] transmit complete; draining for {args.settle:g} s", flush=True)
        deadline = time.monotonic() + args.settle
        while time.monotonic() < deadline:
            recovered.extend(_drain(modem, framer))
            time.sleep(0.1)
    finally:
        modem.close()  # silences TX on every path, including exceptions
        print("[ok] transmitter silenced, radio released", flush=True)

    print("-" * 72)
    print(f"  bits demodulated : {modem.rx_bits_demodulated}")
    print(f"  sync detections  : {modem.sync_detections}")
    print(f"  sends dropped    : {modem.dropped_sends} (transmit queue full)")

    if not recovered:
        # Say which failure this is. The three have different fixes and
        # guessing between them costs a bench session each time.
        if modem.rx_bits_demodulated == 0:
            eprint("FAIL: no bits demodulated at all — RX chain never delivered samples.")
            eprint("      Suspect the flowgraph or the radio, not the link budget.")
        elif modem.sync_detections == 0:
            eprint("FAIL: bits are flowing but the sync marker never correlated.")
            eprint("      The receiver hears noise: check pads, --tx-atten, --rx-gain.")
        else:
            eprint("FAIL: sync locked but no frame survived the CRC.")
            eprint(f"      {framer.dropped_frames} frames dropped — the link is marginal,")
            eprint("      not absent. Try a lower --tx-atten or a longer --warmup.")
        return 3

    matched = 0
    errored = 0
    total_bits = 0
    total_errors = 0
    for payload in recovered:
        if payload in sent:
            matched += 1
            continue
        # A CRC-valid frame whose payload is not one we sent cannot happen
        # unless the payload was corrupted AND the CRC still passed.
        candidate = next((p for p in sent if len(p) == len(payload)), None)
        if candidate is not None:
            errors = bit_errors(candidate, payload)
            errored += 1
            total_errors += errors
            total_bits += len(payload) * 8
    total_bits += matched * args.payload * 8

    ber = (total_errors / total_bits) if total_bits else float("nan")
    print("-" * 72)
    print(f"  frames sent      : {args.frames}")
    print(f"  frames recovered : {len(recovered)}  ({100.0 * len(recovered) / args.frames:.1f}%)")
    print(f"  payloads exact   : {matched}")
    print(f"  payloads errored : {errored}")
    print(f"  frames dropped by CRC: {framer.dropped_frames}")
    print(f"  payload BER      : {ber:.2e}")
    print("-" * 72)
    print("  baseline for comparison: 2026-07-23 bench run, 113/113 frames, BER 0.00")
    print("=" * 72, flush=True)
    return 0


def _drain(modem: PlutoGmskModem, framer: CcsdsFramer) -> list[bytes]:
    """Pull whatever the radio has demodulated and deframe it.

    Args:
        modem: The modem under test.
        framer: The framer above it.

    Returns:
        Payloads recovered this call.
    """
    payloads: list[bytes] = []
    for block in modem.receive():
        payloads.extend(framer.feed(block))
    return payloads


if __name__ == "__main__":
    sys.exit(main())
