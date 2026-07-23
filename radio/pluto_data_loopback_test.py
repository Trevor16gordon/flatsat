#!/usr/bin/env python3
"""Data TX→RX loopback test for the ADALM-Pluto — THIS SCRIPT TRANSMITS RF.

==============================================================
 ⚠⚠⚠  THIS SCRIPT TRANSMITS ON THE PLUTO TX PORT  ⚠⚠⚠
==============================================================

Run policy (PLAN.md §0): explicit per-instance approval required. Refuses to
run without the ``--transmit`` flag. Preconditions, verified by the operator:

  * TX → 30 dB of SMA pads → RX physically cabled on the SAME Pluto.
  * Never TX→RX without the pads inline. Cabled only — no antennas.

RF levels are identical to the passed tone test (GMSK is constant-envelope,
|baseband| = amplitude): amplitude 0.5 + 20 dB TX attenuation ≈ −19 dBm at
the TX port → 30 dB pads ≈ −49 dBm at RX. Worst-case parameter choice stays
pad-safe (see pluto_tx_loopback_test.py for the budget derivation).

What it proves (P3 precursor — first DATA over the link):
  Real bytes are framed (64-bit GR access code + 64-byte ASCII payload),
  GMSK-modulated with the stock ``digital.gmsk_mod`` hierarchical block,
  transmitted through the pads, and demodulated with ``digital.gmsk_demod``
  (quadrature demod + clock recovery, all built-in). Frames are then located
  by access-code correlation and payload bits are compared to ground truth:
  frames found, frames decoded perfectly, aggregate payload BER, and the
  recovered ASCII text.

Modem choice: GMSK rather than hand-assembled BPSK because the paired
mod/demod hier blocks bundle pulse shaping and clock recovery that are known
to work together — the right tool for "prove data flows". The block-by-block
BPSK/ BER-curve modem is milestone B2 and gets its own treatment.

Quiet state: TX attenuation slammed to max (89.75 dB) on every exit path.

Usage (requires explicit flag):
  ./pluto_data_loopback_test.py --transmit
  ./pluto_data_loopback_test.py --transmit --tx-atten 30 --rx-gain 20

Exit code 0 = pass; non-zero identifies the failed stage.
"""

from __future__ import annotations

import argparse
import sys
import time

FLATSAT_PLUTO_SERIAL = "104473b04a06000602001c00dd1f84cfaa"  # role: flat-sat radio
TX_ATTEN_SILENT_DB = 89.75  # max AD936x TX attenuation — the "off" quiet state
CAPTURE_TIMEOUT_S = 30.0
PAYLOAD_LEN = 64  # bytes per frame after the 8-byte access code
MESSAGE = b"FLATSAT DATA LINK 001 * THE QUICK BROWN FOX JUMPS * 0123456789"


def eprint(*args: object) -> None:
    """Print to stderr.

    Args:
        *args: Objects to print, passed through to ``print``.
    """
    print(*args, file=sys.stderr)


def resolve_uri(cli_uri: str | None) -> str:
    """Resolve the IIO context URI for the Pluto.

    Args:
        cli_uri: Explicit URI from the command line, or None to auto-detect.

    Returns:
        The explicit URI if given, else gr-iio's auto-detected Pluto URI,
        else the Pluto's default USB-network address.
    """
    if cli_uri:
        return cli_uri
    try:
        from gnuradio import iio

        detected = iio.get_pluto_uri()
        if detected:
            return str(detected)
    except Exception:  # noqa: BLE001 — any detection failure falls through
        pass
    return "ip:192.168.2.1"


def make_rx_source(uri: str, freq: float, rate: float, gain: float) -> object:
    """Construct the RX fmcomms2 source (verified in-tree gr-iio 3.10 API).

    Args:
        uri: IIO context URI.
        freq: RX LO frequency in Hz.
        rate: Sample rate in samples/second.
        gain: Manual RX gain in dB.

    Returns:
        The constructed gr-iio source block (opaque C++ binding).
    """
    from gnuradio import iio

    src = iio.fmcomms2_source_fc32(uri, [True, True], 0x8000)
    src.set_len_tag_key("")
    src.set_frequency(int(freq))
    src.set_samplerate(int(rate))
    src.set_gain_mode(0, "manual")
    src.set_gain(0, float(gain))
    src.set_quadrature(True)
    src.set_rfdc(True)
    src.set_bbdc(True)
    src.set_filter_params("Auto", "", 0.0, 0.0)
    return src


def make_tx_sink(uri: str, freq: float, rate: float, atten: float) -> object:
    """Construct the TX fmcomms2 sink — THE BLOCK THAT TRANSMITS.

    Args:
        uri: IIO context URI.
        freq: TX LO frequency in Hz.
        rate: Sample rate in samples/second.
        atten: TX attenuation in dB (0 = full power ≈ +7 dBm; 89.75 = silent).

    Returns:
        The constructed gr-iio sink block (opaque C++ binding).
    """
    from gnuradio import iio

    snk = iio.fmcomms2_sink_fc32(uri, [True, True], 0x8000, False)
    snk.set_len_tag_key("")
    snk.set_frequency(int(freq))
    snk.set_samplerate(int(rate))
    snk.set_attenuation(0, float(atten))
    snk.set_filter_params("Auto", "", 0.0, 0.0)
    return snk


def silence_tx(sink: object) -> None:
    """Set TX attenuation to maximum — the quiet-state cleanup.

    Args:
        sink: The fmcomms2 sink block.
    """
    try:
        sink.set_attenuation(0, TX_ATTEN_SILENT_DB)  # type: ignore[attr-defined]
        print(f"[ok] -  TX attenuation set to {TX_ATTEN_SILENT_DB} dB (silenced)")
    except Exception as exc:  # noqa: BLE001 — cleanup must never raise
        eprint("WARN: could not silence TX attenuation:", exc)


def build_frame() -> bytes:
    """Build one frame: 8-byte GR access code + fixed-length ASCII payload.

    Returns:
        The frame as packed bytes (access code first), transmitted on repeat.
    """
    from gnuradio.digital import packet_utils

    code_bits = packet_utils.default_access_code  # 64-char '0'/'1' string
    code_bytes = int(code_bits, 2).to_bytes(len(code_bits) // 8, "big")
    payload = MESSAGE[:PAYLOAD_LEN].ljust(PAYLOAD_LEN, b".")
    return code_bytes + payload


def main() -> int:
    """Run the GMSK data loopback test end to end.

    Returns:
        0 on pass; 1 refused (no ``--transmit``); 2 import failure; 3 flowgraph
        build failure; 4 capture failure/timeout; 5 empty capture; 6 analysis
        FAIL (no frames, high BER, or RX saturated).
    """
    parser = argparse.ArgumentParser(description="Pluto GMSK data loopback test (TRANSMITS RF).")
    parser.add_argument(
        "--transmit",
        action="store_true",
        help="required acknowledgement that this run transmits RF",
    )
    parser.add_argument("--uri", default=None, help="IIO URI (default: auto, else ip:192.168.2.1)")
    parser.add_argument("--freq", type=float, default=915e6, help="LO frequency, TX and RX [Hz]")
    parser.add_argument("--rate", type=float, default=2.084e6, help="sample rate [Sa/s]")
    parser.add_argument("--sps", type=int, default=4, help="samples per symbol (GMSK)")
    parser.add_argument(
        "--amplitude", type=float, default=0.5, help="baseband amplitude, 0..1 full scale"
    )
    parser.add_argument(
        "--tx-atten", type=float, default=20.0, help="TX attenuation [dB] (0=max power)"
    )
    parser.add_argument("--rx-gain", type=float, default=30.0, help="manual RX gain [dB]")
    parser.add_argument("--nsamples", type=int, default=1 << 19, help="IQ samples to capture")
    parser.add_argument(
        "--settle", type=int, default=1 << 16, help="samples to discard before capture"
    )
    args = parser.parse_args()

    print("=" * 66)
    print(" ⚠  Pluto GMSK DATA LOOPBACK TEST — THIS RUN TRANSMITS RF  ⚠")
    print(" Precondition: TX -> 30 dB pads -> RX cabled on the flat-sat Pluto")
    print("=" * 66)

    if not args.transmit:
        eprint("REFUSED: --transmit not given. This script transmits RF and")
        eprint("requires explicit per-instance approval (PLAN.md §0).")
        return 1

    # ---- stage 1: imports -------------------------------------------------
    try:
        import numpy as np
        from gnuradio import blocks, digital, gr
    except Exception as exc:  # noqa: BLE001 — report any import failure as stage 1
        eprint("FAIL [1] import gnuradio/numpy:", exc)
        return 2
    print("[ok] 1  gnuradio + numpy import")

    uri = resolve_uri(args.uri)
    frame = build_frame()
    frame_bits = int(len(frame) * 8)
    baud = args.rate / args.sps
    print(
        f"[..] 2  uri={uri}  LO={args.freq / 1e6:.3f} MHz  "
        f"rate={args.rate / 1e6:.3f} MSa/s  GMSK sps={args.sps} "
        f"({baud / 1e3:.1f} kbaud)  frame={len(frame)} B  "
        f"tx_atten={args.tx_atten:g} dB  rx_gain={args.rx_gain:g} dB"
    )

    # ---- stage 2: build TX + RX flowgraph ---------------------------------
    class DataLoopback(gr.top_block):
        """Bytes -> GMSK mod -> TX; RX -> GMSK demod -> bits (plus raw IQ tap)."""

        def __init__(self) -> None:
            """Build both chains on one Pluto (TX transmits framed data)."""
            gr.top_block.__init__(self, "pluto_data_loopback")
            data = blocks.vector_source_b(list(frame), repeat=True)
            mod = digital.gmsk_mod(samples_per_symbol=args.sps, bt=0.35)
            scale = blocks.multiply_const_cc(args.amplitude)
            self.tx = make_tx_sink(uri, args.freq, args.rate, args.tx_atten)
            self.connect(data, mod, scale, self.tx)

            src = make_rx_source(uri, args.freq, args.rate, args.rx_gain)
            skip = blocks.skiphead(gr.sizeof_gr_complex, args.settle)
            head = blocks.head(gr.sizeof_gr_complex, args.nsamples)
            demod = digital.gmsk_demod(samples_per_symbol=args.sps)
            self.iq_sink = blocks.vector_sink_c()
            self.bit_sink = blocks.vector_sink_b()
            self.connect(src, skip, head, demod, self.bit_sink)
            self.connect(head, self.iq_sink)

    try:
        tb = DataLoopback()
    except Exception as exc:  # noqa: BLE001 — any build failure is stage 2
        eprint("FAIL [2] building data loopback flowgraph:", exc)
        return 3
    print("[ok] 2  TX+RX flowgraph built — transmission begins at start()")

    # ---- stage 3: transmit + capture ---------------------------------------
    try:
        tb.start()
        deadline = time.monotonic() + CAPTURE_TIMEOUT_S
        while len(tb.iq_sink.data()) < args.nsamples and time.monotonic() < deadline:
            time.sleep(0.2)
        tb.stop()
        tb.wait()
    except Exception as exc:  # noqa: BLE001 — any runtime failure is stage 3
        silence_tx(tb.tx)
        eprint("FAIL [3] during transmit/capture:", exc)
        return 4
    silence_tx(tb.tx)

    iq = np.asarray(tb.iq_sink.data()[: args.nsamples], dtype=np.complex64)
    bits = (np.asarray(tb.bit_sink.data(), dtype=np.uint8) & 1).astype(np.int8)
    if iq.size == 0 or bits.size == 0:
        eprint(f"FAIL [3] empty capture (iq={iq.size}, bits={bits.size})")
        return 5
    print(f"[ok] 3  captured {iq.size} IQ samples, demodulated {bits.size} bits")

    # ---- stage 4: frame recovery + BER --------------------------------------
    power_dbfs = float(10.0 * np.log10(float(np.mean(np.abs(iq) ** 2)) + 1e-20))
    peak_amp = float(np.max(np.abs(iq)))

    from gnuradio.digital import packet_utils

    code = np.array([int(c) for c in packet_utils.default_access_code], dtype=np.int8)
    payload_expected = np.unpackbits(np.frombuffer(build_frame()[8:], dtype=np.uint8)).astype(
        np.int8
    )
    payload_nbits = int(payload_expected.size)

    corr = np.correlate(2 * bits - 1, 2 * code - 1, mode="valid")
    hit_indices = np.flatnonzero(corr >= code.size - 2 * 2)  # allow <=2 code bit errors

    frames_found = 0
    frames_perfect = 0
    bit_errors = 0
    bits_compared = 0
    first_payload: bytes | None = None
    last_accepted = -frame_bits
    for h in hit_indices:
        if h < last_accepted + frame_bits:
            continue  # overlaps the previously accepted frame
        start = int(h) + code.size
        if start + payload_nbits > bits.size:
            break
        last_accepted = int(h)
        got = bits[start : start + payload_nbits]
        errs = int(np.count_nonzero(got != payload_expected))
        frames_found += 1
        frames_perfect += int(errs == 0)
        bit_errors += errs
        bits_compared += payload_nbits
        if first_payload is None:
            first_payload = np.packbits(got.astype(np.uint8)).tobytes()

    ber = bit_errors / bits_compared if bits_compared else 1.0
    recovered = first_payload.decode("ascii", errors="replace") if first_payload else "(none)"

    print("-" * 66)
    print(f"  mean power       : {power_dbfs:8.2f} dBFS")
    print(f"  peak |amp|       : {peak_amp:8.4f}   (~1.0 => railed, lower --rx-gain)")
    print(f"  frames found     : {frames_found}")
    print(f"  frames perfect   : {frames_perfect}")
    print(f"  payload BER      : {ber:.2e}  ({bit_errors}/{bits_compared} bits)")
    print(f"  first payload    : {recovered!r}")
    print("-" * 66)

    failures: list[str] = []
    if peak_amp >= 0.999:
        failures.append("RX saturated (peak at full scale) — lower --rx-gain or raise --tx-atten")
    if frames_found < 10:
        failures.append(f"only {frames_found} frames found (<10) — no usable data link")
    elif frames_perfect < 0.9 * frames_found:
        failures.append(f"only {frames_perfect}/{frames_found} frames perfect (<90%)")
    if ber > 1e-3:
        failures.append(f"payload BER {ber:.2e} > 1e-3")

    if failures:
        for f in failures:
            eprint("FAIL [4]", f)
        return 6

    print("PASS: real data framed, transmitted, received, and decoded through")
    print("      the pad loopback. (TX now silenced at max attenuation.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
