#!/usr/bin/env python3
"""SNR sweep over a captured Pluto loopback — TRANSMITS ONCE (unless --load).

==============================================================
 ⚠⚠⚠  THIS SCRIPT TRANSMITS ON THE PLUTO TX PORT  ⚠⚠⚠
     (single capture; the sweep itself is offline DSP)
==============================================================

Run policy (PLAN.md §0): explicit per-instance approval required to transmit.
Refuses to transmit without ``--transmit``. With ``--load FILE.npz`` no RF is
involved at all: the sweep reruns on a previously saved capture.

  * TX → 30 dB of SMA pads → RX physically cabled on the SAME Pluto.
  * RF levels identical to the passed data loopback test (constant-envelope
    GMSK, amplitude 0.5, 20 dB TX attenuation ≈ −49 dBm at RX).

What it does:
  1. Captures raw IQ of the framed GMSK signal (same waveform as
     pluto_data_loopback_test.py: 64-bit access code + 64-byte payload at
     LO+250 kHz) — the ONLY stage that transmits.
  2. Offline, adds complex AWGN to the SAME capture in calibrated stages and
     re-demodulates each (xlating LPF → gmsk_demod), so link quality is
     stepped programmatically while the underlying RF capture stays fixed.
  3. Reports one row per stage: target and measured in-band SNR, frames
     found, frames perfect, payload BER — the BER waterfall in table form.

In-band SNR is measured, not assumed: signal and injected-noise powers are
integrated over the occupied band [offset ± 0.75·baud] via FFT (Parseval),
so each row's SNR reflects what the demod actually saw.

Interpretation: the no-noise row must be BER 0 (link sanity — this is the
PASS criterion). As SNR steps down, expect BER ~0 until the GMSK waterfall
region (~6–12 dB in-band SNR for this noncoherent demod), then rapid
degradation and frame loss. Save the capture with ``--save`` and iterate
offline with ``--load`` — no further transmissions needed.

Usage:
  ./pluto_snr_sweep_test.py --transmit --save capture.npz
  ./pluto_snr_sweep_test.py --load capture.npz               # no RF
  ./pluto_snr_sweep_test.py --load capture.npz --snr-steps 12,10,9,8,7,6,5

Exit code 0 = pass; non-zero identifies the failed stage.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

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

    snk = iio.fmcomms2_sink_fc32(uri, [True, True], 0x10000, False)
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


def demod_offline(iq: np.ndarray, rate: float, offset: float, sps: int) -> np.ndarray:
    """Demodulate a captured/synthesized IQ block offline (no hardware).

    Chain: vector source -> frequency-xlating low-pass (offset -> baseband,
    leakage rejected) -> gmsk_demod -> bits. Runs to completion because the
    source is finite.

    Args:
        iq: Complex64 IQ samples at ``rate``, signal centered at ``offset``.
        rate: Sample rate in samples/second.
        offset: Signal offset from baseband zero in Hz.
        sps: GMSK samples per symbol.

    Returns:
        Demodulated hard bits as an int8 array of 0/1.
    """
    from gnuradio import blocks, digital, gr
    from gnuradio import filter as gr_filter
    from gnuradio.filter import firdes

    baud = rate / sps
    tb = gr.top_block("snr_sweep_demod")
    src = blocks.vector_source_c(iq.tolist(), repeat=False)
    taps = firdes.low_pass(1.0, rate, 0.75 * baud, 0.25 * baud)
    down = gr_filter.freq_xlating_fir_filter_ccf(1, taps, offset, rate)
    demod = digital.gmsk_demod(samples_per_symbol=sps)
    sink = blocks.vector_sink_b()
    tb.connect(src, down, demod, sink)
    tb.run()
    return (np.asarray(sink.data(), dtype=np.uint8) & 1).astype(np.int8)


def analyze_bits(bits: np.ndarray) -> tuple[int, int, int, int]:
    """Locate frames by access-code correlation and count payload bit errors.

    Args:
        bits: Demodulated hard bits (0/1) to search.

    Returns:
        Tuple of (frames_found, frames_perfect, bit_errors, bits_compared).
    """
    from gnuradio.digital import packet_utils

    frame = build_frame()
    frame_bits = len(frame) * 8
    code = np.array([int(c) for c in packet_utils.default_access_code], dtype=np.int8)
    expected = np.unpackbits(np.frombuffer(frame[8:], dtype=np.uint8)).astype(np.int8)
    corr = np.correlate(2 * bits - 1, 2 * code - 1, mode="valid")
    hits = np.flatnonzero(corr >= code.size - 2 * 2)  # allow <=2 code bit errors

    found = perfect = errors = compared = 0
    last = -frame_bits
    for h in hits:
        if h < last + frame_bits:
            continue
        start = int(h) + code.size
        if start + expected.size > bits.size:
            break
        last = int(h)
        errs = int(np.count_nonzero(bits[start : start + expected.size] != expected))
        found += 1
        perfect += int(errs == 0)
        errors += errs
        compared += expected.size
    return found, perfect, errors, compared


def inband_power(x: np.ndarray, rate: float, offset: float, half_bw: float) -> float:
    """Measure mean power of ``x`` inside [offset - half_bw, offset + half_bw].

    Uses FFT integration (Parseval), so no filtering is needed and the same
    band is used for signal and noise measurements.

    Args:
        x: Complex samples.
        rate: Sample rate in samples/second.
        offset: Band center in Hz.
        half_bw: Half of the band width in Hz.

    Returns:
        Mean in-band power (linear, per full-sequence normalization).
    """
    spec = np.fft.fft(x)
    freqs = np.fft.fftfreq(x.size, d=1.0 / rate)
    mask = np.abs(freqs - offset) <= half_bw
    return float(np.sum(np.abs(spec[mask]) ** 2) / (x.size**2))


def main() -> int:
    """Capture once (or load), then sweep added AWGN and report BER per step.

    Returns:
        0 on pass; 1 refused/bad args; 2 import failure; 3 flowgraph build
        failure; 4 capture failure; 5 empty capture; 6 no-noise row not clean.
    """
    parser = argparse.ArgumentParser(
        description="Pluto GMSK SNR sweep (one TX capture, offline noise ladder)."
    )
    parser.add_argument(
        "--transmit",
        action="store_true",
        help="required acknowledgement that the capture stage transmits RF",
    )
    parser.add_argument("--load", default=None, help="load capture .npz instead of transmitting")
    parser.add_argument("--save", default=None, help="save the fresh capture to this .npz path")
    parser.add_argument("--uri", default=None, help="IIO URI (default: auto, else ip:192.168.2.1)")
    parser.add_argument("--freq", type=float, default=915e6, help="LO frequency, TX and RX [Hz]")
    parser.add_argument("--rate", type=float, default=2.084e6, help="sample rate [Sa/s]")
    parser.add_argument("--sps", type=int, default=8, help="samples per symbol (GMSK)")
    parser.add_argument(
        "--offset",
        type=float,
        default=250e3,
        help="digital offset from LO [Hz] (keeps signal off DC)",
    )
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
    parser.add_argument(
        "--snr-steps",
        default="30,20,15,12,10,9,8,7,6,5,4,3,2,1,0",
        help="comma-separated target in-band SNRs [dB], swept high to low",
    )
    parser.add_argument("--seed", type=int, default=42, help="RNG seed (reproducible noise)")
    args = parser.parse_args()

    try:
        snr_steps = [float(s) for s in args.snr_steps.split(",") if s.strip()]
    except ValueError:
        eprint("REFUSED: --snr-steps must be comma-separated numbers")
        return 1

    baud = args.rate / args.sps
    half_bw = 0.75 * baud

    # ---- stage 1: obtain the IQ capture ------------------------------------
    if args.load:
        iq = np.load(args.load)["iq"].astype(np.complex64)
        print(f"[ok] 1  loaded {iq.size} IQ samples from {args.load} (no RF this run)")
    else:
        print("=" * 66)
        print(" ⚠  Pluto SNR SWEEP — THE CAPTURE STAGE TRANSMITS RF  ⚠")
        print(" Precondition: TX -> 30 dB pads -> RX cabled on the flat-sat Pluto")
        print("=" * 66)
        if not args.transmit:
            eprint("REFUSED: --transmit not given (or use --load FILE.npz for no-RF).")
            eprint("Transmitting requires explicit per-instance approval (PLAN.md §0).")
            return 1
        try:
            import math

            from gnuradio import blocks, digital, gr
        except Exception as exc:  # noqa: BLE001 — report any import failure as stage 1
            eprint("FAIL [1] import gnuradio:", exc)
            return 2

        uri = resolve_uri(args.uri)
        frame = build_frame()
        print(
            f"[..] 1  uri={uri}  LO={args.freq / 1e6:.3f} MHz  "
            f"signal at LO{args.offset / 1e3:+.0f} kHz  GMSK sps={args.sps} "
            f"({baud / 1e3:.1f} kbaud)  frame={len(frame)} B  "
            f"tx_atten={args.tx_atten:g} dB  rx_gain={args.rx_gain:g} dB"
        )

        class CaptureLoopback(gr.top_block):
            """Bytes -> GMSK -> rotate up -> TX; RX -> raw IQ sink (no demod)."""

            def __init__(self) -> None:
                """Build the capture flowgraph (TX transmits framed data)."""
                gr.top_block.__init__(self, "pluto_snr_sweep_capture")
                data = blocks.vector_source_b(list(frame), repeat=True)
                mod = digital.gmsk_mod(samples_per_symbol=args.sps, bt=0.35)
                scale = blocks.multiply_const_cc(args.amplitude)
                up = blocks.rotator_cc(2.0 * math.pi * args.offset / args.rate)
                self.tx = make_tx_sink(uri, args.freq, args.rate, args.tx_atten)
                self.connect(data, mod, scale, up, self.tx)

                src = make_rx_source(uri, args.freq, args.rate, args.rx_gain)
                skip = blocks.skiphead(gr.sizeof_gr_complex, args.settle)
                head = blocks.head(gr.sizeof_gr_complex, args.nsamples)
                self.iq_sink = blocks.vector_sink_c()
                self.connect(src, skip, head, self.iq_sink)

        try:
            tb = CaptureLoopback()
        except Exception as exc:  # noqa: BLE001 — any build failure is stage 1
            eprint("FAIL [1] building capture flowgraph:", exc)
            return 3
        try:
            tb.start()
            deadline = time.monotonic() + CAPTURE_TIMEOUT_S
            while len(tb.iq_sink.data()) < args.nsamples and time.monotonic() < deadline:
                time.sleep(0.2)
            tb.stop()
            tb.wait()
        except Exception as exc:  # noqa: BLE001 — any runtime failure is stage 1
            silence_tx(tb.tx)
            eprint("FAIL [1] during transmit/capture:", exc)
            return 4
        silence_tx(tb.tx)
        iq = np.asarray(tb.iq_sink.data()[: args.nsamples], dtype=np.complex64)
        if iq.size == 0:
            eprint("FAIL [1] captured 0 samples")
            return 5
        print(f"[ok] 1  captured {iq.size} IQ samples (TX silenced)")
        if args.save:
            np.savez(args.save, iq=iq)
            print(f"[ok] -  capture saved to {args.save} (re-sweep offline with --load)")

    # ---- stage 2: noise ladder ----------------------------------------------
    p_sig = inband_power(iq, args.rate, args.offset, half_bw)
    inband_fraction = 2.0 * half_bw / args.rate
    rng = np.random.default_rng(args.seed)
    print(
        f"[..] 2  in-band signal power {10 * np.log10(p_sig + 1e-30):.2f} dBFS "
        f"over {2 * half_bw / 1e3:.0f} kHz; sweeping {len(snr_steps)} noise steps"
    )
    print("-" * 66)
    print("  target SNR   measured SNR   frames   perfect      payload BER")
    print("-" * 66)

    clean_ok = False
    rows: list[tuple[float, float, int, int, float]] = []
    for i, snr_db in enumerate([float("inf"), *snr_steps]):
        if np.isinf(snr_db):
            noisy = iq
            measured = float("inf")
        else:
            sigma2_fullband = p_sig / (10.0 ** (snr_db / 10.0)) / inband_fraction
            noise = rng.standard_normal(iq.size) + 1j * rng.standard_normal(iq.size)
            noise = (noise * np.sqrt(sigma2_fullband / 2.0)).astype(np.complex64)
            p_noise = inband_power(noise, args.rate, args.offset, half_bw)
            measured = float(10.0 * np.log10(p_sig / (p_noise + 1e-30)))
            noisy = (iq + noise).astype(np.complex64)
        bits = demod_offline(noisy, args.rate, args.offset, args.sps)
        found, perfect, errors, compared = analyze_bits(bits)
        ber = errors / compared if compared else 1.0
        rows.append((snr_db, measured, found, perfect, ber))
        label = "  clean " if np.isinf(snr_db) else f"{snr_db:7.1f} "
        mlabel = "   --  " if np.isinf(measured) else f"{measured:7.2f}"
        print(f"  {label}dB   {mlabel} dB   {found:6d}   {perfect:7d}      {ber:.2e}")
        if i == 0:
            clean_ok = found >= 10 and errors == 0

    print("-" * 66)
    if not clean_ok:
        eprint("FAIL [2] the no-noise row is not clean — link itself is degraded;")
        eprint("         fix the link before interpreting the sweep.")
        return 6
    print("PASS: clean row error-free; rows above are the BER-vs-SNR staircase.")
    if not args.load:
        print("      (No further RF needed: rerun offline with --load.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
