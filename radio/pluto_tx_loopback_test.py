#!/usr/bin/env python3
"""TX→RX loopback tone test for the ADALM-Pluto — THIS SCRIPT TRANSMITS RF.

==============================================================
 ⚠⚠⚠  THIS SCRIPT TRANSMITS ON THE PLUTO TX PORT  ⚠⚠⚠
==============================================================

Run policy (PLAN.md §0): explicit per-instance approval required. Refuses to
run without the ``--transmit`` flag. Preconditions, verified by the operator:

  * TX → 30 dB of SMA pads → RX physically cabled on the SAME Pluto.
  * Never TX→RX without the pads inline. Cabled only — no antennas.

Link budget at the defaults (survey figures + on-device verification, not
calibrated numbers): Pluto full-scale TX ≈ +7 dBm at 915 MHz. Amplitude 0.5
(−6 dB) and 20 dB TX attenuation → ≈ −19 dBm at the TX port → 30 dB pads →
≈ −49 dBm at RX. At 30 dB RX gain that lands the tone comfortably above the
measured noise floor (RX-only baseline: −65.7 dBFS at 40 dB gain) and far
below saturation. Worst case (amplitude 1.0, 0 dB attenuation) is ≈ −23 dBm
at RX — still safe for the AD936x front end, so no parameter choice here can
damage the hardware while the pads are inline.

What it proves (P3 precursor):
  1. The TX chain actually radiates a known signal (cos tone at ``--tone-offset``
     from the LO).
  2. The RX chain receives it through the pads: the dominant FFT bin lands at
     the commanded offset — a real, physical measurement, not a numerical
     artifact (the RX-only smoke test saw only noise floor here).
  3. Tone-above-floor margin is quantified and PASS/FAIL is automatic.

Quiet state: on every exit path the TX attenuation is slammed to max
(89.75 dB) before the process ends, so the transmitter is left silenced even
if a stale DMA buffer keeps cycling in hardware.

Usage (requires explicit flag):
  ./pluto_tx_loopback_test.py --transmit
  ./pluto_tx_loopback_test.py --transmit --tx-atten 30 --rx-gain 20

Exit code 0 = pass; non-zero identifies the failed stage.
"""

from __future__ import annotations

import argparse
import sys
import time

FLATSAT_PLUTO_SERIAL = "104473b04a06000602001c00dd1f84cfaa"  # role: flat-sat radio
TX_ATTEN_SILENT_DB = 89.75  # max AD936x TX attenuation — the "off" quiet state
CAPTURE_TIMEOUT_S = 30.0


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

    Verified in-tree gr-iio 3.10 API (introspected on-device):
    ``fmcomms2_sink_fc32(uri, ch_en, buffer_size, cyclic)`` + setters.

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

    Called on every exit path after the sink exists, so the transmitter ends
    silenced even if hardware keeps cycling a stale DMA buffer.

    Args:
        sink: The fmcomms2 sink block, or None-like if construction failed.
    """
    try:
        sink.set_attenuation(0, TX_ATTEN_SILENT_DB)  # type: ignore[attr-defined]
        print(f"[ok] -  TX attenuation set to {TX_ATTEN_SILENT_DB} dB (silenced)")
    except Exception as exc:  # noqa: BLE001 — cleanup must never raise
        eprint("WARN: could not silence TX attenuation:", exc)


def main() -> int:
    """Run the TX→RX loopback tone test end to end.

    Returns:
        0 on pass; 1 refused (no ``--transmit``); 2 import failure; 3 flowgraph
        build failure; 4 capture failure/timeout; 5 empty capture; 6 analysis
        FAIL (tone absent, off-frequency, or RX saturated).
    """
    parser = argparse.ArgumentParser(description="Pluto TX->RX loopback tone test (TRANSMITS RF).")
    parser.add_argument(
        "--transmit",
        action="store_true",
        help="required acknowledgement that this run transmits RF",
    )
    parser.add_argument("--uri", default=None, help="IIO URI (default: auto, else ip:192.168.2.1)")
    parser.add_argument("--freq", type=float, default=915e6, help="LO frequency, TX and RX [Hz]")
    parser.add_argument("--rate", type=float, default=2.084e6, help="sample rate [Sa/s]")
    parser.add_argument("--tone-offset", type=float, default=100e3, help="tone offset from LO [Hz]")
    parser.add_argument(
        "--amplitude", type=float, default=0.5, help="tone amplitude, 0..1 full scale"
    )
    parser.add_argument(
        "--tx-atten", type=float, default=20.0, help="TX attenuation [dB] (0=max power)"
    )
    parser.add_argument("--rx-gain", type=float, default=30.0, help="manual RX gain [dB]")
    parser.add_argument("--nsamples", type=int, default=1 << 18, help="IQ samples to capture")
    parser.add_argument(
        "--settle", type=int, default=1 << 16, help="samples to discard before capture"
    )
    args = parser.parse_args()

    print("=" * 66)
    print(" ⚠  Pluto TX->RX LOOPBACK TEST — THIS RUN TRANSMITS RF  ⚠")
    print(" Precondition: TX -> 30 dB pads -> RX cabled on the flat-sat Pluto")
    print("=" * 66)

    if not args.transmit:
        eprint("REFUSED: --transmit not given. This script transmits RF and")
        eprint("requires explicit per-instance approval (PLAN.md §0).")
        return 1

    # ---- stage 1: imports -------------------------------------------------
    try:
        import numpy as np
        from gnuradio import analog, blocks, gr
    except Exception as exc:  # noqa: BLE001 — report any import failure as stage 1
        eprint("FAIL [1] import gnuradio/numpy:", exc)
        return 2
    print("[ok] 1  gnuradio + numpy import")

    uri = resolve_uri(args.uri)
    print(
        f"[..] 2  uri={uri}  LO={args.freq / 1e6:.3f} MHz  "
        f"tone=LO{args.tone_offset / 1e3:+.1f} kHz  "
        f"rate={args.rate / 1e6:.3f} MSa/s  amp={args.amplitude:g}  "
        f"tx_atten={args.tx_atten:g} dB  rx_gain={args.rx_gain:g} dB"
    )

    # ---- stage 2: build TX + RX flowgraph ---------------------------------
    class Loopback(gr.top_block):
        """Tone -> fmcomms2 sink (TX); fmcomms2 source -> skiphead -> head -> sink (RX)."""

        def __init__(self) -> None:
            """Build both chains on one Pluto (TX transmits the tone)."""
            gr.top_block.__init__(self, "pluto_tx_loopback")
            tone = analog.sig_source_c(
                args.rate, analog.GR_COS_WAVE, args.tone_offset, args.amplitude
            )
            self.tx = make_tx_sink(uri, args.freq, args.rate, args.tx_atten)
            self.connect(tone, self.tx)

            src = make_rx_source(uri, args.freq, args.rate, args.rx_gain)
            skip = blocks.skiphead(gr.sizeof_gr_complex, args.settle)
            head = blocks.head(gr.sizeof_gr_complex, args.nsamples)
            self.rx_sink = blocks.vector_sink_c()
            self.connect(src, skip, head, self.rx_sink)

    try:
        tb = Loopback()
    except Exception as exc:  # noqa: BLE001 — any build failure is stage 2
        eprint("FAIL [2] building loopback flowgraph:", exc)
        return 3
    print("[ok] 2  TX+RX flowgraph built — transmission begins at start()")

    # ---- stage 3: transmit + capture ---------------------------------------
    try:
        tb.start()
        deadline = time.monotonic() + CAPTURE_TIMEOUT_S
        while len(tb.rx_sink.data()) < args.nsamples and time.monotonic() < deadline:
            time.sleep(0.2)
        tb.stop()
        tb.wait()
    except Exception as exc:  # noqa: BLE001 — any runtime failure is stage 3
        silence_tx(tb.tx)
        eprint("FAIL [3] during transmit/capture:", exc)
        return 4
    silence_tx(tb.tx)

    data = np.asarray(tb.rx_sink.data()[: args.nsamples], dtype=np.complex64)
    if data.size == 0:
        eprint("FAIL [3] captured 0 samples (timeout?)")
        return 5
    print(f"[ok] 3  captured {data.size} IQ samples through the 30 dB loopback")

    # ---- stage 4: tone analysis --------------------------------------------
    power_dbfs = float(10.0 * np.log10(float(np.mean(np.abs(data) ** 2)) + 1e-20))
    peak_amp = float(np.max(np.abs(data)))
    window = np.hanning(data.size)
    psd = np.abs(np.fft.fftshift(np.fft.fft(data * window))) ** 2
    fft_freqs = np.fft.fftshift(np.fft.fftfreq(data.size, d=1.0 / args.rate))
    peak_bin = int(np.argmax(psd))
    found_offset = float(fft_freqs[peak_bin])

    guard = np.abs(fft_freqs - found_offset) > 10e3
    guard &= np.abs(fft_freqs) > 5e3  # exclude residual LO leakage at DC
    floor_med = float(np.median(psd[guard]))
    tone_above_floor_db = float(10.0 * np.log10(psd[peak_bin] / (floor_med + 1e-30)))
    offset_err_hz = found_offset - args.tone_offset

    print("-" * 66)
    print(f"  mean power       : {power_dbfs:8.2f} dBFS   (RX-only baseline was ~-66)")
    print(f"  peak |amp|       : {peak_amp:8.4f}   (~1.0 => railed, lower --rx-gain)")
    print(f"  dominant bin     : {found_offset / 1e3:+9.2f} kHz from LO")
    print(f"  commanded tone   : {args.tone_offset / 1e3:+9.2f} kHz from LO")
    print(f"  offset error     : {offset_err_hz / 1e3:+9.3f} kHz")
    print(f"  tone above floor : {tone_above_floor_db:8.1f} dB (median floor)")
    print("-" * 66)

    bin_hz = args.rate / data.size
    tol_hz = max(5e3, 3 * bin_hz)
    failures: list[str] = []
    if peak_amp >= 0.999:
        failures.append("RX saturated (peak at full scale) — lower --rx-gain or raise --tx-atten")
    if abs(offset_err_hz) > tol_hz:
        failures.append(
            f"dominant tone at {found_offset / 1e3:+.1f} kHz, expected "
            f"{args.tone_offset / 1e3:+.1f} kHz (tol ±{tol_hz / 1e3:.1f} kHz)"
        )
    if tone_above_floor_db < 20.0:
        failures.append(f"tone only {tone_above_floor_db:.1f} dB above floor (<20 dB)")

    if failures:
        for f in failures:
            eprint("FAIL [4]", f)
        return 6

    print("PASS: commanded tone received through the pad loopback — TX and RX")
    print("      paths are both real. (TX now silenced at max attenuation.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
