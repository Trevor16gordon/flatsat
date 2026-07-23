#!/usr/bin/env python3
"""RX-only smoke test for the ADALM-Pluto via GNU Radio (gr-iio).

SAFETY: THIS SCRIPT NEVER TRANSMITS.
It builds a receive-only flowgraph using gr-iio's ``fmcomms2_source``. No
``fmcomms2_sink`` / TX block is ever instantiated, so no RF leaves the Pluto's
TX port. The transmit test lives in a separate script and runs only after this
passes and the 30 dB pads are confirmed inline.

What it validates (bring-up P1):
  1. GNU Radio + gr-iio import cleanly.
  2. A Pluto URI resolves (auto-detect, or ``--uri``).
  3. An RX flowgraph builds and streams real IQ samples off the device.
  4. The captured block looks sane: sample count, mean power (dBFS), peak
     amplitude, DC offset, dominant FFT bin — a fast "is the front end alive
     and not railed/dead" read.

With the TX port silent and pads looped to RX, expect noise floor only
(very low power, no dominant tone) — that is a PASS.

Usage:
  ./pluto_smoke_test.py                       # auto-detect, defaults
  ./pluto_smoke_test.py --uri ip:192.168.2.1
  ./pluto_smoke_test.py --freq 915e6 --rate 2.084e6 --gain 40

Exit code 0 = pass; non-zero identifies the failed stage.
"""

from __future__ import annotations

import argparse
import sys

FLATSAT_PLUTO_SERIAL = "104473b04a06000602001c00dd1f84cfaa"  # role: flat-sat radio


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


def make_rx_source(uri: str, freq: float, rate: float, bw: float, gain: float) -> object:
    """Construct an RX-only fmcomms2 source block (no TX block anywhere).

    The positional argument order matches gr-iio 3.10.1.1's
    ``fmcomms2_source_fc32``. If this install differs, construction raises
    TypeError and the caller prints the real signature via
    :func:`diag_signature` — without ever transmitting.

    Args:
        uri: IIO context URI (e.g. ``ip:192.168.2.1`` or ``usb:1.4.5``).
        freq: RX LO frequency in Hz.
        rate: Sample rate in samples/second.
        bw: RF bandwidth in Hz.
        gain: Manual RX gain in dB.

    Returns:
        The constructed gr-iio source block (opaque C++ binding, typed as
        ``object`` — the ANN401 ban on ``Any`` stands even here).
    """
    from gnuradio import iio

    return iio.fmcomms2_source_fc32(
        uri,  # IIO context URI
        int(freq),  # RX LO frequency [Hz]
        int(rate),  # sample rate [Sa/s]
        int(bw),  # RF bandwidth [Hz]
        True,  # ch1 (I) enabled -> RX1
        True,  # ch2 (Q) enabled -> RX1
        False,  # ch3 disabled (Pluto is 1x1)
        False,  # ch4 disabled
        0x8000,  # kernel buffer size [samples]
        True,  # quadrature tracking
        True,  # RF DC offset correction
        True,  # baseband DC offset correction
        "manual",  # RX1 gain mode
        float(gain),  # RX1 gain value [dB]
        "manual",  # RX2 gain mode (unused on Pluto)
        float(gain),  # RX2 gain value (unused on Pluto)
        "A_BALANCED",  # RF port select
    )


def diag_signature() -> None:
    """Print the fmcomms2_source_fc32 signature installed on this system."""
    try:
        from gnuradio import iio

        eprint("\n--- fmcomms2_source_fc32 signature on THIS system ---")
        help(iio.fmcomms2_source_fc32)
    except Exception as exc:  # noqa: BLE001 — diagnostics must never raise
        eprint("could not introspect:", exc)


def main() -> int:
    """Run the RX-only smoke test end to end.

    Returns:
        0 on pass; 2 import failure; 3 flowgraph build failure; 4 capture
        failure; 5 empty capture.
    """
    parser = argparse.ArgumentParser(description="RX-only Pluto smoke test (no transmit).")
    parser.add_argument("--uri", default=None, help="IIO URI (default: auto, else ip:192.168.2.1)")
    parser.add_argument("--freq", type=float, default=915e6, help="RX LO frequency [Hz]")
    parser.add_argument("--rate", type=float, default=2.084e6, help="sample rate [Sa/s]")
    parser.add_argument("--bw", type=float, default=None, help="RF bandwidth [Hz] (default: rate)")
    parser.add_argument("--gain", type=float, default=40.0, help="manual RX gain [dB]")
    parser.add_argument("--nsamples", type=int, default=1 << 18, help="IQ samples to capture")
    args = parser.parse_args()
    bw: float = args.bw if args.bw else args.rate

    print("=" * 66)
    print(" Pluto RX-only smoke test — NO TRANSMIT (receive path only)")
    print("=" * 66)

    # ---- stage 1: imports -------------------------------------------------
    try:
        import numpy as np
        from gnuradio import blocks, gr
    except Exception as exc:  # noqa: BLE001 — report any import failure as stage 1
        eprint("FAIL [1] import gnuradio/numpy:", exc)
        return 2
    print("[ok] 1  gnuradio + numpy import")

    uri = resolve_uri(args.uri)
    print(
        f"[..] 2  uri={uri}  freq={args.freq / 1e6:.3f} MHz  "
        f"rate={args.rate / 1e6:.3f} MSa/s  bw={bw / 1e6:.3f} MHz  "
        f"gain={args.gain:g} dB  nsamples={args.nsamples}"
    )

    # ---- stage 2: build RX-only flowgraph ---------------------------------
    class RxProbe(gr.top_block):
        """Minimal RX flowgraph: fmcomms2 source -> head -> vector sink."""

        def __init__(self) -> None:
            """Build the source→head→sink chain (RX only, no TX block)."""
            gr.top_block.__init__(self, "pluto_rx_smoke")
            src = make_rx_source(uri, args.freq, args.rate, bw, args.gain)
            head = blocks.head(gr.sizeof_gr_complex, args.nsamples)
            self.sink = blocks.vector_sink_c()
            self.connect(src, head, self.sink)

    try:
        tb = RxProbe()
    except TypeError as exc:
        eprint("FAIL [2] fmcomms2_source_fc32 constructor mismatch:", exc)
        diag_signature()
        return 3
    except Exception as exc:  # noqa: BLE001 — any build failure is stage 2
        eprint("FAIL [2] building RX flowgraph:", exc)
        return 3
    print("[ok] 2  RX flowgraph built (fmcomms2_source only — no sink/TX)")

    # ---- stage 3: run the capture -----------------------------------------
    try:
        tb.run()
    except Exception as exc:  # noqa: BLE001 — any runtime failure is stage 3
        eprint("FAIL [3] during capture (device busy? wrong URI?):", exc)
        return 4
    data = np.asarray(tb.sink.data(), dtype=np.complex64)
    if data.size == 0:
        eprint("FAIL [3] captured 0 samples")
        return 5
    print(f"[ok] 3  captured {data.size} IQ samples off the device")

    # ---- stage 4: quick front-end health metrics --------------------------
    power_lin = float(np.mean(np.abs(data) ** 2))
    power_dbfs = float(10.0 * np.log10(power_lin + 1e-20))
    peak = float(np.max(np.abs(data)))
    dc = complex(np.mean(data))
    window = np.hanning(data.size)
    spectrum = np.fft.fftshift(np.fft.fft(data * window))
    fft_freqs = np.fft.fftshift(np.fft.fftfreq(data.size, d=1.0 / args.rate))
    peak_bin = int(np.argmax(np.abs(spectrum)))

    print("-" * 66)
    print(f"  mean power    : {power_dbfs:8.2f} dBFS")
    print(f"  peak |amp|    : {peak:8.4f}   (full scale ~1.0; ~1.0 => railed)")
    print(f"  DC offset     : {abs(dc):8.4f}   (I={dc.real:+.4f} Q={dc.imag:+.4f})")
    print(f"  dominant bin  : {float(fft_freqs[peak_bin]) / 1e3:+8.1f} kHz from LO")
    print("-" * 66)

    if peak >= 0.999:
        print("  ! peak at full scale — RX likely saturated; lower --gain")
    if power_dbfs < -90:
        print("  ! very low power — expected with TX silent; front end is streaming")

    print("PASS: receive path is alive and streaming. (No RF was transmitted.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
