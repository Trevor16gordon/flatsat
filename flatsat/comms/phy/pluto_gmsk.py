"""ADALM-Pluto GMSK modem — the classical baseline PHY, as a real driver.

The radio and settings that produced BER 0.00 on 113/113 framed packets
through 30 dB of cabled attenuation (2026-07-23), graduated from
``radio/pluto_data_loopback_test.py`` into a streaming modem:

    send()    bytes -> preamble -> GMSK mod -> rotate off DC -> fmcomms2 sink
    receive() fmcomms2 source -> freq-xlating LPF -> GMSK demod -> bits
              -> ASM bit-sync -> byte-aligned blocks

Three design points, each learned the hard way:

  * **Keep signal energy off DC.** Zero-IF LO leakage and the AD936x
    DC-correction loops sit at baseband centre; a signal placed there
    measured BER 1.8e-1 versus 0.00 offset-tuned. The modulator output
    is rotated up by ``offset_hz`` and translated back down on receive.
  * **The carrier never stops.** A GMSK demodulator's clock recovery
    holds lock on symbol transitions; take the carrier away between
    frames and every frame must re-acquire from its own preamble, on a
    transmit DMA that has meanwhile run dry. A dedicated feeder thread
    therefore owns the transmit pipe and writes ``IDLE_BYTE`` whenever
    no frame is queued, so the modulator runs continuously from the
    moment the radio opens until ``close()``. Its blocking writes are
    also the flow control: the pipe drains at exactly the modulator's
    rate, so the feeder is paced by the radio rather than by a clock.
    Consequently an *open* transmit chain radiates continuously —
    going quiet is ``silence_tx()``/``close()``, not idleness.
  * **Frame synchronization is a PHY function.** GMSK demodulation
    yields a bit stream with arbitrary phase, so this driver hunts the
    attached sync marker at BIT level and emits BYTE-ALIGNED blocks from
    that offset. The framer above then does length and CRC as usual —
    identical framing over any PHY, which is the point of the layering.

RF SAFETY, structurally enforced:
  * ``send()`` refuses and returns a COMM flag unless the configuration
    carries ``transmit_ack: true`` (FSW-RADIO-001). The TX chain is not
    even built without it, so an unacknowledged configuration cannot
    radiate even by accident.
  * The transmitter is driven to maximum attenuation on close, on error,
    and on every exit path (FSW-RADIO-002): the Pluto's DMA can replay a
    stale buffer after a flowgraph stops, so leaving must leave it quiet.
  * Receiving is always permitted; listening radiates nothing.

**The two pipes carry different things, and it is not a bug.** GNU
Radio's GMSK blocks are asymmetric: ``gmsk_mod`` consumes PACKED bytes
(8 symbols per byte, unpacking internally), while ``gmsk_demod`` emits
UNPACKED bits — one bit per byte, valued 0 or 1. So at 250 kbit/s the
transmit pipe carries ~31 kB/s and the receive pipe ~250 kB/s of the
same information, the receive buffer is sized eight times larger for
it, and ``len(block)`` on the receive side counts BITS despite being a
byte count. Every apparent factor-of-eight discrepancy below traces
back to this one asymmetry.

No Python-defined GNU Radio blocks appear here, deliberately: on this
JetPack build of GNU Radio 3.10.1.1, a *minimal* ``gr.sync_block``
subclass segfaults the interpreter (reproduced under both the venv and
system Python with identical numpy 1.21.5). Bytes therefore cross into
and out of the flowgraph through OS pipes read by stock C++
``file_descriptor`` blocks — which is also the more robust design: no
Python on the sample path at all.

GNU Radio imports live inside methods, so this module imports on any
machine — a flight computer or CI box without gr-iio must still load
the registry.
"""

from __future__ import annotations

import fcntl
import math
import os
import threading
from collections import deque
from typing import Any

from flatsat.comms import comms_config_pb2
from flatsat.comms.modem import Modem
from flatsat.msgs import hal_pb2

DEFAULT_OFFSET_HZ = 250e3  # off DC by construction
DEFAULT_SPS = 8
DEFAULT_SYNC_HEX = "1acffc1d"  # the CCSDS attached sync marker (matches CcsdsFramer)
MAX_ATTENUATION_DB = 89.75  # the Pluto's floor: "as quiet as this radio gets"
PREAMBLE = (
    bytes([0x55]) * 16
)  # alternating bits: gives the demod's clock recovery something to lock
IDLE_BYTE = 0x55  # keeps the modulator running between frames
# Byte -> its low bit, as a translation table. The receive path masks
# ~260 kB/s, and a Python-level generator over that holds the GIL long
# enough to starve the transmit feeder into underruns; bytes.translate
# does the same work in C.
_LOW_BIT = bytes(i & 1 for i in range(256))
_MAX_SYNC_BIT_ERRORS = 2
_RX_BIT_BUFFER_LIMIT = 1 << 22  # ~4 Mbit: bound memory if nothing ever syncs
# Idle written per feeder pass. This is the latency grain: a frame queued
# just after a pass waits for this much idle to drain first — 256 packed
# bytes is ~8 ms at 250 kbit/s. Smaller means more syscalls for no gain.
_IDLE_CHUNK_BYTES = 256
# SMALL on purpose: pipe depth IS transmit latency, since the feeder
# fills whatever space exists with idle. 4 KiB is one page (the F_SETPIPE_SZ
# floor) and ~130 ms of air at 250 kbit/s.
_TX_PIPE_BYTES = 4096
_TX_QUEUE_LIMIT = 64  # frames awaiting the modulator before send() pushes back
_GMSK_BT = 0.35  # Gaussian bandwidth-time product; the bench's measured value
# LINK BUDGET — these two are MEASURED, not chosen. Together with 30 dB of
# cabled pads they are the 2026-07-23 operating point that produced BER 0.00:
# amplitude 0.5 at 20 dB attenuation is about -19 dBm at the TX port and
# -49 dBm at RX, which the pad budget in radio/pluto_tx_loopback_test.py
# derives as worst-case safe.
#
# Do not "harden" these by raising attenuation. A first attempt at defaults
# used 0.3/60 dB on the theory that quieter is safer; that is 44 dB below
# the proven point, and it put the receiver in the noise -- bits demodulated
# at the correct rate, zero sync correlations, a whole bench session lost.
# Pad safety comes from the 30 dB of pads, not from starving the modulator.
_TX_BACKOFF = 0.5
_DEFAULT_TX_ATTEN_DB = 20.0


def _sync_bits(sync_hex: str) -> list[int]:
    """Expand a hex sync marker into its bit pattern.

    Args:
        sync_hex: Marker as hex, e.g. ``1acffc1d``.

    Returns:
        The bits, most significant first.
    """
    raw = bytes.fromhex(sync_hex)
    return [(byte >> shift) & 1 for byte in raw for shift in range(7, -1, -1)]


class PlutoGmskModem(Modem):
    """GMSK over an ADALM-Pluto, transmit-gated by explicit acknowledgement."""

    def __init__(
        self,
        name: str,
        uri: str,
        center_freq_hz: float,
        sample_rate_hz: float,
        offset_hz: float = DEFAULT_OFFSET_HZ,
        samples_per_symbol: int = DEFAULT_SPS,
        tx_attenuation_db: float = _DEFAULT_TX_ATTEN_DB,
        rx_gain_db: float = 40.0,
        amplitude: float = _TX_BACKOFF,
        sync_hex: str = DEFAULT_SYNC_HEX,
        transmit_ack: bool = False,
    ) -> None:
        """Configure the radio without opening it.

        Args:
            name: Instance name.
            uri: IIO context URI, e.g. ``ip:192.168.2.1``.
            center_freq_hz: LO frequency.
            sample_rate_hz: Complex sample rate.
            offset_hz: Baseband offset keeping energy off DC.
            samples_per_symbol: GMSK oversampling.
            tx_attenuation_db: Transmit attenuation (higher = quieter).
            rx_gain_db: Manual receive gain.
            amplitude: Modulator output scaling before the sink.
            sync_hex: Attached sync marker to hunt at bit level; must
                match the framer's marker above.
            transmit_ack: Explicit permission to radiate. False means
                receive-only: the TX chain is never even built.
        """
        self._name = name
        self._uri = uri
        self._center_freq_hz = center_freq_hz
        self._sample_rate_hz = sample_rate_hz
        self._offset_hz = offset_hz
        self._sps = samples_per_symbol
        self._tx_attenuation_db = tx_attenuation_db
        self._rx_gain_db = rx_gain_db
        self._amplitude = amplitude
        self._sync_bits = _sync_bits(sync_hex)
        self._sync_hex = sync_hex
        self._transmit_ack = transmit_ack

        self._tx_lock = threading.Lock()
        self._tx_queue: deque[bytes] = deque()
        self._tx_thread: threading.Thread | None = None
        self._tx_running = False
        self._rx_bits = bytearray()
        self._rx_lock = threading.Lock()
        self._tx_write_fd: int | None = None
        self._tx_read_fd: int | None = None
        self._rx_read_fd: int | None = None
        self._pipe_fds: list[int] = []
        self._flowgraph: Any | None = None
        self._tx_sink: Any | None = None
        self._start_error: str = ""
        self._refused = 0
        self._dropped_sends = 0
        self._rx_bits_seen = 0
        self._sync_detections = 0
        self._locked = False  # byte alignment acquired; see _recover_blocks
        self._resyncs = 0

    @classmethod
    def from_config(
        cls, name: str, options: comms_config_pb2.PlutoGmskModemOptions
    ) -> PlutoGmskModem:
        """Build from vehicle-file modem options.

        Args:
            name: Instance name.
            options: Typed options; ``uri``, ``center_freq_hz`` and
                ``sample_rate_hz`` are required.

        Returns:
            The configured modem — receive-only unless the config
            explicitly acknowledges transmission.

        Raises:
            ValueError: If a required radio parameter is missing.
        """
        if not options.uri or options.center_freq_hz <= 0 or options.sample_rate_hz <= 0:
            raise ValueError(
                f"modem {name!r}: pluto_gmsk requires uri, center_freq_hz, sample_rate_hz"
            )
        return cls(
            name=name,
            uri=options.uri,
            center_freq_hz=options.center_freq_hz,
            sample_rate_hz=options.sample_rate_hz,
            offset_hz=options.offset_hz if options.HasField("offset_hz") else DEFAULT_OFFSET_HZ,
            samples_per_symbol=(
                options.samples_per_symbol
                if options.HasField("samples_per_symbol")
                else DEFAULT_SPS
            ),
            tx_attenuation_db=(
                float(options.tx_attenuation_db)
                if options.HasField("tx_attenuation_db")
                else _DEFAULT_TX_ATTEN_DB
            ),
            transmit_ack=options.transmit_ack,
        )

    # ------------------------------------------------------------ lifecycle --

    @property
    def may_transmit(self) -> bool:
        """Whether this modem is permitted to radiate."""
        return self._transmit_ack

    @property
    def refused_transmissions(self) -> int:
        """Frames declined because transmission was not acknowledged."""
        return self._refused

    @property
    def start_error(self) -> str:
        """Why the radio failed to open, if it did."""
        return self._start_error

    @property
    def dropped_sends(self) -> int:
        """Frames declined because the transmit queue was already full."""
        return self._dropped_sends

    @property
    def rx_bits_demodulated(self) -> int:
        """Bits the demodulator has produced since the radio opened.

        Zero means no signal path at all — the radio never delivered
        samples. Non-zero with no ``sync_detections`` means the opposite
        problem: bits are flowing but nothing locks. A failed bench run
        needs to tell those two apart.
        """
        return self._rx_bits_seen

    @property
    def sync_detections(self) -> int:
        """Sync-marker correlations found in the demodulated bit stream."""
        return self._sync_detections

    @property
    def resyncs(self) -> int:
        """Times byte alignment was re-acquired after a demodulator slip."""
        return self._resyncs

    def _build(self) -> Any:  # noqa: ANN401 — an opaque GNU Radio top_block
        """Construct the flowgraph: RX always, TX only when acknowledged.

        Returns:
            The started top block.

        Raises:
            Exception: Any GNU Radio or gr-iio failure; callers convert
                it to a COMM flag rather than propagating.
        """
        from gnuradio import blocks, digital, gr
        from gnuradio import filter as gr_filter
        from gnuradio.filter import firdes

        baud = self._sample_rate_hz / self._sps

        top = gr.top_block("flatsat_pluto_gmsk")

        # ---- receive chain (always built: listening radiates nothing) ----
        from gnuradio import iio

        source = iio.fmcomms2_source_fc32(self._uri, [True, True], 0x8000)
        # An empty length-tag key selects gr-iio's STREAMING mode. With a
        # key set, the block expects tagged bursts and will stall waiting
        # for packet boundaries this driver never produces.
        source.set_len_tag_key("")
        source.set_frequency(int(self._center_freq_hz))
        source.set_samplerate(int(self._sample_rate_hz))
        source.set_gain_mode(0, "manual")
        source.set_gain(0, float(self._rx_gain_db))
        source.set_quadrature(True)
        source.set_rfdc(True)
        source.set_bbdc(True)
        source.set_filter_params("Auto", "", 0.0, 0.0)

        # Cutoff at 0.75x baud passes the GMSK main lobe (which for
        # BT=0.35 is appreciably narrower than the baud rate) while the
        # 0.25x-baud transition width keeps the tap count modest — this
        # filter runs at the full sample rate on a Jetson core.
        taps = firdes.low_pass(1.0, self._sample_rate_hz, 0.75 * baud, 0.25 * baud)
        # Decimation stays 1: gmsk_demod needs its full samples_per_symbol,
        # so there is nothing here to throw away.
        translate = gr_filter.freq_xlating_fir_filter_ccf(
            1, taps, self._offset_hz, self._sample_rate_hz
        )
        demod = digital.gmsk_demod(samples_per_symbol=self._sps)
        rx_read, rx_write = self._make_pipe()
        self._rx_read_fd = rx_read
        os.set_blocking(rx_read, False)  # receive() must never block the caller
        top.connect(source, translate, demod, blocks.file_descriptor_sink(gr.sizeof_char, rx_write))

        # ---- transmit chain (ONLY when acknowledged) ----
        if self._transmit_ack:
            tx_read, tx_write = self._make_pipe(_TX_PIPE_BYTES)
            self._tx_write_fd = tx_write
            self._tx_read_fd = tx_read
            # Reads PACKED bytes (see the module docstring). It blocks on
            # an empty pipe rather than idling, which is exactly why the
            # feeder thread below must never let the pipe run dry.
            byte_source = blocks.file_descriptor_source(gr.sizeof_char, tx_read)
            modulator = digital.gmsk_mod(samples_per_symbol=self._sps, bt=_GMSK_BT)
            scale = blocks.multiply_const_cc(self._amplitude)
            rotate = blocks.rotator_cc(2.0 * math.pi * self._offset_hz / self._sample_rate_hz)
            # 64 Ki-sample DMA buffer, twice the receive side's: the
            # transmitter has no way to ask for more time, so depth here
            # is the only cushion against a late feeder. The trailing
            # False is cyclic mode — which must stay off, since cyclic
            # would replay the last buffer forever after we stop.
            sink = iio.fmcomms2_sink_fc32(self._uri, [True, True], 0x10000, False)
            sink.set_len_tag_key("")
            sink.set_frequency(int(self._center_freq_hz))
            sink.set_samplerate(int(self._sample_rate_hz))
            sink.set_attenuation(0, float(self._tx_attenuation_db))
            sink.set_filter_params("Auto", "", 0.0, 0.0)
            top.connect(byte_source, modulator, scale, rotate, sink)
            self._tx_sink = sink

        top.start()
        if self._tx_write_fd is not None:
            # Only now: the feeder's first write must have a running
            # flowgraph to drain it, or it blocks on a pipe nobody reads.
            self._tx_running = True
            self._tx_thread = threading.Thread(
                target=self._feed_tx,
                args=(self._tx_write_fd,),
                name=f"{self._name}-tx-feed",
                daemon=True,
            )
            self._tx_thread.start()
        return top

    def _ensure_started(self) -> bool:
        """Open the radio once, remembering failure rather than retrying hard.

        Returns:
            True when the flowgraph is running.
        """
        if self._flowgraph is not None:
            return True
        if self._start_error:
            return False  # already failed; do not thrash the radio every call
        try:
            self._flowgraph = self._build()
        except Exception as exc:  # noqa: BLE001 — a radio fault is a flag, never a crash
            self._start_error = f"{type(exc).__name__}: {exc}"
            self.silence_tx()
            return False
        return True

    # -------------------------------------------------------------- traffic --

    def send(self, frame: bytes) -> int:
        """Transmit one frame — if and only if permitted.

        Args:
            frame: Complete frame bytes (sync marker included by the
                framer above).

        Returns:
            0 when queued for transmission; ``VALIDITY_FLAG_COMM`` when
            refused for want of acknowledgement, when the radio could
            not be opened, or when the modulator is already further
            behind than ``_TX_QUEUE_LIMIT`` frames. Never raises, never
            radiates unacknowledged.
        """
        if not self._transmit_ack:
            self._refused += 1
            return int(hal_pb2.VALIDITY_FLAG_COMM)
        if not self._ensure_started():
            return int(hal_pb2.VALIDITY_FLAG_COMM)
        with self._tx_lock:
            if not self._tx_running:
                # Radio open but the feeder is gone (shutdown, or a wedged
                # write). Queueing here would be a silent black hole.
                return int(hal_pb2.VALIDITY_FLAG_COMM)
            if len(self._tx_queue) >= _TX_QUEUE_LIMIT:
                # The air is slower than the caller. Dropping here and
                # saying so beats an unbounded queue that turns into
                # minutes of stale telemetry arriving late.
                self._dropped_sends += 1
                return int(hal_pb2.VALIDITY_FLAG_COMM)
            self._tx_queue.append(PREAMBLE + frame)
        return 0

    def _feed_tx(self, write_fd: int) -> None:
        """Keep the modulator fed: queued frames, otherwise idle.

        Runs until ``close()`` clears the running flag. Every write is
        blocking, which is precisely the point — the pipe drains at the
        modulator's rate, so this thread cannot outrun the radio and
        cannot starve it either.

        Args:
            write_fd: The transmit pipe's write end, owned by this thread.
        """
        idle = bytes([IDLE_BYTE]) * _IDLE_CHUNK_BYTES
        while self._tx_running:
            with self._tx_lock:
                chunk = self._tx_queue.popleft() if self._tx_queue else idle
            # The write is OUTSIDE the lock, and must stay there: it
            # blocks until the modulator makes room, so holding the lock
            # across it would stall every send() for a pipe depth. The
            # lock protects the queue, never the I/O.
            try:
                os.write(write_fd, chunk)
            except OSError:
                # BrokenPipeError when close() drops the read end while
                # we are blocked here — the intended way out, not a fault.
                return

    def receive(self) -> list[bytes]:
        """Collect byte-aligned blocks recovered since the last call.

        Returns:
            Byte blocks starting at a recovered sync marker; empty when
            nothing has locked or the radio is unavailable. Never raises.
        """
        if not self._ensure_started():
            return []
        try:
            self._drain_rx_pipe()
            return self._recover_blocks()
        except Exception:  # noqa: BLE001 — demodulation trouble is not a crash
            return []

    def _recover_blocks(self) -> list[bytes]:
        """Hunt the sync marker at bit level and pack byte-aligned blocks.

        GMSK demodulation gives a bit stream of arbitrary phase, so byte
        alignment must be re-established from the marker before the
        framer (which works in bytes) can see anything.

        Despite the list return type this yields AT MOST ONE block, and
        that block runs from the first marker to the end of the buffer —
        so several frames commonly ride inside it. That is intentional:
        once byte alignment is established every following frame shares
        it, and the framer above already scans for markers and resyncs.
        Splitting frames here would duplicate the framer badly, and would
        make this PHY assume a framing it is not supposed to know. The
        list stays because the interface must also serve PHYs that
        genuinely recover discrete bursts.

        Returns:
            A single byte block starting at a recovered sync marker, or
            an empty list if nothing locked.
        """
        import numpy as np

        with self._rx_lock:
            if len(self._rx_bits) < len(self._sync_bits) + 8:
                return []
            # Take the buffer and release the lock: the collector thread
            # keeps appending while we correlate, and whatever we could
            # not consume is prepended back at the end.
            keep = np.frombuffer(bytes(self._rx_bits), dtype=np.uint8)
            self._rx_bits = bytearray()

        # Correlate in +/-1 space rather than 0/1: matching bits then
        # contribute +1 and mismatching bits -1, so a perfect match scores
        # exactly the marker length and disagreement actually subtracts.
        # (In 0/1 space a stream of all-zeros would score the same as a
        # marker whose one-bits happened to line up.)
        pattern = np.asarray(self._sync_bits, dtype=np.int8) * 2 - 1
        correlation = np.correlate((keep.astype(np.int8) * 2 - 1), pattern, mode="valid")
        # Each wrong bit costs TWO: it forfeits the +1 it would have
        # scored and contributes -1 instead. Hence 2x, not 1x, the error
        # budget — the usual off-by-a-factor-of-two trap in this idiom.
        threshold = len(self._sync_bits) - 2 * _MAX_SYNC_BIT_ERRORS
        hits = np.flatnonzero(correlation >= threshold)
        self._sync_detections += int(hits.size)

        # ACQUIRE once, then TRACK. The retained partial byte below keeps
        # this call in phase with the last, so once alignment is known it
        # stays known and every subsequent call simply packs and emits.
        #
        # Re-hunting the marker on every call instead is what cost 32% of
        # frames on 2026-07-29: a frame whose marker landed near the end
        # of one call's buffer was emitted as a few bytes, and the
        # CONTINUATION arriving next call contained no marker of its own,
        # so it was discarded — the framer's partial frame could never
        # complete. Byte alignment is a property of the stream, not of
        # whatever slice of it one call happened to see.
        realigned = False
        if self._locked:
            start = 0
            # While locked, markers must land on byte boundaries. If some
            # correlated and NONE are aligned, the demodulator slipped:
            # re-acquire on the new phase rather than emit garbage.
            if hits.size and not bool(np.any(hits % 8 == 0)):
                start = int(hits[0])
                realigned = True
                self._resyncs += 1
        elif hits.size == 0:
            # Never locked and nothing found: keep a marker-length tail so
            # an ASM straddling this read is still findable, drop the rest.
            with self._rx_lock:
                tail = keep[-(len(self._sync_bits) - 1) :] if keep.size else keep
                self._rx_bits = bytearray(tail.tobytes()) + self._rx_bits
            return []
        else:
            start = int(hits[0])
            realigned = True
            self._locked = True

        usable = keep[start:]
        whole_bytes = (usable.size // 8) * 8
        if whole_bytes == 0:
            with self._rx_lock:
                self._rx_bits = bytearray(usable.tobytes()) + self._rx_bits
            return []
        packed = np.packbits(usable[:whole_bytes])
        # Correlation has DECIDED this is the marker, so hand the framer
        # a nominal one: a bit error inside the ASM is a synchronization
        # detail, not payload. The CRC below still judges the frame, so a
        # noise pattern that merely correlated is rejected there.
        #
        # Only the FIRST marker is repaired, since only the first was
        # correlated. Later frames inside this block need intact markers
        # of their own for the framer to find them — an acceptable
        # asymmetry, because a channel corrupting markers that often is
        # failing the CRC anyway.
        #
        # Gated on `realigned`: only a block that BEGINS at a correlation
        # hit begins with a marker. While tracking, the block starts
        # wherever the stream happens to be, and stamping a marker over
        # its first four bytes would corrupt payload.
        marker = np.frombuffer(bytes.fromhex(self._sync_hex), dtype=np.uint8)
        if realigned and packed.size >= marker.size:
            packed = packed.copy()
            packed[: marker.size] = marker
        with self._rx_lock:
            # Retain the trailing partial byte so the next call continues
            # in phase with this burst.
            self._rx_bits = bytearray(usable[whole_bytes:].tobytes()) + self._rx_bits
        return [packed.tobytes()]

    def _make_pipe(self, size_bytes: int = 1 << 20) -> tuple[int, int]:
        """Create a pipe for the sample path and remember both ends.

        Args:
            size_bytes: Requested pipe capacity. Roomy on receive, so
                demodulated bits never back-pressure the radio; small on
                transmit, because a deep pipe is transmit latency.

        Returns:
            Tuple of (read fd, write fd).
        """
        read_fd, write_fd = os.pipe()
        try:
            # 1031 is F_SETPIPE_SZ, spelled numerically because Python's
            # fcntl module does not export it on this platform. Capacity
            # belongs to the pipe, not to an end, so either fd will do.
            #
            # Failure is deliberately ignored: growing a pipe beyond
            # /proc/sys/fs/pipe-max-size needs CAP_SYS_RESOURCE, and the
            # kernel silently clamps rather than erroring in most cases.
            # This is a buffering hint — the default 64 KiB still works,
            # just with more round trips.
            fcntl.fcntl(read_fd, 1031, size_bytes)
        except OSError:
            pass
        self._pipe_fds.extend([read_fd, write_fd])
        return read_fd, write_fd

    def _drain_rx_pipe(self) -> None:
        """Move demodulated bits from the pipe into the bit buffer.

        One pipe byte is one demodulated BIT here (module docstring), so
        the byte counts in this method are bit counts — no factor of
        eight is missing.
        """
        if self._rx_read_fd is None:
            return
        while True:
            try:
                block = os.read(self._rx_read_fd, 1 << 16)
            except BlockingIOError:
                return  # non-blocking fd, nothing left: the normal exit
            except OSError:
                return
            if not block:
                return
            self._rx_bits_seen += len(block)
            with self._rx_lock:
                # The binary slicer already emits 0x00/0x01; masking to
                # the low bit costs nothing and keeps the correlator's
                # +/-1 mapping correct if a block ever emits 0xFF.
                self._rx_bits.extend(block.translate(_LOW_BIT))
                if len(self._rx_bits) > _RX_BIT_BUFFER_LIMIT:
                    del self._rx_bits[: len(self._rx_bits) - _RX_BIT_BUFFER_LIMIT]

    # --------------------------------------------------------------- safety --

    def silence_tx(self) -> None:
        """Drive the transmitter to maximum attenuation (every exit path).

        The Pluto's DMA can keep replaying a stale buffer after a
        flowgraph stops; leaving the process must leave the radio quiet.
        """
        if self._tx_sink is None:
            return
        try:
            self._tx_sink.set_attenuation(0, MAX_ATTENUATION_DB)
        except Exception:  # noqa: BLE001 — best effort; never mask the exit path
            return

    def close(self) -> None:
        """Stop the flowgraph, silencing the transmitter first.

        Order matters: the TX pipe's write end is closed FIRST, because a
        file-descriptor source blocked on an empty pipe will not return
        from ``stop()`` until its input reaches EOF.
        """
        self.silence_tx()
        # Stop the feeder BEFORE its pipe goes away. It is blocked in a
        # write at most one pipe-depth long, and the flowgraph is still
        # draining, so this returns promptly; the timeout is the
        # backstop for a flowgraph that has already died.
        self._tx_running = False
        feeder_stopped = True
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=2.0)
            feeder_stopped = not self._tx_thread.is_alive()
            self._tx_thread = None
        with self._tx_lock:
            self._tx_queue.clear()
            if not feeder_stopped:
                # The feeder is wedged on a flowgraph that stopped
                # consuming. Leak the descriptor rather than close it:
                # fd numbers are reused, and a stray idle write into
                # somebody else's descriptor is a far worse failure than
                # one leaked fd on a pathological shutdown. Closing the
                # read end below still frees the writer with EPIPE.
                self._pipe_fds = [fd for fd in self._pipe_fds if fd != self._tx_write_fd]
                self._tx_write_fd = None
                self._close_tx_read()  # EPIPE frees the writer; EOF frees stop()
            elif self._tx_write_fd is not None:
                try:
                    os.close(self._tx_write_fd)
                except OSError:
                    pass
                self._pipe_fds = [fd for fd in self._pipe_fds if fd != self._tx_write_fd]
                self._tx_write_fd = None
        if self._flowgraph is not None:
            try:
                self._flowgraph.stop()
                self._flowgraph.wait()
            except Exception:  # noqa: BLE001 — shutdown must never raise
                pass
        self.silence_tx()  # again after stop: the DMA may have replayed
        for fd in self._pipe_fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self._pipe_fds = []
        self._tx_read_fd = None
        self._rx_read_fd = None
        self._flowgraph = None
        self._tx_sink = None
        # A future flowgraph starts at an unrelated bit phase, so the
        # alignment learned by this one does not survive the radio.
        self._locked = False

    def _close_tx_read(self) -> None:
        """Close the transmit pipe's read end, if it is still open."""
        if self._tx_read_fd is None:
            return
        try:
            os.close(self._tx_read_fd)
        except OSError:
            pass
        self._pipe_fds = [fd for fd in self._pipe_fds if fd != self._tx_read_fd]
        self._tx_read_fd = None

    def describe(self) -> list[str]:
        """Describe the radio and, first, whether it may transmit.

        Returns:
            Lines naming the transmit gate and the radio settings.
        """
        gate = (
            "TRANSMIT ENABLED (cabled + 30 dB pads only; CONTINUOUS carrier once open)"
            if self._transmit_ack
            else "receive-only (no transmit_ack in config; TX chain not built)"
        )
        baud = self._sample_rate_hz / self._sps
        return [
            f"modem: pluto_gmsk {gate}",
            f"modem: uri={self._uri} center={self._center_freq_hz / 1e6:g} MHz "
            f"rate={self._sample_rate_hz / 1e6:g} MSa/s offset={self._offset_hz / 1e3:g} kHz "
            f"sps={self._sps} ({baud / 1e3:.1f} kbaud) tx_atten={self._tx_attenuation_db:g} dB",
            f"modem: bit-sync on ASM 0x{self._sync_hex} (must match the framer's marker)",
        ]
