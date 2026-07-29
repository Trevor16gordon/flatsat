"""Pluto modem: RF safety and bit synchronization, provable without a radio.

Two things are tested here on any machine, with no hardware and no GNU
Radio: that the transmit gate holds, and that the bit-level frame
synchronizer recovers byte alignment from a demodulated bit stream of
arbitrary phase. The parts that genuinely need a radio (modulation,
BER through the cabled path) belong to the bench campaign, not to CI.
"""

import contextlib
import os
import threading

import numpy as np
import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing.ccsds import CcsdsFramer
from flatsat.comms.phy.pluto_gmsk import IDLE_BYTE, PREAMBLE, PlutoGmskModem, _sync_bits
from flatsat.msgs import hal_pb2


def _options(transmit_ack: bool = False) -> comms_config_pb2.PlutoGmskModemOptions:
    return comms_config_pb2.PlutoGmskModemOptions(
        uri="ip:192.168.2.1",
        center_freq_hz=915e6,
        sample_rate_hz=2e6,
        transmit_ack=transmit_ack,
    )


def _modem(transmit_ack: bool = False) -> PlutoGmskModem:
    return PlutoGmskModem.from_config("radio0", _options(transmit_ack))


def _feed_bits(modem: PlutoGmskModem, data: bytes, phase: int) -> None:
    """Push bytes into the modem's RX bit buffer at an arbitrary bit phase.

    Args:
        modem: The modem under test.
        data: Bytes as they would arrive on the air.
        phase: Junk bits prepended, simulating demodulator phase.
    """
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))
    junk = np.zeros(phase, dtype=np.uint8)
    modem._rx_bits.extend(np.concatenate([junk, bits]).tobytes())  # noqa: SLF001


# ------------------------------------------------------------- RF safety --


@pytest.mark.verifies("FSW-RADIO-001", "FSW-LINK-005")
def test_transmit_refused_without_acknowledgement() -> None:
    """The default configuration cannot radiate. Ever."""
    modem = _modem(transmit_ack=False)
    assert not modem.may_transmit
    flags = modem.send(b"would-be transmission")
    assert flags & hal_pb2.VALIDITY_FLAG_COMM, "refusal must be flagged, not silent"
    assert modem.refused_transmissions == 1


@pytest.mark.verifies("FSW-LINK-005")
def test_refusal_does_not_even_open_the_radio() -> None:
    """A refused send must not touch hardware — no flowgraph, no TX chain."""
    modem = _modem(transmit_ack=False)
    modem.send(b"frame")
    assert modem._flowgraph is None  # noqa: SLF001
    assert modem._tx_sink is None  # noqa: SLF001


@pytest.mark.verifies("FSW-LINK-005")
def test_refusal_is_visible_in_describe() -> None:
    """An operator reading the startup log must see the gate state."""
    quiet = _modem(transmit_ack=False)
    assert "receive-only" in quiet.describe()[0]
    assert "TX chain not built" in quiet.describe()[0]
    armed = _modem(transmit_ack=True)
    assert "TRANSMIT ENABLED" in armed.describe()[0]


def test_defaults_keep_energy_off_dc() -> None:
    """The zero-IF lesson is a default, not a footnote (BER 1.8e-1 -> 0.00)."""
    assert "offset=250 kHz" in " ".join(_modem().describe())


def test_missing_radio_parameters_fail_loud() -> None:
    with pytest.raises(ValueError, match="requires uri"):
        PlutoGmskModem.from_config("radio0", comms_config_pb2.PlutoGmskModemOptions())


def test_close_is_safe_without_a_radio() -> None:
    """Every exit path silences TX; with no flowgraph that is a no-op."""
    _modem(transmit_ack=True).close()  # must not raise


def test_radio_failure_is_a_flag_not_a_crash() -> None:
    """No Pluto at this URI: send flags COMM, receive returns nothing."""
    modem = PlutoGmskModem(
        "radio0",
        uri="ip:203.0.113.1",  # TEST-NET-3: guaranteed to have no radio
        center_freq_hz=915e6,
        sample_rate_hz=2e6,
        transmit_ack=True,
    )
    assert modem.send(b"frame") & hal_pb2.VALIDITY_FLAG_COMM
    assert modem.receive() == []
    assert modem.start_error, "the failure reason must be retained, not swallowed"


# ------------------------------------------------------ continuous carrier --


def _feed_once(modem: PlutoGmskModem, read_size: int) -> bytes:
    """Run the transmit feeder for exactly one write and return what it wrote.

    Args:
        modem: The modem under test.
        read_size: Bytes to read back from the pipe.

    Returns:
        The bytes the feeder placed on the wire.
    """
    # A plain pipe stands in for the flowgraph: reading from it is what
    # the modulator would do, so the feeder is paced by this test exactly
    # as it would be by the radio. No GNU Radio, no hardware.
    read_fd, write_fd = os.pipe()
    try:
        modem._tx_running = True  # noqa: SLF001
        thread = threading.Thread(target=modem._feed_tx, args=(write_fd,), daemon=True)  # noqa: SLF001
        thread.start()
        written = os.read(read_fd, read_size)
        # Clearing the flag is not enough to stop a feeder already blocked
        # in write(); closing the read end is what frees it, via EPIPE.
        modem._tx_running = False  # noqa: SLF001
        os.close(read_fd)
        thread.join(timeout=2.0)
        return written
    finally:
        with contextlib.suppress(OSError):
            os.close(write_fd)


@pytest.mark.verifies("FSW-LINK-008")
def test_idle_fill_keeps_the_modulator_running() -> None:
    """With nothing queued the feeder must still supply symbols.

    A silent gap costs the demodulator its clock lock — the regression
    that produced 100 frames sent and 0 recovered on 2026-07-29.
    """
    modem = _modem(transmit_ack=True)
    written = _feed_once(modem, 64)
    assert written == bytes([IDLE_BYTE]) * 64, "gaps must be filled, never left empty"


@pytest.mark.verifies("FSW-LINK-008")
def test_queued_frames_take_precedence_over_idle() -> None:
    """A queued frame goes out ahead of filler, preamble first."""
    modem = _modem(transmit_ack=True)
    modem._tx_queue.append(PREAMBLE + b"frame-bytes")  # noqa: SLF001
    written = _feed_once(modem, len(PREAMBLE) + 11)
    assert written == PREAMBLE + b"frame-bytes"


def test_send_pushes_back_instead_of_queueing_without_limit() -> None:
    """When the air is slower than the caller, say so rather than lag."""
    modem = _modem(transmit_ack=True)
    # A non-None flowgraph is all _ensure_started() checks, so any object
    # convinces send() the radio is open. The queue logic under test is
    # pure bookkeeping — it never touches the flowgraph it is handed.
    modem._flowgraph = object()  # noqa: SLF001
    modem._tx_running = True  # the feeder would normally set this  # noqa: SLF001
    accepted = sum(1 for _ in range(200) if modem.send(b"frame") == 0)
    assert accepted == 64, "the queue must be bounded"
    assert modem.dropped_sends == 136
    assert modem.send(b"frame") & hal_pb2.VALIDITY_FLAG_COMM


def test_default_link_budget_matches_the_measured_bench_point() -> None:
    """The transmit levels are measured, not chosen — pin them.

    A run with defaults 44 dB below this produced bits at the right rate
    and zero sync correlations (2026-07-29). Nothing in the type system
    distinguishes a safe-looking number from a working one, so this test
    does: change these only with a bench measurement in hand.
    """
    modem = _modem(transmit_ack=True)
    assert modem._amplitude == 0.5  # noqa: SLF001
    assert modem._tx_attenuation_db == 20.0  # noqa: SLF001
    assert "tx_atten=20 dB" in " ".join(modem.describe())


# -------------------------------------------------- bit synchronization --


@pytest.mark.verifies("FSW-LINK-008")
@pytest.mark.parametrize("phase", [0, 1, 3, 7, 13])
def test_byte_alignment_is_recovered_at_any_bit_phase(phase: int) -> None:
    """GMSK gives arbitrary bit phase; the ASM hunt restores byte alignment."""
    modem = _modem()
    framer = CcsdsFramer()
    frame = framer.frame(b"telemetry payload")
    _feed_bits(modem, PREAMBLE + frame + bytes(4), phase)

    blocks = modem._recover_blocks()  # noqa: SLF001
    assert blocks, f"no lock at bit phase {phase}"
    assert framer.feed(blocks[0]) == [b"telemetry payload"]


@pytest.mark.verifies("FSW-LINK-008")
def test_sync_tolerates_a_bit_error_in_the_marker() -> None:
    """A channel that flips a marker bit must not cost the whole frame."""
    modem = _modem()
    framer = CcsdsFramer()
    frame = bytearray(framer.frame(b"payload through noise"))
    bits = np.unpackbits(np.frombuffer(bytes(frame), dtype=np.uint8))
    bits[2] ^= 1  # one bit error inside the sync marker itself
    modem._rx_bits.extend(np.concatenate([np.zeros(5, np.uint8), bits]).tobytes())  # noqa: SLF001

    blocks = modem._recover_blocks()  # noqa: SLF001
    assert blocks, "one marker bit error must still lock"
    assert framer.feed(blocks[0]) == [b"payload through noise"]


@pytest.mark.verifies("FSW-LINK-008")
def test_frame_straddling_two_reads_is_not_lost() -> None:
    """Byte alignment must survive a read boundary.

    Frames do not arrive aligned to receive() calls, so a frame whose
    marker lands in one read and whose payload finishes in the next must
    still be recovered. Re-hunting the marker per call instead discards
    the continuation and cost 32% of frames on hardware (2026-07-29).
    """
    modem = _modem()
    framer = CcsdsFramer()
    first = framer.frame(b"first payload")
    stream = PREAMBLE + first + framer.frame(b"second payload")
    bits = np.unpackbits(np.frombuffer(stream, dtype=np.uint8))
    # Cut 60 bits into the second frame: past its marker, mid-payload, so
    # the second chunk contains no marker of its own at all.
    split = (len(PREAMBLE) + len(first)) * 8 + 60

    payloads: list[bytes] = []
    for chunk in (bits[:split], bits[split:]):
        modem._rx_bits.extend(chunk.tobytes())  # noqa: SLF001
        for block in modem._recover_blocks():  # noqa: SLF001
            payloads.extend(framer.feed(block))
    assert payloads == [b"first payload", b"second payload"]


@pytest.mark.verifies("FSW-LINK-008")
def test_tracking_does_not_stamp_a_marker_over_payload() -> None:
    """Marker repair applies to an acquired block only, never mid-stream.

    A tracking block starts wherever the stream happens to be. Stamping
    the nominal ASM over its first four bytes — correct when the block
    begins at a correlation hit — would silently corrupt payload.
    """
    modem = _modem()
    framer = CcsdsFramer()
    bits = np.unpackbits(np.frombuffer(PREAMBLE + framer.frame(b"x" * 40), dtype=np.uint8))
    modem._rx_bits.extend(bits[:200].tobytes())  # noqa: SLF001
    assert modem._recover_blocks(), "expected to acquire on the marker"  # noqa: SLF001

    modem._rx_bits.extend(bits[200:].tobytes())  # noqa: SLF001
    blocks = modem._recover_blocks()  # noqa: SLF001
    assert blocks and not blocks[0].startswith(bytes.fromhex("1acffc1d"))


def test_noise_without_a_marker_yields_nothing() -> None:
    """Pure noise must not fabricate a frame."""
    modem = _modem()
    rng = np.random.default_rng(7)
    modem._rx_bits.extend(rng.integers(0, 2, 4096, dtype=np.uint8).tobytes())  # noqa: SLF001
    assert modem._recover_blocks() == []  # noqa: SLF001


def test_buffer_is_bounded_when_nothing_ever_syncs() -> None:
    """A receiver hearing only noise must not grow without limit."""
    modem = _modem()
    rng = np.random.default_rng(11)
    for _ in range(4):
        modem._rx_bits.extend(rng.integers(0, 2, 8192, dtype=np.uint8).tobytes())  # noqa: SLF001
        modem._recover_blocks()  # noqa: SLF001
    assert len(modem._rx_bits) < 1024, "unlocked bits must not accumulate"  # noqa: SLF001


def test_sync_bits_expand_the_marker_correctly() -> None:
    assert _sync_bits("1acffc1d")[:8] == [0, 0, 0, 1, 1, 0, 1, 0]  # 0x1a
    assert len(_sync_bits("1acffc1d")) == 32
