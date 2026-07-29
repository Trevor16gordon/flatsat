"""Pluto modem: RF safety and bit synchronization, provable without a radio.

Two things are tested here on any machine, with no hardware and no GNU
Radio: that the transmit gate holds, and that the bit-level frame
synchronizer recovers byte alignment from a demodulated bit stream of
arbitrary phase. The parts that genuinely need a radio (modulation,
BER through the cabled path) belong to the bench campaign, not to CI.
"""

import numpy as np
import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing.ccsds import CcsdsFramer
from flatsat.comms.phy.pluto_gmsk import PREAMBLE, PlutoGmskModem, _sync_bits
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
