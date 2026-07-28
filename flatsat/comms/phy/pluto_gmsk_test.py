"""Pluto modem: RF safety is structural, and provable without a radio.

No hardware and no GNU Radio required — these tests exist precisely to
prove the transmit gate holds on any machine, including CI. Nothing here
opens a radio or radiates.
"""

import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.phy.pluto_gmsk import PlutoGmskModem
from flatsat.msgs import hal_pb2


def _options(transmit_ack: bool = False) -> comms_config_pb2.PlutoGmskModemOptions:
    return comms_config_pb2.PlutoGmskModemOptions(
        uri="ip:192.168.2.1",
        center_freq_hz=915e6,
        sample_rate_hz=2e6,
        transmit_ack=transmit_ack,
    )


@pytest.mark.verifies("FSW-RADIO-001", "FSW-LINK-005")
def test_transmit_refused_without_acknowledgement() -> None:
    """The default configuration cannot radiate. Ever."""
    modem = PlutoGmskModem.from_config("radio0", _options(transmit_ack=False))
    assert not modem.may_transmit
    flags = modem.send(b"would-be transmission")
    assert flags & hal_pb2.VALIDITY_FLAG_COMM, "refusal must be flagged, not silent"
    assert modem.refused_transmissions == 1


@pytest.mark.verifies("FSW-LINK-005")
def test_refusal_is_visible_in_describe() -> None:
    """An operator reading the startup log must see the gate state."""
    quiet = PlutoGmskModem.from_config("radio0", _options(transmit_ack=False))
    assert "receive-only" in quiet.describe()[0]
    armed = PlutoGmskModem.from_config("radio0", _options(transmit_ack=True))
    assert "TRANSMIT ENABLED" in armed.describe()[0]


def test_receive_is_always_permitted() -> None:
    """Listening radiates nothing, so it needs no acknowledgement."""
    modem = PlutoGmskModem.from_config("radio0", _options(transmit_ack=False))
    assert modem.receive() == []


def test_acknowledged_transmit_fails_soft_until_graduated() -> None:
    """With TX acknowledged but no radio: a flag, never a crash or a hang."""
    modem = PlutoGmskModem.from_config("radio0", _options(transmit_ack=True))
    flags = modem.send(b"frame")
    assert flags & hal_pb2.VALIDITY_FLAG_COMM


def test_defaults_keep_energy_off_dc() -> None:
    """The zero-IF lesson is a default, not a footnote (BER 1.8e-1 -> 0.00)."""
    modem = PlutoGmskModem.from_config("radio0", _options())
    assert "offset=250 kHz" in " ".join(modem.describe())


def test_missing_radio_parameters_fail_loud() -> None:
    with pytest.raises(ValueError, match="requires uri"):
        PlutoGmskModem.from_config("radio0", comms_config_pb2.PlutoGmskModemOptions())


def test_close_is_safe_without_a_radio() -> None:
    """Every exit path silences TX; with no flowgraph that is a no-op."""
    modem = PlutoGmskModem.from_config("radio0", _options(transmit_ack=True))
    modem.close()  # must not raise
