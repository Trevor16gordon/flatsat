"""Loopback modem: delivery between endpoints, and impairments on demand."""

import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.phy.loopback import LoopbackModem, reset_channels


@pytest.fixture(autouse=True)
def _isolate_channels() -> None:
    """Every test gets fresh loopback channels."""
    reset_channels()


def _modem(name: str, channel: str, **kwargs: float) -> LoopbackModem:
    options = comms_config_pb2.LoopbackModemOptions(channel=channel, seed=5)
    for field, value in kwargs.items():
        setattr(options, field, value)
    return LoopbackModem.from_config(name, options)


def test_frames_reach_the_other_endpoint() -> None:
    flight = _modem("flight", "test/phy/basic")
    ground = _modem("ground", "test/phy/basic")
    assert flight.send(b"hello") == 0
    assert ground.receive() == [b"hello"]


def test_sender_does_not_hear_itself() -> None:
    flight = _modem("flight", "test/phy/echo")
    _modem("ground", "test/phy/echo")
    flight.send(b"hello")
    assert flight.receive() == []


def test_receive_drains() -> None:
    flight = _modem("flight", "test/phy/drain")
    ground = _modem("ground", "test/phy/drain")
    flight.send(b"one")
    flight.send(b"two")
    assert ground.receive() == [b"one", b"two"]
    assert ground.receive() == []


def test_channels_are_isolated() -> None:
    flight = _modem("flight", "test/phy/a")
    other = _modem("ground", "test/phy/b")
    flight.send(b"not for you")
    assert other.receive() == []


def test_frame_loss_drops_some_frames() -> None:
    flight = _modem("flight", "test/phy/loss", frame_loss=0.5)
    ground = _modem("ground", "test/phy/loss")
    for _ in range(100):
        flight.send(b"frame")
    received = len(ground.receive())
    assert 0 < received < 100


def test_bit_errors_corrupt_without_losing() -> None:
    flight = _modem("flight", "test/phy/ber", bit_error_rate=0.05)
    ground = _modem("ground", "test/phy/ber")
    original = b"\x00" * 64
    flight.send(original)
    delivered = ground.receive()
    assert len(delivered) == 1, "bit errors corrupt, they do not drop"
    assert delivered[0] != original


def test_perfect_channel_by_default() -> None:
    flight = _modem("flight", "test/phy/clean")
    ground = _modem("ground", "test/phy/clean")
    payload = bytes(range(256))
    flight.send(payload)
    assert ground.receive() == [payload]


def test_close_leaves_the_channel() -> None:
    flight = _modem("flight", "test/phy/close")
    ground = _modem("ground", "test/phy/close")
    ground.close()
    flight.send(b"nobody home")
    assert ground.receive() == []


def test_channel_name_required() -> None:
    with pytest.raises(ValueError, match="requires a channel"):
        LoopbackModem.from_config("flight", comms_config_pb2.LoopbackModemOptions())
