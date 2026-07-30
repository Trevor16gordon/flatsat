"""KISS seam: protocol conformance, and surviving a socket that splits anywhere.

Two things are proved here. That the framing matches the KISS spec —
checked against hand-written byte sequences rather than only against our
own encoder, because an encoder and decoder that share a mistake agree
with each other forever and with nobody else. And that the decoder
survives the ways a real socket delivers bytes: split mid-frame, several
frames at once, padding, and garbage.
"""

import socket
import threading
import time

import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.phy.kiss import FEND, FESC, TFEND, TFESC, KissModem, decode, encode
from flatsat.msgs import hal_pb2

# ------------------------------------------------------------ conformance --


@pytest.mark.verifies("FSW-LINK-011")
def test_encoding_matches_the_kiss_spec_byte_for_byte() -> None:
    """A hand-written expectation, not a round trip through our own code."""
    assert encode(b"\x01\x02") == bytes([FEND, 0x00, 0x01, 0x02, FEND])


@pytest.mark.verifies("FSW-LINK-011")
def test_delimiter_inside_the_payload_is_escaped() -> None:
    """An unescaped FEND would end the frame early and split it in two."""
    assert encode(bytes([FEND])) == bytes([FEND, 0x00, FESC, TFEND, FEND])


@pytest.mark.verifies("FSW-LINK-011")
def test_escape_character_inside_the_payload_is_escaped() -> None:
    """Escaping FEND is useless if FESC itself is not escaped too."""
    assert encode(bytes([FESC])) == bytes([FEND, 0x00, FESC, TFESC, FEND])


def test_port_number_lands_in_the_high_nibble() -> None:
    assert encode(b"x", port=3)[1] == 0x30


@pytest.mark.parametrize(
    "payload",
    [b"", b"x", bytes([FEND]), bytes([FESC]), bytes([FEND, FESC, FEND]), bytes(range(256))],
)
def test_round_trip_survives_every_awkward_payload(payload: bytes) -> None:
    """Including the two bytes the protocol reserves for itself."""
    buffer = bytearray(encode(payload))
    assert decode(buffer) == ([payload] if payload else [])


# --------------------------------------------------------- socket realities --


@pytest.mark.verifies("FSW-LINK-011")
def test_a_frame_split_across_reads_is_reassembled() -> None:
    """TCP splits wherever it likes; a partial frame must be held, not lost."""
    wire = encode(b"telemetry payload")
    buffer = bytearray(wire[:7])
    assert decode(buffer) == [], "half a frame is not a frame"
    buffer.extend(wire[7:])
    assert decode(buffer) == [b"telemetry payload"]


def test_several_frames_in_one_read_all_come_out() -> None:
    buffer = bytearray(encode(b"one") + encode(b"two") + encode(b"three"))
    assert decode(buffer) == [b"one", b"two", b"three"]


def test_trailing_partial_frame_is_kept_for_next_time() -> None:
    buffer = bytearray(encode(b"complete") + encode(b"partial")[:5])
    assert decode(buffer) == [b"complete"]
    assert len(buffer) == 5, "the partial frame must still be buffered"


def test_back_to_back_delimiters_are_padding_not_empty_frames() -> None:
    """Many TNCs pad with FEND; that must not manufacture empty frames."""
    buffer = bytearray([FEND, FEND, FEND]) + bytearray(encode(b"real"))
    assert decode(buffer) == [b"real"]


def test_non_data_commands_are_ignored() -> None:
    """TX-delay and similar TNC commands are not our traffic."""
    buffer = bytearray([FEND, 0x01, 0x32, FEND]) + bytearray(encode(b"data"))
    assert decode(buffer) == [b"data"]


def test_garbage_without_a_delimiter_is_discarded() -> None:
    buffer = bytearray(b"noise with no delimiter at all")
    assert decode(buffer) == []
    assert not buffer, "unframeable bytes must not accumulate"


# ---------------------------------------------------------------- the modem --


def test_unreachable_far_side_is_a_flag_not_a_crash() -> None:
    """No radio process listening: send flags COMM, receive returns nothing."""
    modem = KissModem("radio0", host="127.0.0.1", port=1, connect_timeout_s=0.2)
    assert modem.send(b"frame") & hal_pb2.VALIDITY_FLAG_COMM
    assert modem.receive() == []
    assert modem.start_error, "the failure reason must be retained"


def test_describe_says_the_transmit_gate_is_not_ours() -> None:
    """This modem cannot radiate; an operator must not think it gates RF."""
    lines = " ".join(KissModem("radio0").describe())
    assert "TRANSMIT GATE" in lines
    assert "far end" in lines


def test_from_config_defaults_host_and_port() -> None:
    modem = KissModem.from_config("radio0", comms_config_pb2.KissModemOptions())
    assert "127.0.0.1:8001" in " ".join(modem.describe())


@pytest.mark.verifies("FSW-LINK-011")
def test_frames_cross_a_real_socket() -> None:
    """End to end against an actual TCP peer, echoing like a TNC would."""
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def echo() -> None:
        """Accept one connection and mirror whatever arrives."""
        conn, _ = server.accept()
        with conn:
            data = conn.recv(4096)
            conn.sendall(data)

    thread = threading.Thread(target=echo, daemon=True)
    thread.start()

    modem = KissModem("radio0", port=port)
    try:
        assert modem.send(b"hello from the link layer") == 0
        # A wall-clock deadline, not an iteration count: spinning 50
        # times with no sleep is microseconds, so under load the test
        # gave up long before the echo could return.
        received: list[bytes] = []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not received:
            received = modem.receive()
            time.sleep(0.01)
        assert received == [b"hello from the link layer"]
        assert modem.frames_sent == 1
        assert modem.frames_received == 1
    finally:
        modem.close()
        server.close()
        thread.join(timeout=2.0)
