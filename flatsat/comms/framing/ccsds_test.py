"""CCSDS framing: round trip, resynchronization, and honest corruption."""

import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing.ccsds import SYNC, CcsdsFramer


def _framer(max_payload: int = 4096) -> CcsdsFramer:
    return CcsdsFramer.from_config(
        comms_config_pb2.CcsdsFramerOptions(max_payload_bytes=max_payload)
    )


@pytest.mark.verifies("FSW-LINK-001")
def test_round_trip() -> None:
    framer = _framer()
    payload = b"telemetry payload \x00\x01\xff"
    assert framer.feed(framer.frame(payload)) == [payload]
    assert framer.dropped_frames == 0


def test_multiple_frames_in_one_block() -> None:
    framer = _framer()
    wire = framer.frame(b"one") + framer.frame(b"two") + framer.frame(b"three")
    assert framer.feed(wire) == [b"one", b"two", b"three"]


def test_frame_split_across_reads() -> None:
    """The channel delivers bytes, not frames — partials must buffer."""
    framer = _framer()
    wire = framer.frame(b"split me up")
    assert framer.feed(wire[:5]) == []
    assert framer.feed(wire[5:9]) == []
    assert framer.feed(wire[9:]) == [b"split me up"]


@pytest.mark.verifies("FSW-LINK-002")
def test_leading_garbage_resynchronizes() -> None:
    """Reception starts mid-noise: find the marker, deliver the frame."""
    framer = _framer()
    wire = b"\x00\xff\xa5 noise before the marker " + framer.frame(b"payload")
    assert framer.feed(wire) == [b"payload"]


@pytest.mark.verifies("FSW-LINK-002")
def test_corrupted_frame_is_dropped_and_counted() -> None:
    framer = _framer()
    wire = bytearray(framer.frame(b"important telemetry"))
    wire[len(SYNC) + 4] ^= 0xFF  # flip a payload bit
    assert framer.feed(bytes(wire)) == [], "corrupt frame must never be delivered"
    assert framer.dropped_frames == 1


@pytest.mark.verifies("FSW-LINK-002")
def test_good_frame_after_corrupted_one_still_arrives() -> None:
    """One bad frame must not poison the stream behind it."""
    framer = _framer()
    bad = bytearray(framer.frame(b"corrupted"))
    bad[len(SYNC) + 5] ^= 0x01
    good = framer.frame(b"intact")
    assert framer.feed(bytes(bad) + good) == [b"intact"]
    assert framer.dropped_frames == 1


def test_absurd_length_is_rejected_without_stalling() -> None:
    """A corrupted length field must not make the deframer wait forever."""
    framer = _framer(max_payload=64)
    bogus = SYNC + (9999).to_bytes(2, "big") + b"\x00" * 8
    assert framer.feed(bogus + framer.frame(b"after")) == [b"after"]
    assert framer.dropped_frames == 1


def test_oversized_payload_fails_loud_on_send() -> None:
    framer = _framer(max_payload=16)
    with pytest.raises(ValueError, match="exceeds max"):
        framer.frame(b"x" * 17)


def test_empty_payload_round_trips() -> None:
    framer = _framer()
    assert framer.feed(framer.frame(b"")) == [b""]
