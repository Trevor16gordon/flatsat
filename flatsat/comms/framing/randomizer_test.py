"""CCSDS randomizer: conformance to the published sequence, and the property that matters.

The sequence is checked against the bits CCSDS 131.0-B prints, because a
randomizer that is self-consistent but non-standard would interoperate
with nothing and the mistake would only surface against real hardware.
Beyond that, the point of the thing is transition density, so that is
measured rather than assumed.
"""

import numpy as np
import pytest

from flatsat.comms.framing import randomizer

# CCSDS 131.0-B, section 9: the first 40 bits of the pseudo-random
# sequence, printed in the standard as
#   1111 1111 0100 1000 0000 1110 1100 0000 1001 1010
PUBLISHED_PREFIX = bytes([0xFF, 0x48, 0x0E, 0xC0, 0x9A])


def _longest_run(data: bytes) -> int:
    """Length of the longest run of identical BITS.

    Args:
        data: Bytes to inspect.

    Returns:
        The longest run of consecutive equal bits.
    """
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))
    if bits.size == 0:
        return 0
    # Boundaries are where the value changes; the gaps between them are
    # the runs.
    edges = np.flatnonzero(np.diff(bits)) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [bits.size]])
    return int(np.max(ends - starts))


def _transition_density(data: bytes) -> float:
    """Fraction of adjacent bit pairs that differ.

    This is what a timing-recovery loop actually feeds on.

    Args:
        data: Bytes to inspect.

    Returns:
        Transitions divided by bit pairs; 0.5 is ideal for random data.
    """
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))
    return float(np.count_nonzero(np.diff(bits))) / float(max(bits.size - 1, 1))


@pytest.mark.verifies("FSW-LINK-009")
def test_sequence_matches_the_published_standard() -> None:
    """The LFSR must reproduce the bits CCSDS prints, exactly."""
    assert randomizer.sequence(5)[:5] == PUBLISHED_PREFIX


def test_first_byte_is_the_all_ones_seed() -> None:
    """All eight stages preset to 1 means the first output byte is 0xFF."""
    assert randomizer.sequence(1)[0] == 0xFF


def test_apply_is_its_own_inverse() -> None:
    """XOR both ways, so the two ends cannot disagree about direction."""
    payload = bytes(range(256)) * 3
    assert randomizer.apply(randomizer.apply(payload)) == payload


def test_sequence_is_stable_as_the_cache_grows() -> None:
    """A longer request must not change the bytes already handed out."""
    short = randomizer.sequence(64)[:64]
    long_prefix = randomizer.sequence(20_000)[:64]
    assert short == long_prefix


def test_sequence_period_is_255_bytes() -> None:
    """A maximal-length 8-bit LFSR has a 255-bit period.

    Because 255 and 8 are coprime, the BYTE sequence repeats only after
    255 bytes — worth pinning, since a shorter period would mean frames
    at certain offsets get correlated randomization.
    """
    seq = randomizer.sequence(600)
    assert seq[:255] == seq[255:510]


@pytest.mark.verifies("FSW-LINK-009")
def test_randomizing_breaks_up_a_constant_payload() -> None:
    """The whole point: a pathological payload must not reach the air as-is.

    48 zero bytes recovered 3 of 40 frames on hardware because a
    constant run starves the demodulator's timing loop. After
    randomization the wire sees no run longer than a byte or so.
    """
    flat = bytes(64)  # 512 identical bits — the measured failure case
    assert _longest_run(flat) == 512, "the raw payload really is one long run"

    scrambled = randomizer.apply(flat)
    assert _longest_run(scrambled) <= 16, f"still has a {_longest_run(scrambled)}-bit run"
    assert _transition_density(scrambled) > 0.3


@pytest.mark.verifies("FSW-LINK-009")
def test_randomizing_helps_all_the_pathological_patterns() -> None:
    """Zeros are not the only low-entropy payload; 0xFF and 0xAA also matter."""
    for pattern in (b"\x00", b"\xff", b"\xf0", b"\x01"):
        raw = pattern * 128
        scrambled = randomizer.apply(raw)
        assert _longest_run(scrambled) <= 16, f"pattern {pattern!r} still runs long"


def test_high_entropy_payload_is_not_made_worse() -> None:
    """Randomizing already-random data must not degrade it."""
    rng = np.random.default_rng(5)
    raw = rng.integers(0, 256, 512, dtype=np.uint8).tobytes()
    assert _transition_density(randomizer.apply(raw)) > 0.35
