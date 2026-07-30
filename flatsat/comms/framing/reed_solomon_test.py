"""Reed-Solomon: correction up to t, refusal beyond it.

An error-correcting code has two jobs and the second is easily
forgotten: correct what it can, and REFUSE what it cannot. A decoder
that miscorrects is worse than no decoder at all, because it hands
plausible wrong bytes upward with the CRC recomputed over them. The
tests below spend as much effort on refusal as on correction.
"""

import numpy as np
import pytest

from flatsat.comms.framing import reed_solomon as rs

MESSAGE = bytes(range(223))


def _corrupt(block: bytes, positions: list[int], value: int = 0xA5) -> bytes:
    """Overwrite bytes at the given positions.

    Args:
        block: The codeword.
        positions: Indices to damage.
        value: Value to XOR in (non-zero, so it is a real change).

    Returns:
        The damaged codeword.
    """
    out = bytearray(block)
    for position in positions:
        out[position] ^= value
    return bytes(out)


# ------------------------------------------------------------ field algebra --


def test_field_tables_are_a_permutation() -> None:
    """Every non-zero element must appear exactly once as a power of alpha."""
    powers = {rs._EXP[i] for i in range(255)}  # noqa: SLF001
    assert powers == set(range(1, 256))


def test_log_and_exp_are_inverse() -> None:
    for value in range(1, 256):
        assert rs._EXP[rs._LOG[value]] == value  # noqa: SLF001


def test_multiplication_has_inverses() -> None:
    for value in (1, 2, 3, 127, 200, 255):
        assert rs._mul(value, rs._inv(value)) == 1  # noqa: SLF001


# ------------------------------------------------------------------ encoding --


def test_encoding_is_systematic() -> None:
    """The data must survive verbatim, with parity appended."""
    block = rs.encode(MESSAGE)
    assert len(block) == rs.BLOCK_BYTES
    assert block[: rs.DATA_BYTES] == MESSAGE


def test_clean_codeword_decodes_with_no_corrections() -> None:
    result = rs.decode(rs.encode(MESSAGE))
    assert result is not None
    data, corrected = result
    assert data == MESSAGE
    assert corrected == 0


def test_shortened_blocks_round_trip() -> None:
    """Frames are not all 223 bytes; a shortened code must work too."""
    for size in (1, 7, 64, 100, 222):
        payload = bytes(range(size))
        result = rs.decode(rs.encode(payload), rs.PARITY_BYTES)
        assert result is not None, f"size {size}"
        assert result[0] == payload, f"size {size}"
        assert result[1] == 0


def test_oversized_block_is_refused() -> None:
    with pytest.raises(ValueError, match="exceeds 255"):
        rs.encode(bytes(224))


# ---------------------------------------------------------------- correction --


@pytest.mark.verifies("FSW-LINK-010")
@pytest.mark.parametrize("count", [1, 2, 5, 15, 16])
def test_corrects_up_to_sixteen_corrupted_bytes(count: int) -> None:
    """RS(255,223) must recover from any t=16 byte errors."""
    rng = np.random.default_rng(count)
    positions = sorted(rng.choice(rs.BLOCK_BYTES, size=count, replace=False).tolist())
    damaged = _corrupt(rs.encode(MESSAGE), positions)

    result = rs.decode(damaged)
    assert result is not None, f"{count} errors should be correctable"
    data, corrected = result
    assert data == MESSAGE
    assert corrected == count


@pytest.mark.verifies("FSW-LINK-010")
def test_corrects_a_burst_that_would_ruin_a_bit_oriented_code() -> None:
    """16 CONSECUTIVE bad bytes is 128 bad bits — and costs the same as 16.

    This is why a byte-oriented code suits this channel: our losses
    arrive in bursts, not sprinkled uniformly.
    """
    damaged = _corrupt(rs.encode(MESSAGE), list(range(40, 56)))
    result = rs.decode(damaged)
    assert result is not None
    assert result[0] == MESSAGE


@pytest.mark.verifies("FSW-LINK-010")
def test_corrects_errors_landing_in_the_parity_itself() -> None:
    """Parity bytes are as exposed as data bytes on the air."""
    damaged = _corrupt(rs.encode(MESSAGE), [230, 240, 250, 254])
    result = rs.decode(damaged)
    assert result is not None
    assert result[0] == MESSAGE


@pytest.mark.verifies("FSW-LINK-010")
def test_single_bit_errors_are_corrected() -> None:
    """The common case: one flipped bit, which today costs a whole frame."""
    block = bytearray(rs.encode(MESSAGE))
    block[77] ^= 0x01
    result = rs.decode(bytes(block))
    assert result is not None
    assert result[0] == MESSAGE


# ------------------------------------------------------------------- refusal --


@pytest.mark.verifies("FSW-LINK-010")
@pytest.mark.parametrize("count", [17, 20, 40, 100])
def test_refuses_beyond_its_correction_power(count: int) -> None:
    """Past t, the decoder must refuse rather than MISCORRECT.

    The property that matters is not "it fails" — it is that it never
    returns data that is neither correct nor flagged. A miscorrection
    passes a recomputed CRC and reaches the application as plausible
    wrong bytes, which is strictly worse than a dropped frame.
    """
    rng = np.random.default_rng(1000 + count)
    positions = sorted(rng.choice(rs.BLOCK_BYTES, size=count, replace=False).tolist())
    damaged = _corrupt(rs.encode(MESSAGE), positions)

    result = rs.decode(damaged)
    assert result is None or result[0] == MESSAGE, (
        f"MISCORRECTION with {count} errors: returned wrong data claiming success"
    )


@pytest.mark.verifies("FSW-LINK-010")
def test_pure_noise_is_almost_never_accepted_as_a_codeword() -> None:
    """Random bytes must overwhelmingly be rejected, not decoded."""
    rng = np.random.default_rng(99)
    accepted = 0
    for _ in range(200):
        noise = rng.integers(0, 256, rs.BLOCK_BYTES, dtype=np.uint8).tobytes()
        if rs.decode(noise) is not None:
            accepted += 1
    assert accepted <= 2, f"{accepted}/200 noise blocks decoded — locator too permissive"


def test_decoder_never_raises_on_arbitrary_input() -> None:
    """A radio delivers garbage; the decoder must return None, not explode."""
    rng = np.random.default_rng(7)
    for size in (rs.BLOCK_BYTES, 100, 33):
        for _ in range(50):
            noise = rng.integers(0, 256, size, dtype=np.uint8).tobytes()
            rs.decode(noise)  # must not raise
