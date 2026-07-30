"""CCSDS pseudo-randomizer: guarantee transition density regardless of payload.

A GMSK receiver recovers its symbol clock from TRANSITIONS in the
signal. Our demodulator runs a Mueller & Müller timing loop, which is a
feedback loop driven by changes between samples — so a long run of
identical bits gives it nothing to servo against and its timing estimate
drifts. Measured on the cabled loopback (2026-07-29):

    payload bytes(48)        -- 48 zero bytes --   3/40 frames recovered
    payload bytes(range(48)) -- varied         -- 40/40 frames recovered

Same link, same instant. The only difference was payload content. This
matters because the traffic here is protobuf, which is full of zero
runs, so real telemetry is closer to the bad case than to the good one.

The second problem is DC. In GMSK a constant bit run is a constant
frequency offset, which puts energy at baseband centre — exactly where
this radio is weakest, and where the AD936x DC-correction loops we
enable will actively fight the signal. That is the same failure that
took BER from 1.8e-1 to 0.00 when the carrier was offset-tuned; a
low-entropy payload recreates it from the data side.

The fix is the standard one (CCSDS 131.0-B, section 9): XOR the frame
with a pseudo-random sequence so the wire sees high-entropy bits
whatever the payload was. Notes on the choices:

  * **Frame-synchronous (additive), not self-synchronizing.** The
    sequence restarts at every frame and is a pure XOR, so one channel
    bit error stays one bit error. A self-synchronizing scrambler feeds
    the LFSR from the data and needs no alignment, but multiplies each
    error across several bits — which is why CCSDS specifies additive.
  * **The sync marker is NOT randomized.** Frame detection has to work
    before derandomization can start, so the ASM stays in the clear and
    the PN sequence begins at the first byte after it.
  * This is PROBABILISTIC, not a guarantee. Randomizing makes long runs
    improbable; it does not bound them. Line codes (8b/10b, 64b/66b)
    give hard bounds on run length and disparity at a bandwidth cost.
    If we ever need a guarantee rather than good odds, that is the
    upgrade — see the module docstring in ccsds.py for where it'd sit.

The generator is h(x) = x^8 + x^7 + x^5 + x^3 + 1, all eight stages
preset to 1 at the start of every frame.
"""

from __future__ import annotations

_SEED = 0xFF
# Bit positions XORed to form the feedback, for h(x) = x^8+x^7+x^5+x^3+1
# under the shift-left / output-bit-7 convention used below. Verified
# against the sequence the standard publishes: FF 48 0E C0 9A ...
_TAP_SHIFTS = (0, 2, 4, 7)
_CACHE_GROWTH = 4096


def _generate(length: int) -> bytes:
    """Run the LFSR to produce ``length`` bytes of the PN sequence.

    Args:
        length: Bytes required.

    Returns:
        The first ``length`` bytes of the CCSDS pseudo-random sequence.
    """
    state = _SEED
    out = bytearray()
    for _ in range(length):
        byte = 0
        for _ in range(8):
            byte = (byte << 1) | ((state >> 7) & 1)
            feedback = 0
            for shift in _TAP_SHIFTS:
                feedback ^= state >> shift
            state = ((state << 1) | (feedback & 1)) & 0xFF
        out.append(byte)
    return bytes(out)


# Generated once and extended on demand: the sequence is fixed, so there
# is no reason to re-run the LFSR per frame.
_SEQUENCE = _generate(_CACHE_GROWTH)


def sequence(length: int) -> bytes:
    """Return at least ``length`` bytes of the PN sequence.

    Args:
        length: Bytes required.

    Returns:
        The sequence prefix, from a cache that grows as needed.
    """
    global _SEQUENCE  # noqa: PLW0603 — a grow-only cache of a constant
    if length > len(_SEQUENCE):
        _SEQUENCE = _generate(max(length, len(_SEQUENCE) + _CACHE_GROWTH))
    return _SEQUENCE


def apply(data: bytes) -> bytes:
    """Randomize or derandomize a frame body.

    XOR is its own inverse, so one function serves both directions —
    which also means the two ends cannot disagree about which is which.

    Args:
        data: Frame bytes AFTER the sync marker, starting at sequence
            position zero.

    Returns:
        The bytes XORed with the PN sequence.
    """
    pn = sequence(len(data))
    return bytes(byte ^ pn[index] for index, byte in enumerate(data))
