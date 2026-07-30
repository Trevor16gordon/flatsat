"""Reed-Solomon over GF(256): correct bit errors instead of dropping the frame.

Today a frame with a single bad bit fails CRC and is thrown away — the
link's only recovery is for the ground to send the whole artifact again.
Repeated cabled runs drop 0-3 frames per hundred that way, and every one
of them was nearly intact.

Reed-Solomon is the right code for this receiver, and the reason is
worth stating because it constrains the choice more than performance
does: ``gmsk_demod`` ends in a binary slicer, so what arrives here is
HARD bits with no confidence information. Convolutional codes with
Viterbi decoding and modern LDPC both want soft decisions and give up
most of their advantage without them. RS works on hard symbols, is
byte-oriented like everything else at this layer, and is the CCSDS
outer code for exactly this reason.

RS(255, 223) corrects up to 16 corrupted bytes per 255-byte codeword —
and because it counts BYTES, a burst of up to 128 consecutive bad bits
inside one byte-run costs the same as one isolated error. That suits a
channel whose errors arrive in bursts, which ours does.

Deliberate deviation, documented so nobody assumes otherwise: this uses
the CONVENTIONAL GF(256) field polynomial 0x11D in the standard basis.
CCSDS specifies 0x187 with a dual-basis representation, so codewords
here are NOT bit-compatible with a real CCSDS ground station — they are
structurally the same code (n=255, k=223, t=16), not the same bytes.
Both ends of this link are ours, so the difference costs nothing; if
interoperating with real CCSDS equipment ever matters, swap the field
constants and the basis transform, not the algorithm.

The implementation is the textbook one: syndromes, Berlekamp-Massey for
the error locator, Chien search for the positions, Forney for the
magnitudes.
"""

from __future__ import annotations

FIELD_POLY = 0x11D  # x^8 + x^4 + x^3 + x^2 + 1, the conventional GF(256) field
BLOCK_BYTES = 255  # codeword length
DATA_BYTES = 223  # payload per codeword
PARITY_BYTES = BLOCK_BYTES - DATA_BYTES  # 32 -> corrects up to 16 byte errors

_EXP = [0] * 512  # antilog table, doubled so index arithmetic never wraps
_LOG = [0] * 256


def _build_tables() -> None:
    """Fill the GF(256) log and antilog tables."""
    value = 1
    for power in range(255):
        _EXP[power] = value
        _LOG[value] = power
        value <<= 1
        if value & 0x100:  # degree 8 reached: reduce by the field polynomial
            value ^= FIELD_POLY
    # The upper half repeats, so _EXP[a + b] is valid for any a, b < 255
    # without a modulo in the inner loops.
    for power in range(255, 512):
        _EXP[power] = _EXP[power - 255]


_build_tables()


def _mul(a: int, b: int) -> int:
    """Multiply in GF(256).

    Args:
        a: First factor.
        b: Second factor.

    Returns:
        The product.
    """
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def _inv(a: int) -> int:
    """Multiplicative inverse in GF(256).

    Args:
        a: Element to invert; must be non-zero.

    Returns:
        The inverse.
    """
    return _EXP[255 - _LOG[a]]


def _div(a: int, b: int) -> int:
    """Divide in GF(256).

    Args:
        a: Numerator.
        b: Denominator; must be non-zero.

    Returns:
        The quotient.
    """
    if a == 0:
        return 0
    return _EXP[(_LOG[a] + 255 - _LOG[b]) % 255]


def _pow(a: int, power: int) -> int:
    """Raise a field element to an integer power.

    Args:
        a: Base; must be non-zero.
        power: Exponent, may be negative.

    Returns:
        ``a ** power`` in GF(256).
    """
    return _EXP[(_LOG[a] * power) % 255]


# Polynomials are coefficient lists with index 0 holding the HIGHEST
# power, which is the convention the algorithms below assume throughout.


def _poly_add(p: list[int], q: list[int]) -> list[int]:
    """Add two polynomials (XOR, in characteristic 2).

    Args:
        p: First polynomial.
        q: Second polynomial.

    Returns:
        The sum.
    """
    out = [0] * max(len(p), len(q))
    for index, coef in enumerate(p):
        out[index + len(out) - len(p)] = coef
    for index, coef in enumerate(q):
        out[index + len(out) - len(q)] ^= coef
    return out


def _poly_scale(p: list[int], factor: int) -> list[int]:
    """Multiply a polynomial by a scalar.

    Args:
        p: The polynomial.
        factor: Field element to scale by.

    Returns:
        The scaled polynomial.
    """
    return [_mul(coef, factor) for coef in p]


def _poly_mul(p: list[int], q: list[int]) -> list[int]:
    """Multiply two polynomials.

    Args:
        p: First polynomial.
        q: Second polynomial.

    Returns:
        The product.
    """
    out = [0] * (len(p) + len(q) - 1)
    for j, qj in enumerate(q):
        if qj == 0:
            continue
        log_qj = _LOG[qj]
        for i, pi in enumerate(p):
            if pi:
                out[i + j] ^= _EXP[_LOG[pi] + log_qj]
    return out


def _poly_eval(p: list[int], x: int) -> int:
    """Evaluate a polynomial at x by Horner's method.

    Args:
        p: The polynomial.
        x: Point to evaluate at.

    Returns:
        p(x).
    """
    y = p[0]
    for coef in p[1:]:
        y = _mul(y, x) ^ coef
    return y


def _generator(parity: int) -> list[int]:
    """Build the RS generator polynomial for a given parity length.

    Args:
        parity: Number of parity symbols.

    Returns:
        The product of (x - alpha^i) for i in range(parity).
    """
    g = [1]
    for power in range(parity):
        g = _poly_mul(g, [1, _EXP[power]])
    return g


_GENERATORS: dict[int, list[int]] = {}


def _generator_cached(parity: int) -> list[int]:
    """Return the generator polynomial, building it once per parity length.

    Args:
        parity: Number of parity symbols.

    Returns:
        The generator polynomial.
    """
    if parity not in _GENERATORS:
        _GENERATORS[parity] = _generator(parity)
    return _GENERATORS[parity]


def encode(data: bytes, parity: int = PARITY_BYTES) -> bytes:
    """Append RS parity to a data block.

    Args:
        data: Up to ``255 - parity`` bytes. Shorter blocks are fine — a
            shortened code decodes identically as long as both ends
            agree on the length.
        parity: Parity symbols to append.

    Returns:
        ``data`` followed by ``parity`` parity bytes.

    Raises:
        ValueError: If the block would exceed 255 bytes.
    """
    if len(data) + parity > BLOCK_BYTES:
        raise ValueError(f"block {len(data)}+{parity} exceeds {BLOCK_BYTES} bytes")
    gen = _generator_cached(parity)
    out = bytearray(data) + bytearray(parity)
    for i in range(len(data)):
        coef = out[i]
        if coef:  # synthetic division; gen[0] is 1 so it is skipped
            log_coef = _LOG[coef]
            for j in range(1, len(gen)):
                out[i + j] ^= _EXP[log_coef + _LOG[gen[j]]]
    out[: len(data)] = data  # the loop used this region as scratch
    return bytes(out)


def _syndromes(block: list[int], parity: int) -> list[int]:
    """Evaluate the received polynomial at the code's roots.

    Args:
        block: Received codeword coefficients.
        parity: Number of parity symbols.

    Returns:
        The syndrome sequence; all zero means no detected error.
    """
    return [_poly_eval(block, _EXP[i]) for i in range(parity)]


def _error_locator(synd: list[int], parity: int) -> list[int]:
    """Berlekamp-Massey: find the error locator polynomial.

    Args:
        synd: Syndromes.
        parity: Number of parity symbols.

    Returns:
        The error locator polynomial, highest power first.
    """
    locator = [1]
    previous = [1]
    for i in range(parity):
        delta = synd[i]
        for j in range(1, len(locator)):
            delta ^= _mul(locator[len(locator) - 1 - j], synd[i - j])
        previous = previous + [0]
        if delta != 0:
            if len(previous) > len(locator):
                scaled = _poly_scale(previous, delta)
                previous = _poly_scale(locator, _inv(delta))
                locator = scaled
            locator = _poly_add(locator, _poly_scale(previous, delta))
    while locator and locator[0] == 0:
        del locator[0]
    return locator


def _error_positions(locator: list[int], length: int) -> list[int] | None:
    """Chien search: find where the errors are.

    Args:
        locator: Error locator polynomial.
        length: Codeword length in symbols.

    Returns:
        Error positions as indices into the codeword, or None if the
        locator's degree and its root count disagree — which means more
        errors than this code can place.
    """
    errors = len(locator) - 1
    # A root at alpha^-i marks an error i symbols from the END, so the
    # index is counted back from the last symbol.
    positions = [length - 1 - i for i in range(length) if _poly_eval(locator, _EXP[255 - i]) == 0]
    if len(positions) != errors:
        return None
    return positions


def _errata_locator(positions: list[int]) -> list[int]:
    """Build the locator polynomial for known error positions.

    Args:
        positions: Coefficient positions of the errors.

    Returns:
        The errata locator polynomial.
    """
    out = [1]
    for position in positions:
        out = _poly_mul(out, _poly_add([1], [_EXP[position], 0]))
    return out


def _error_evaluator(synd: list[int], locator: list[int], degree: int) -> list[int]:
    """Compute the error evaluator polynomial, Omega = S(x)Lambda(x) mod x^(degree+1).

    Keeping the low ``degree + 1`` coefficients is the reduction modulo
    x^(degree+1); dropping one term instead yields magnitudes that are
    wrong for every error count, including a single byte error.

    Args:
        synd: Syndromes, highest power first.
        locator: Errata locator polynomial.
        degree: Degree of the locator.

    Returns:
        The error evaluator polynomial.
    """
    product = _poly_mul(synd, locator)
    return product[len(product) - (degree + 1) :]


def decode(block: bytes, parity: int = PARITY_BYTES) -> tuple[bytes, int] | None:
    """Correct a received codeword.

    Args:
        block: Received codeword, data followed by parity.
        parity: Number of parity symbols.

    Returns:
        Tuple of (corrected data, number of symbols corrected), or None
        when the codeword carries more errors than the code can correct.
        None means "do not trust these bytes" — the caller drops the
        frame exactly as it would have on a CRC failure.
    """
    received = list(block)
    synd = _syndromes(received, parity)
    if max(synd) == 0:
        return bytes(received[: len(received) - parity]), 0

    locator = _error_locator(synd, parity)
    if len(locator) - 1 > parity // 2:
        return None  # more errors than t; correcting would invent data

    positions = _error_positions(locator, len(received))
    if positions is None:
        return None

    # Forney: magnitude at each located position.
    coef_positions = [len(received) - 1 - p for p in positions]
    errata = _errata_locator(coef_positions)
    # NOTE the leading zero, which is not decoration. Berlekamp-Massey
    # above wants the syndromes as S_0..S_31, but Forney's numerator is
    # X_i * Omega(X_i^-1), and that identity holds only when the
    # syndrome polynomial is shifted up by one power. Feeding the same
    # list to both yields a plausible locator, the correct error
    # POSITIONS, and wrong magnitudes at every one of them.
    evaluator = _error_evaluator(([0] + synd)[::-1], errata, len(errata) - 1)[::-1]

    locations = [_pow(2, coef) for coef in coef_positions]
    correction = [0] * len(received)
    for index, location in enumerate(locations):
        inverse = _inv(location)
        # The formal derivative of the locator, evaluated the cheap way:
        # the product of (1 - X_j / X_i) over the other error locations.
        derivative = 1
        for other, elsewhere in enumerate(locations):
            if other != index:
                derivative = _mul(derivative, 1 ^ _mul(inverse, elsewhere))
        numerator = _mul(location, _poly_eval(evaluator[::-1], inverse))
        if derivative == 0:
            return None  # degenerate locator; refuse rather than guess
        correction[positions[index]] = _div(numerator, derivative)

    corrected = [value ^ correction[i] for i, value in enumerate(received)]
    # Trust nothing until the syndromes agree: a miscorrection is worse
    # than a drop, because it delivers plausible wrong bytes upward.
    if max(_syndromes(corrected, parity)) != 0:
        return None
    return bytes(corrected[: len(corrected) - parity]), len(positions)
