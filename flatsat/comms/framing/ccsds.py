"""CCSDS-style framing: attached sync marker, length, payload, CRC-32.

Wire layout, big-endian:

    | 0x1ACFFC1D |<------ randomized (CCSDS 131.0-B) ------>|
    |    ASM     | length (u16) | payload | CRC-32 (u32)    |

Everything after the marker is XORed with a pseudo-random sequence so
the PHY sees high-entropy bits whatever the payload contained. This is
not cosmetic: a low-transition payload starves the demodulator's timing
loop, and measured on hardware a zero-filled payload recovered 3 of 40
frames where a varied one recovered 40 of 40. See ``randomizer.py`` for
the reasoning and the standard's rationale. Randomization is
unconditional, because two ends that disagree about it cannot talk and a
configuration knob would only create that possibility.

If bounded run length is ever needed rather than merely probable, the
upgrade is a line code (8b/10b, 64b/66b) applied here in place of the
randomizer, at a bandwidth cost.

The sync marker is the real CCSDS attached sync marker, which is what
lets the deframer find frame boundaries in a stream that starts
mid-garbage — the normal condition on a channel that was noise a moment
ago. The CRC covers length + payload: a corrupted frame is dropped and
counted, never delivered.

Resynchronization is deliberate and defensive: on a CRC failure the
scanner advances ONE byte past the failed marker rather than skipping
the claimed length, because a corrupted length field cannot be trusted
to tell you where the next frame starts.
"""

from __future__ import annotations

import struct
import zlib

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing import randomizer
from flatsat.comms.framing.framer import Framer

SYNC = b"\x1a\xcf\xfc\x1d"
_HEADER = struct.Struct(">H")
_CRC = struct.Struct(">I")
DEFAULT_MAX_PAYLOAD = 4096


class CcsdsFramer(Framer):
    """Sync-marker framing with a CRC-32 integrity check."""

    def __init__(self, max_payload_bytes: int = DEFAULT_MAX_PAYLOAD) -> None:
        """Configure the length sanity bound.

        Args:
            max_payload_bytes: Frames claiming more than this are
                rejected without buffering — a corrupted length field
                must not make the deframer wait for megabytes that will
                never come.
        """
        self._max_payload = max_payload_bytes
        self._buffer = bytearray()
        self._dropped = 0

    @classmethod
    def from_config(cls, options: comms_config_pb2.CcsdsFramerOptions) -> CcsdsFramer:
        """Build from vehicle-file framer options.

        Args:
            options: Typed options; ``max_payload_bytes`` defaults.

        Returns:
            The configured framer.
        """
        limit = (
            options.max_payload_bytes
            if options.HasField("max_payload_bytes")
            else DEFAULT_MAX_PAYLOAD
        )
        return cls(max_payload_bytes=limit)

    @property
    def dropped_frames(self) -> int:
        """Frames rejected for failing CRC or the length bound."""
        return self._dropped

    def frame(self, payload: bytes) -> bytes:
        """Wrap one payload.

        Args:
            payload: The bytes to carry.

        Returns:
            Sync + length + payload + CRC-32.

        Raises:
            ValueError: If the payload exceeds the configured bound —
                a programming error on the SEND side, caught loudly
                here rather than producing frames no receiver accepts.
        """
        if len(payload) > self._max_payload:
            raise ValueError(f"payload {len(payload)} B exceeds max {self._max_payload} B")
        body = _HEADER.pack(len(payload)) + payload
        tail = body + _CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)
        # Randomize everything after the marker, CRC included: the
        # receiver derandomizes first, so the CRC still judges the
        # payload as sent. The marker stays in the clear because frame
        # detection has to happen before derandomization can start.
        return SYNC + randomizer.apply(tail)

    def feed(self, data: bytes) -> list[bytes]:
        """Absorb bytes and recover every complete, valid frame.

        Args:
            data: Arbitrary bytes from the modem.

        Returns:
            Recovered payloads, in wire order.
        """
        self._buffer.extend(data)
        payloads: list[bytes] = []
        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                # Keep a marker-length tail: the sync may straddle reads.
                if len(self._buffer) > len(SYNC):
                    del self._buffer[: -len(SYNC)]
                return payloads
            if start > 0:
                del self._buffer[:start]  # discard leading garbage
            if len(self._buffer) < len(SYNC) + _HEADER.size:
                return payloads  # header not here yet

            offset = len(SYNC)
            # The length field is randomized like everything else, so it
            # must be recovered before it can be trusted. The sequence is
            # positional from the first byte after the marker, so the
            # header can be derandomized on its own to learn how much
            # more of the frame to expect.
            (length,) = _HEADER.unpack(
                randomizer.apply(bytes(self._buffer[offset : offset + _HEADER.size]))
            )
            if length > self._max_payload:
                self._dropped += 1
                del self._buffer[: len(SYNC)]  # bogus length: resync past this marker
                continue
            end = offset + _HEADER.size + length + _CRC.size
            if len(self._buffer) < end:
                return payloads  # frame still arriving

            clear = randomizer.apply(bytes(self._buffer[offset:end]))
            body = clear[: _HEADER.size + length]
            (claimed,) = _CRC.unpack_from(clear, _HEADER.size + length)
            if claimed == (zlib.crc32(body) & 0xFFFFFFFF):
                payloads.append(body[_HEADER.size :])
                del self._buffer[:end]
            else:
                self._dropped += 1
                # A corrupted length cannot say where the next frame is:
                # advance one marker only and rescan.
                del self._buffer[: len(SYNC)]

    def describe(self) -> list[str]:
        """Describe the framing in force.

        Returns:
            One line naming the format and bound.
        """
        return [f"framing: ccsds sync+len+crc32, max payload {self._max_payload} B"]
