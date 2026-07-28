"""What a framer IS: delimit payloads, and reject corruption honestly.

Contract for implementers:
  * ``frame()`` wraps one payload into bytes the PHY can carry.
  * ``feed()`` accepts arbitrary byte blocks — partial frames, garbage
    between frames, concatenated frames — and returns whatever complete,
    integrity-checked payloads it could recover. It MUST NOT raise: a
    corrupt frame is DROPPED and COUNTED, never delivered. Silence about
    corruption would let the link fabricate telemetry, which is worse
    than losing it.
  * The deframer is stateful (it buffers partial input) and must
    resynchronize after garbage without losing later frames.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class Framer(ABC):
    """Wraps payloads for the channel and recovers them from its noise."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> Framer:
        """Build a framer from its typed options message.

        Args:
            options: The oneof block the vehicle file filled.

        Returns:
            A ready framer.
        """

    @abstractmethod
    def frame(self, payload: bytes) -> bytes:
        """Wrap one payload for transmission.

        Args:
            payload: The bytes to carry.

        Returns:
            The complete frame.
        """

    @abstractmethod
    def feed(self, data: bytes) -> list[bytes]:
        """Absorb received bytes and recover complete payloads.

        Args:
            data: Arbitrary bytes from the modem.

        Returns:
            Payloads recovered from this and previously buffered input.
        """

    @property
    @abstractmethod
    def dropped_frames(self) -> int:
        """Frames rejected for failing their integrity check."""

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines naming the framing in force.
        """
        return [f"framing: {type(self).__name__}"]
