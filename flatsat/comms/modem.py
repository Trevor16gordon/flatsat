"""What a modem IS: payload bytes ⇄ RF, and nothing else.

A modem owns one radio (or one fake) and answers two questions: "send
these frame bytes" and "what frames arrived". It knows nothing about
framing, segmentation, topics, contact windows, or the bus — the link
service supplies all of that. That split is what lets a hand-built
classical PHY, a stock GNU Radio modem, and a learned receiver be
swapped by configuration and compared under identical framing.

Contract for implementers:
  * ``send()`` and ``receive()`` MUST NOT raise. A radio fault is
    reported as validity flags (flag and forward) — a raise would stop
    the link service, which downstream cannot distinguish from the
    process dying.
  * ``receive()`` returns whatever whole frames arrived since the last
    call, and never blocks indefinitely.
  * **RF safety**: an implementation that can radiate MUST refuse to
    transmit unless its options carry an explicit acknowledgement, and
    must say so plainly in ``describe()``. Receiving needs no gate.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class Modem(ABC):
    """Owns one radio: frame bytes out, frame bytes in."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        name: str,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> Modem:
        """Build a modem from its typed options message.

        Args:
            name: Instance name (also the telemetry source).
            options: This modem's options (the oneof block the vehicle
                file filled — see comms_config.proto).

        Returns:
            A ready modem.
        """

    @abstractmethod
    def send(self, frame: bytes) -> int:
        """Transmit one framed packet.

        Args:
            frame: Complete frame bytes from the framer.

        Returns:
            Validity flags (0 == sent). A modem that is not permitted to
            transmit returns a flag rather than raising or radiating.
        """

    @abstractmethod
    def receive(self) -> list[bytes]:
        """Collect whatever arrived since the last call.

        Returns:
            Zero or more raw byte blocks. These are NOT guaranteed to be
            frame-aligned — the framer resynchronizes.
        """

    def close(self) -> None:
        """Release radio resources. Local fakes inherit the no-op."""
        return

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs.

        Returns:
            Lines naming the radio and, critically, whether it may
            transmit.
        """
        return [f"modem: {type(self).__name__}"]
