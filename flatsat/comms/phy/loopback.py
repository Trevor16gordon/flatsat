"""In-process loopback modem: the channel you can make as bad as you like.

Two endpoints sharing a channel name exchange frames through an
in-memory queue — no hardware, no RF, no spectrum. Because it applies
frame loss and bit errors on demand, it is both the CI PHY (every link
test runs anywhere) and a campaign rig: sweep ``bit_error_rate`` and
watch the framing layer's drop count rise, exactly the shape of the
BER-vs-SNR waterfall the real radio produces, with none of the setup.

Deliberately NOT a physics model: no modulation, no noise floor, no
symbol timing. Fidelity about the channel belongs to the real PHY; this
one exists to prove the layers ABOVE the channel behave when the channel
misbehaves.
"""

from __future__ import annotations

import random
import threading

from flatsat.comms import comms_config_pb2
from flatsat.comms.modem import Modem

# channel name -> endpoint name -> inbox
_CHANNELS: dict[str, dict[str, list[bytes]]] = {}
_CHANNELS_LOCK = threading.Lock()


def reset_channels() -> None:
    """Forget every loopback channel (test isolation)."""
    with _CHANNELS_LOCK:
        _CHANNELS.clear()


class LoopbackModem(Modem):
    """Frames handed to one endpoint appear at every other on the channel."""

    def __init__(
        self,
        name: str,
        channel: str,
        frame_loss: float = 0.0,
        bit_error_rate: float = 0.0,
        seed: int | None = None,
    ) -> None:
        """Join a loopback channel.

        Args:
            name: This endpoint's name (sender never receives its own).
            channel: Shared channel name.
            frame_loss: Fraction of transmitted frames silently dropped.
            bit_error_rate: Per-bit flip probability applied to
                delivered frames.
            seed: Corruption RNG seed; None uses the shared generator.
        """
        self._name = name
        self._channel = channel
        self._frame_loss = frame_loss
        self._ber = bit_error_rate
        self._rng = random.Random(seed) if seed is not None else random.Random()
        with _CHANNELS_LOCK:
            _CHANNELS.setdefault(channel, {})[name] = []

    @classmethod
    def from_config(
        cls, name: str, options: comms_config_pb2.LoopbackModemOptions
    ) -> LoopbackModem:
        """Build from vehicle-file modem options.

        Args:
            name: Endpoint name.
            options: Typed options; channel required, impairments default
                to a perfect channel.

        Returns:
            The configured modem.

        Raises:
            ValueError: If no channel is named.
        """
        if not options.channel:
            raise ValueError(f"modem {name!r}: loopback requires a channel name")
        return cls(
            name=name,
            channel=options.channel,
            frame_loss=options.frame_loss if options.HasField("frame_loss") else 0.0,
            bit_error_rate=(options.bit_error_rate if options.HasField("bit_error_rate") else 0.0),
            seed=options.seed if options.HasField("seed") else None,
        )

    def _corrupt(self, frame: bytes) -> bytes:
        """Apply the configured bit-error rate to one frame.

        Args:
            frame: The frame as sent.

        Returns:
            The frame as delivered — possibly with flipped bits.
        """
        if self._ber <= 0.0:
            return frame
        data = bytearray(frame)
        for index in range(len(data)):
            for bit in range(8):
                if self._rng.random() < self._ber:
                    data[index] ^= 1 << bit
        return bytes(data)

    def send(self, frame: bytes) -> int:
        """Deliver a frame to every other endpoint on the channel.

        Args:
            frame: Complete frame bytes.

        Returns:
            Always 0 — a fake channel never faults; it loses and
            corrupts, which is the framing layer's problem to notice.
        """
        if self._rng.random() < self._frame_loss:
            return 0  # the channel ate it; the receiver simply never sees it
        delivered = self._corrupt(frame)
        with _CHANNELS_LOCK:
            for endpoint, inbox in _CHANNELS.get(self._channel, {}).items():
                if endpoint != self._name:
                    inbox.append(delivered)
        return 0

    def receive(self) -> list[bytes]:
        """Take everything delivered since the last call.

        Returns:
            Delivered byte blocks, in arrival order.
        """
        with _CHANNELS_LOCK:
            inbox = _CHANNELS.get(self._channel, {}).get(self._name, [])
            received = list(inbox)
            inbox.clear()
        return received

    def close(self) -> None:
        """Leave the channel."""
        with _CHANNELS_LOCK:
            _CHANNELS.get(self._channel, {}).pop(self._name, None)

    def describe(self) -> list[str]:
        """Describe the simulated channel.

        Returns:
            Lines naming the endpoint and its impairments.
        """
        return [
            f"modem: loopback endpoint={self._name!r} channel={self._channel!r} "
            f"(no RF, no hardware)",
            f"modem: impairments frame_loss={self._frame_loss:g} ber={self._ber:g}",
        ]
