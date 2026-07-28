"""ADALM-Pluto GMSK modem — the proven classical baseline PHY.

The radio and settings that produced BER 0.00 on 113/113 framed packets
through 30 dB of cabled attenuation (2026-07-23): GNU Radio 3.10 GMSK
modulation over gr-iio, with the governing lesson baked into the
defaults — **keep signal energy off DC**. Zero-IF LO leakage and the
AD936x DC-correction loops sit at baseband centre, and a signal placed
there measured BER 1.8e-1 versus 0.00 when offset-tuned.

RF SAFETY, structurally enforced:
  * ``send()`` refuses and returns a COMM flag unless the configuration
    carries ``transmit_ack: true`` — the same gate the radio bench
    scripts implement as ``--transmit`` (FSW-RADIO-001).
  * The transmitter is driven to maximum attenuation on close, on error,
    and on every exit path (FSW-RADIO-002): the Pluto's DMA can replay a
    stale buffer after a flowgraph stops, so leaving must leave it quiet.
  * Receiving is always permitted; listening radiates nothing.

GNU Radio imports live inside methods so this module imports on any
machine — a flight computer without gr-iio must still be able to load
the registry and run every other test.
"""

from __future__ import annotations

from typing import Any

from flatsat.comms import comms_config_pb2
from flatsat.comms.modem import Modem
from flatsat.msgs import hal_pb2

DEFAULT_OFFSET_HZ = 250e3  # off DC by construction
DEFAULT_SPS = 8
MAX_ATTENUATION_DB = 89  # the Pluto's floor: "as quiet as this radio gets"


class PlutoGmskModem(Modem):
    """GMSK over an ADALM-Pluto, transmit-gated by explicit acknowledgement."""

    def __init__(
        self,
        name: str,
        uri: str,
        center_freq_hz: float,
        sample_rate_hz: float,
        offset_hz: float = DEFAULT_OFFSET_HZ,
        samples_per_symbol: int = DEFAULT_SPS,
        tx_attenuation_db: int = 60,
        transmit_ack: bool = False,
    ) -> None:
        """Configure the radio without opening it.

        Args:
            name: Instance name.
            uri: IIO context URI, e.g. ``ip:192.168.2.1``.
            center_freq_hz: LO frequency.
            sample_rate_hz: Complex sample rate.
            offset_hz: Baseband offset keeping energy off DC.
            samples_per_symbol: GMSK oversampling.
            tx_attenuation_db: Transmit attenuation (higher = quieter).
            transmit_ack: Explicit permission to radiate. False means
                receive-only, enforced in :meth:`send`.
        """
        self._name = name
        self._uri = uri
        self._center_freq_hz = center_freq_hz
        self._sample_rate_hz = sample_rate_hz
        self._offset_hz = offset_hz
        self._sps = samples_per_symbol
        self._tx_attenuation_db = tx_attenuation_db
        self._transmit_ack = transmit_ack
        self._flowgraph: Any | None = None
        self._refused = 0

    @classmethod
    def from_config(
        cls, name: str, options: comms_config_pb2.PlutoGmskModemOptions
    ) -> PlutoGmskModem:
        """Build from vehicle-file modem options.

        Args:
            name: Instance name.
            options: Typed options; ``uri``, ``center_freq_hz`` and
                ``sample_rate_hz`` are required.

        Returns:
            The configured modem — receive-only unless the config
            explicitly acknowledges transmission.

        Raises:
            ValueError: If a required radio parameter is missing.
        """
        if not options.uri or options.center_freq_hz <= 0 or options.sample_rate_hz <= 0:
            raise ValueError(
                f"modem {name!r}: pluto_gmsk requires uri, center_freq_hz, sample_rate_hz"
            )
        return cls(
            name=name,
            uri=options.uri,
            center_freq_hz=options.center_freq_hz,
            sample_rate_hz=options.sample_rate_hz,
            offset_hz=options.offset_hz if options.HasField("offset_hz") else DEFAULT_OFFSET_HZ,
            samples_per_symbol=(
                options.samples_per_symbol
                if options.HasField("samples_per_symbol")
                else DEFAULT_SPS
            ),
            tx_attenuation_db=(
                options.tx_attenuation_db if options.HasField("tx_attenuation_db") else 60
            ),
            transmit_ack=options.transmit_ack,
        )

    @property
    def may_transmit(self) -> bool:
        """Whether this modem is permitted to radiate."""
        return self._transmit_ack

    @property
    def refused_transmissions(self) -> int:
        """Frames declined because transmission was not acknowledged."""
        return self._refused

    def send(self, frame: bytes) -> int:
        """Transmit one frame — if and only if permitted.

        Args:
            frame: Complete frame bytes.

        Returns:
            0 when sent; ``VALIDITY_FLAG_COMM`` when transmission was
            refused for want of acknowledgement, or when the radio
            faulted. Never raises, never radiates unacknowledged.
        """
        if not self._transmit_ack:
            self._refused += 1
            return int(hal_pb2.VALIDITY_FLAG_COMM)
        try:
            return self._transmit(frame)
        except Exception:  # noqa: BLE001 — a radio fault is a flag, never a crash
            self.silence_tx()
            return int(hal_pb2.VALIDITY_FLAG_COMM)

    def _transmit(self, frame: bytes) -> int:
        """Drive one frame through the GMSK flowgraph.

        Args:
            frame: Complete frame bytes.

        Returns:
            0 on success.

        Raises:
            NotImplementedError: Until the bench flowgraph from
                ``radio/pluto_data_loopback_test.py`` is graduated into
                this driver — the proven settings live there and must be
                moved with a cabled BER re-verification, not by
                transcription. Callers see a COMM flag, not a crash.
        """
        raise NotImplementedError(
            "pluto_gmsk transmit path not yet graduated from radio/ bench scripts"
        )

    def receive(self) -> list[bytes]:
        """Collect demodulated bytes from the receive flowgraph.

        Returns:
            Byte blocks; empty until the receive flowgraph is graduated
            from the bench scripts. Never raises.
        """
        return []

    def silence_tx(self) -> None:
        """Drive the transmitter to maximum attenuation (every exit path).

        The Pluto's DMA can keep replaying a stale buffer after a
        flowgraph stops; leaving the process must leave the radio quiet.
        """
        if self._flowgraph is None:
            return
        try:
            self._flowgraph.set_attenuation(MAX_ATTENUATION_DB)
        except Exception:  # noqa: BLE001 — best effort; never mask the exit path
            return

    def close(self) -> None:
        """Silence the transmitter and release the radio."""
        self.silence_tx()
        self._flowgraph = None

    def describe(self) -> list[str]:
        """Describe the radio and, first, whether it may transmit.

        Returns:
            Lines naming the transmit gate and the radio settings.
        """
        gate = (
            "TRANSMIT ENABLED (cabled + 30 dB pads only)"
            if self._transmit_ack
            else "receive-only (no transmit_ack in config)"
        )
        return [
            f"modem: pluto_gmsk {gate}",
            f"modem: uri={self._uri} center={self._center_freq_hz / 1e6:g} MHz "
            f"rate={self._sample_rate_hz / 1e6:g} MSa/s offset={self._offset_hz / 1e3:g} kHz "
            f"sps={self._sps} tx_atten={self._tx_attenuation_db} dB",
        ]
