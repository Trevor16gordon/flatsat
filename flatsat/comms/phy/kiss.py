"""KISS over TCP: the seam where somebody else's radio plugs into this link.

This modem does no DSP at all. It speaks KISS — the framing amateur and
cubesat ground software has used for decades — to an external process
that owns the actual radio. That process can be a gr-satellites
flowgraph, a GNU Radio Companion design, Direwolf, a hardware TNC, or a
future flight modem written by somebody who has never seen this repo.

Why this seam and not in-process GNU Radio blocks: linking their blocks
into our flowgraph is faster but marries us to one GNU Radio version and
ABI, and on this build a minimal ``gr.sync_block`` segfaults the
interpreter. A byte-framed socket costs a context switch and buys the
ability to swap an entire PHY — theirs or ours — with a config edit and
no rebuild. It is also the interface real ground stations already
expose, so interoperability is the default rather than a project.

TWO WAYS TO USE IT, and the difference is which layer owns framing:

  * **Tunnel** (pair with ``ccsds``): our own frames — sync marker,
    length, CRC, randomizer — ride inside KISS payloads and the far side
    is a dumb bit pipe. Works with any KISS TNC.
  * **Native** (pair with a pass-through framer): the far side deframes,
    so each KISS payload already IS one application frame. This is how
    you use somebody else's standard deframer, and it is the reason the
    framer is a separate registry entry from the modem.

KISS itself (the 1997 spec, unchanged since): frames are delimited by
FEND, and any FEND or FESC inside the payload is escaped so the
delimiter is unambiguous. The first byte after FEND is a command; the
low nibble 0 means "data", the high nibble is a port number.

RF SAFETY: this modem cannot radiate on its own — it has no radio. The
transmit gate therefore lives in the process at the far end of the
socket, and ``describe()`` says so rather than implying a gate that is
not ours to hold.
"""

from __future__ import annotations

import socket
import threading

from flatsat.comms import comms_config_pb2
from flatsat.comms.modem import Modem
from flatsat.msgs import hal_pb2

FEND = 0xC0  # frame delimiter
FESC = 0xDB  # escape
TFEND = 0xDC  # escaped FEND
TFESC = 0xDD  # escaped FESC
DATA_FRAME = 0x00  # command byte: port 0, data
DEFAULT_PORT = 8001  # the port gr-satellites and Direwolf conventionally serve
_READ_BYTES = 1 << 16
_RX_LIMIT = 1 << 22  # bound the buffer if the far side never sends a delimiter


def encode(payload: bytes, port: int = 0) -> bytes:
    """Wrap a payload in a KISS data frame.

    Args:
        payload: Bytes to send.
        port: KISS port number (the command byte's high nibble).

    Returns:
        The framed bytes, delimiters included.
    """
    escaped = bytearray()
    for byte in payload:
        if byte == FEND:
            escaped += bytes([FESC, TFEND])
        elif byte == FESC:
            escaped += bytes([FESC, TFESC])
        else:
            escaped.append(byte)
    return bytes([FEND, (port << 4) | DATA_FRAME]) + bytes(escaped) + bytes([FEND])


def decode(buffer: bytearray) -> list[bytes]:
    """Pull every complete KISS frame out of a buffer, in place.

    Consumed bytes are removed from ``buffer``; a trailing partial frame
    is left for the next call, since a socket read can split anywhere.

    Args:
        buffer: Accumulated received bytes, modified in place.

    Returns:
        Payloads of the complete data frames found, unescaped.
    """
    frames: list[bytes] = []
    while True:
        try:
            start = buffer.index(FEND)
        except ValueError:
            buffer.clear()  # no delimiter at all: nothing recoverable here
            return frames
        try:
            end = buffer.index(FEND, start + 1)
        except ValueError:
            del buffer[:start]  # keep the partial frame for next time
            return frames

        raw = bytes(buffer[start + 1 : end])
        # Consume up to but NOT past the closing delimiter: in KISS a
        # FEND ends one frame and may equally BEGIN the next, so senders
        # that share a single delimiter between frames are legal. Eating
        # it would swallow the start of the following frame — which it
        # did, silently, until a padding test caught it.
        del buffer[:end]
        # Back-to-back delimiters are padding, which many TNCs emit.
        if not raw or raw[0] & 0x0F != DATA_FRAME:
            continue  # padding, or a TNC command (TX delay etc.), not traffic

        payload = bytearray()
        escaping = False
        for byte in raw[1:]:
            if escaping:
                payload.append(FEND if byte == TFEND else FESC if byte == TFESC else byte)
                escaping = False
            elif byte == FESC:
                escaping = True
            else:
                payload.append(byte)
        # A zero-length data frame carries nothing any layer above wants,
        # and is indistinguishable in intent from padding.
        if payload:
            frames.append(bytes(payload))


class KissModem(Modem):
    """Frames in and out of an external radio process over a KISS socket."""

    def __init__(
        self,
        name: str,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        kiss_port: int = 0,
        connect_timeout_s: float = 2.0,
    ) -> None:
        """Configure the socket without opening it.

        Args:
            name: Instance name.
            host: Where the radio process is listening.
            port: TCP port of that process.
            kiss_port: KISS port number inside the protocol (not TCP).
            connect_timeout_s: How long to wait for the far side.
        """
        self._name = name
        self._host = host
        self._port = port
        self._kiss_port = kiss_port
        self._timeout_s = connect_timeout_s
        self._socket: socket.socket | None = None
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._start_error = ""
        self._sent = 0
        self._received = 0

    @classmethod
    def from_config(cls, name: str, options: comms_config_pb2.KissModemOptions) -> KissModem:
        """Build from vehicle-file modem options.

        Args:
            name: Instance name.
            options: Typed options; ``host`` and ``port`` default.

        Returns:
            The configured modem.
        """
        return cls(
            name=name,
            host=options.host or "127.0.0.1",
            port=options.port or DEFAULT_PORT,
            kiss_port=options.kiss_port,
        )

    @property
    def start_error(self) -> str:
        """Why the socket failed to open, if it did."""
        return self._start_error

    @property
    def frames_sent(self) -> int:
        """Frames handed to the far side."""
        return self._sent

    @property
    def frames_received(self) -> int:
        """Frames recovered from the far side."""
        return self._received

    def _ensure_open(self) -> bool:
        """Connect once, remembering failure rather than retrying hard.

        Returns:
            True when the socket is usable.
        """
        if self._socket is not None:
            return True
        if self._start_error:
            return False
        try:
            sock = socket.create_connection((self._host, self._port), timeout=self._timeout_s)
            sock.setblocking(False)  # receive() must never block the caller
            self._socket = sock
        except OSError as exc:
            self._start_error = f"{type(exc).__name__}: {exc}"
            return False
        return True

    def send(self, frame: bytes) -> int:
        """Hand one frame to the external radio.

        Args:
            frame: Complete frame bytes.

        Returns:
            0 when written; ``VALIDITY_FLAG_COMM`` when the far side is
            unreachable. Never raises.
        """
        if not self._ensure_open() or self._socket is None:
            return int(hal_pb2.VALIDITY_FLAG_COMM)
        try:
            with self._lock:
                self._socket.sendall(encode(frame, self._kiss_port))
        except OSError:
            return int(hal_pb2.VALIDITY_FLAG_COMM)
        self._sent += 1
        return 0

    def receive(self) -> list[bytes]:
        """Collect whatever the external radio has delivered.

        Returns:
            Complete frame payloads; empty when nothing arrived or the
            far side is unreachable. Never raises.
        """
        if not self._ensure_open() or self._socket is None:
            return []
        while True:
            try:
                block = self._socket.recv(_READ_BYTES)
            except BlockingIOError:
                break  # non-blocking socket drained: the normal exit
            except OSError:
                break
            if not block:
                break  # far side closed
            self._rx.extend(block)
            if len(self._rx) > _RX_LIMIT:
                del self._rx[: len(self._rx) - _RX_LIMIT]
        frames = decode(self._rx)
        self._received += len(frames)
        return frames

    def close(self) -> None:
        """Close the socket; the far side owns the radio and its silence."""
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

    def describe(self) -> list[str]:
        """Describe the seam, including who holds the transmit gate.

        Returns:
            Lines naming the endpoint and the safety boundary.
        """
        return [
            f"modem: kiss over tcp to {self._host}:{self._port} (kiss port {self._kiss_port})",
            "modem: no radio here — the TRANSMIT GATE belongs to the process at the far end",
        ]
