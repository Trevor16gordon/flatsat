"""Link layer: segmentation, reassembly, contact windows, lossy channels."""

import pytest

from flatsat.comms import comms_config_pb2
from flatsat.comms.framing.ccsds import CcsdsFramer
from flatsat.comms.link import ContactSchedule, Link
from flatsat.comms.phy.loopback import LoopbackModem, reset_channels

S = int(1e9)


def _pair(channel: str, segment_bytes: int = 64, **impairments: float) -> tuple[Link, Link]:
    """Build a flight/ground link pair on one loopback channel.

    Args:
        channel: Loopback channel name.
        segment_bytes: Segment size for both ends.
        impairments: frame_loss / bit_error_rate for the SENDING end.

    Returns:
        Tuple of (flight link, ground link).
    """
    flight_modem = LoopbackModem(
        "flight",
        channel,
        seed=7,
        **impairments,
    )
    ground_modem = LoopbackModem("ground", channel, seed=8)
    framer_options = comms_config_pb2.CcsdsFramerOptions()
    flight = Link(
        flight_modem, CcsdsFramer.from_config(framer_options), segment_bytes=segment_bytes
    )
    ground = Link(
        ground_modem, CcsdsFramer.from_config(framer_options), segment_bytes=segment_bytes
    )
    return flight, ground


@pytest.fixture(autouse=True)
def _isolate_channels() -> None:
    """Every test gets fresh loopback channels."""
    reset_channels()


@pytest.mark.verifies("FSW-LINK-003")
def test_small_message_crosses_intact() -> None:
    flight, ground = _pair("test/link/small")
    flight.enqueue("hal/imu0/sample", b"a small telemetry payload")
    flight.send_pending(0)
    assert ground.poll(0) == [("hal/imu0/sample", b"a small telemetry payload")]


@pytest.mark.verifies("FSW-LINK-003")
def test_large_message_is_segmented_and_reassembled() -> None:
    """A payload many segments long arrives byte-identical — the file path."""
    flight, ground = _pair("test/link/large", segment_bytes=64)
    payload = bytes(range(256)) * 40  # 10 KiB, ~160 segments
    flight.enqueue("uplink/artifact", payload)
    flight.send_pending(0)
    delivered = ground.poll(0)
    assert delivered == [("uplink/artifact", payload)]
    assert flight.frames_sent > 100, "must actually have segmented"


def test_partial_message_is_never_delivered() -> None:
    """Half a model is not a model: incomplete messages are withheld."""
    flight, ground = _pair("test/link/partial", segment_bytes=16)
    flight.enqueue("uplink/artifact", b"x" * 128)
    flight.send_pending(0)
    # Drop the last frame before the receiver ever sees it.
    inbox_before = ground._modem.receive()  # noqa: SLF001 — surgical channel edit
    for block in inbox_before[:-1]:
        ground._framer.feed(block)  # noqa: SLF001
    assert ground.poll(0) == []
    assert ground.messages_delivered == 0


@pytest.mark.verifies("FSW-LINK-004")
def test_store_and_forward_holds_traffic_until_contact() -> None:
    """Out of contact the queue grows; the pass drains it."""
    flight, ground = _pair("test/link/contact")
    flight._contact = ContactSchedule(period_s=10.0, duration_s=2.0)  # noqa: SLF001
    for index in range(5):
        flight.enqueue("health/adcs", f"window {index}".encode())

    assert flight.send_pending(5 * S) == 0, "no pass open at t=5 s"
    assert flight.queued == 5
    assert ground.poll(5 * S) == []

    sent = flight.send_pending(10 * S)  # pass opens at the period boundary
    assert sent == 5
    assert flight.queued == 0
    assert len(ground.poll(10 * S)) == 5


def test_contact_schedule_windows() -> None:
    schedule = ContactSchedule(period_s=10.0, duration_s=2.0)
    assert schedule.in_contact(0)
    assert schedule.in_contact(int(1.9 * S))
    assert not schedule.in_contact(int(2.1 * S))
    assert schedule.in_contact(int(10.5 * S))
    assert ContactSchedule().always_in_contact


def test_queue_is_bounded() -> None:
    """A link that never opens must not exhaust memory."""
    flight, _ = _pair("test/link/bounded")
    flight._queue = type(flight._queue)(maxlen=8)  # noqa: SLF001
    for index in range(20):
        flight.enqueue("health/adcs", str(index).encode())
    assert flight.queued == 8
    assert flight.messages_dropped_queue > 0


@pytest.mark.verifies("FSW-LINK-002")
def test_bit_errors_cost_frames_not_correctness() -> None:
    """On a bad channel: fewer messages arrive, none arrive corrupted."""
    flight, ground = _pair("test/link/noisy", segment_bytes=32, bit_error_rate=2e-3)
    expected = {}
    for index in range(40):
        payload = f"telemetry sample {index:03d}".encode()
        expected[f"health/sample{index}"] = payload
        flight.enqueue(f"health/sample{index}", payload)
    flight.send_pending(0)
    delivered = dict(ground.poll(0))

    assert ground.dropped_frames > 0, "the channel should have corrupted something"
    assert len(delivered) < len(expected), "and some messages should be lost"
    for topic, payload in delivered.items():
        assert payload == expected[topic], "but nothing delivered may be wrong"


def test_frame_loss_costs_only_the_lost_messages() -> None:
    flight, ground = _pair("test/link/lossy", segment_bytes=64, frame_loss=0.3)
    for index in range(30):
        flight.enqueue("health/adcs", f"payload {index}".encode())
    flight.send_pending(0)
    delivered = ground.poll(0)
    assert 0 < len(delivered) < 30
    assert all(payload.startswith(b"payload ") for _, payload in delivered)


# ------------------------------------------------- contact windows and RF --


class _KeyedModem(LoopbackModem):
    """A loopback modem that also remembers keying, like a real radio would.

    The loopback PHY radiates nothing, so it inherits the no-op keying
    default. Contact-window gating still has to be provable without a
    radio, so this records the calls a real transmitter would receive.
    """

    def __init__(self, name: str, channel: str, latency_s: float = 0.0) -> None:
        """Build a keyed loopback modem.

        Args:
            name: Instance name.
            channel: Loopback channel.
            latency_s: Pipeline depth to report to the link.
        """
        super().__init__(name, channel, seed=3)
        self.keyings: list[bool] = []
        self._latency_s = latency_s

    def key_transmitter(self, on: bool) -> None:
        """Record the keying.

        Args:
            on: Whether the carrier was raised.
        """
        self.keyings.append(on)

    @property
    def pipeline_latency_s(self) -> float:
        """Configured pipeline depth."""
        return self._latency_s


def _keyed_link(channel: str, latency_s: float = 0.0) -> tuple[Link, _KeyedModem]:
    """Build a link whose transmitter records keying, on a 10 s / 4 s pass.

    Args:
        channel: Loopback channel name.
        latency_s: Pipeline depth the modem reports.

    Returns:
        Tuple of (link, its modem).
    """
    modem = _KeyedModem("flight", channel, latency_s=latency_s)
    link = Link(
        modem,
        CcsdsFramer(),
        contact=ContactSchedule(period_s=10.0, duration_s=4.0),
    )
    return link, modem


@pytest.mark.verifies("FSW-LINK-004", "FSW-RADIO-002")
def test_carrier_is_keyed_down_when_the_window_closes() -> None:
    """A closed window must stop RADIATION, not merely stop queueing.

    A PHY holding a carrier up between frames goes on transmitting
    through a closed pass unless it is keyed down explicitly.
    """
    link, modem = _keyed_link("test/link/keying")
    link.send_pending(1 * S)  # inside the 4 s pass
    assert modem.keyings == [True]
    link.send_pending(6 * S)  # pass has closed
    assert modem.keyings == [True, False]


@pytest.mark.verifies("FSW-LINK-004")
def test_keying_happens_only_on_transitions() -> None:
    """Keying is an edge, not a level — no attribute write every cycle."""
    link, modem = _keyed_link("test/link/edges")
    for elapsed in (0, 1, 2, 3):
        link.send_pending(elapsed * S)
    assert modem.keyings == [True], "one key-up for one pass"


@pytest.mark.verifies("FSW-LINK-004")
def test_a_link_that_starts_out_of_contact_never_raises_a_carrier() -> None:
    """Silence is the default; nothing radiates before the first pass."""
    link, modem = _keyed_link("test/link/quiet")
    link.send_pending(5 * S)  # between passes
    assert modem.keyings == [False] or modem.keyings == []
    assert True not in modem.keyings


@pytest.mark.verifies("FSW-LINK-004", "FSW-RADIO-002")
def test_frames_that_would_radiate_after_the_window_are_held() -> None:
    """The guard band: a deep pipeline must not spill past a pass.

    With a 0.7 s pipeline, a message handed over at t=3.5 s of a 4 s
    pass would leave the antenna at 4.2 s — outside the window. It waits
    for the next pass instead, which is what the queue is for.
    """
    # Control: the SAME instant with an instant PHY sends, so what
    # follows is the guard band acting and not the window being shut.
    instant, _ = _keyed_link("test/link/guard-control", latency_s=0.0)
    instant.enqueue("hal/imu0/sample", b"late telemetry")
    assert instant.send_pending(int(3.5 * S)) == 1, "t=3.5 s is inside the pass"

    link, modem = _keyed_link("test/link/guard", latency_s=0.7)
    link.enqueue("hal/imu0/sample", b"late telemetry")
    assert link.send_pending(int(3.5 * S)) == 0, "must not radiate past the window"
    assert link.queued == 1, "held, not dropped"
    assert link.messages_held_for_window >= 1

    # Early in the NEXT pass the same message goes out untouched.
    assert link.send_pending(int(10.5 * S)) == 1
    assert link.queued == 0


@pytest.mark.verifies("FSW-RADIO-002")
def test_close_contact_silences_regardless_of_schedule() -> None:
    """Shutdown must not leave a carrier up waiting for a cycle that never comes."""
    link, modem = _keyed_link("test/link/shutdown")
    link.send_pending(1 * S)
    link.close_contact()
    assert modem.keyings[-1] is False
