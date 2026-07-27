"""Integration tests: the actuator daemon's protections on the real bus.

The two daemon-owned protections — stale-command zeroing and the mounting
projection — are exactly what these prove, end to end through zenoh with
the registered sim wheel underneath.
"""

import threading
import time

import pytest
import zenoh

from flatsat.apps.actuator_daemon import ActuatorDaemon
from flatsat.core.config import ActuatorEntry, Mounting
from flatsat.core.health import health_topic
from flatsat.core.registry import get_actuator_class
from flatsat.msgs import adcs_pb2, hal_pb2, health_pb2

RECV_TIMEOUT_S = 5.0


def _entry(name: str, axis: tuple[float, float, float] = (1.0, 0.0, 0.0)) -> ActuatorEntry:
    return ActuatorEntry(
        name=name,
        driver="sim_reaction_wheel",
        command_topic=f"test/act/{name}/cmd",
        state_topic=f"test/act/{name}/state",
        rate_hz=50.0,
        stale_zero_s=0.15,
        mounting=Mounting(position_m=(0.0, 0.0, 0.0), axis=axis),
        options={"device": "config/devices/wheel0.toml"},
    )


def _command(torque: tuple[float, float, float]) -> bytes:
    cmd = adcs_pb2.WheelTorqueCommand()
    cmd.torque_x_n_m, cmd.torque_y_n_m, cmd.torque_z_n_m = torque
    payload: bytes = cmd.SerializeToString()
    return payload


def _wait_for(predicate: object, timeout_s: float = RECV_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():  # type: ignore[operator]
            return True
        time.sleep(0.02)
    return False


@pytest.fixture(name="sessions")
def fixture_sessions() -> object:
    """Open publisher/daemon zenoh sessions, closed on teardown.

    Yields:
        Tuple of (command/observer session, daemon session).
    """
    a = zenoh.open(zenoh.Config())
    b = zenoh.open(zenoh.Config())
    yield a, b
    a.close()
    b.close()


def _run_daemon(
    entry: ActuatorEntry, session: zenoh.Session
) -> tuple[ActuatorDaemon, threading.Thread]:
    driver = get_actuator_class(entry.driver).from_config(entry.name, entry.options)
    daemon = ActuatorDaemon(entry, driver, session)
    thread = threading.Thread(target=lambda: daemon.run(health_every_s=0.4), daemon=True)
    thread.start()
    return daemon, thread


@pytest.mark.verifies("FSW-ACT-004")
def test_daemon_publishes_wheel_state_on_cadence(
    sessions: tuple[zenoh.Session, zenoh.Session],
) -> None:
    pub_session, daemon_session = sessions
    entry = _entry("wheel_state_test")
    states: list[hal_pb2.WheelState] = []
    sub = pub_session.declare_subscriber(
        entry.state_topic,
        lambda s: states.append(hal_pb2.WheelState.FromString(bytes(s.payload.to_bytes()))),
    )
    time.sleep(0.5)  # discovery

    daemon, thread = _run_daemon(entry, daemon_session)
    assert _wait_for(lambda: len(states) >= 5), "no wheel state on the bus"
    daemon.stop()
    thread.join(timeout=2.0)
    daemon.close()
    sub.undeclare()

    seqs = [s.header.seq for s in states]
    assert seqs[0] == 1 and seqs == sorted(seqs)
    for s in states:
        assert s.header.source == entry.name


@pytest.mark.verifies("FSW-ACT-001")
def test_daemon_zeroes_when_commands_stop(sessions: tuple[zenoh.Session, zenoh.Session]) -> None:
    """The flight-side twin of the sim sink's rule, now in the flight path."""
    pub_session, daemon_session = sessions
    entry = _entry("wheel_stale_test")
    states: list[hal_pb2.WheelState] = []
    sub = pub_session.declare_subscriber(
        entry.state_topic,
        lambda s: states.append(hal_pb2.WheelState.FromString(bytes(s.payload.to_bytes()))),
    )
    cmd_pub = pub_session.declare_publisher(entry.command_topic)
    time.sleep(0.5)

    daemon, thread = _run_daemon(entry, daemon_session)
    # Live phase: command torque along the wheel axis, see it applied.
    for _ in range(10):
        cmd_pub.put(_command((0.02, 0.0, 0.0)))
        time.sleep(0.03)
    assert _wait_for(lambda: any(s.torque_n_m != 0.0 for s in states)), (
        "commanded torque never reached the wheel"
    )

    # Silence: stop commanding, exceed stale_zero_s; the wheel must zero.
    time.sleep(entry.stale_zero_s + 0.3)
    states.clear()
    assert _wait_for(lambda: len(states) >= 3), "state publishing stopped with the commands"
    daemon.stop()
    thread.join(timeout=2.0)
    daemon.close()
    sub.undeclare()

    assert all(s.torque_n_m == 0.0 for s in states), (
        "an actuator must not keep flying a dead controller's last order"
    )


@pytest.mark.verifies("FSW-ACT-002")
def test_daemon_projects_through_mounting(sessions: tuple[zenoh.Session, zenoh.Session]) -> None:
    """A z-axis wheel commanded about x applies nothing; about z, everything."""
    pub_session, daemon_session = sessions
    entry = _entry("wheel_proj_test", axis=(0.0, 0.0, 1.0))
    states: list[hal_pb2.WheelState] = []
    sub = pub_session.declare_subscriber(
        entry.state_topic,
        lambda s: states.append(hal_pb2.WheelState.FromString(bytes(s.payload.to_bytes()))),
    )
    cmd_pub = pub_session.declare_publisher(entry.command_topic)
    time.sleep(0.5)

    daemon, thread = _run_daemon(entry, daemon_session)

    # Phase 1: torque about x only — orthogonal to this wheel's axis.
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        cmd_pub.put(_command((0.02, 0.0, 0.0)))
        time.sleep(0.03)
    phase1 = list(states)
    assert phase1, "no state during orthogonal phase"
    assert all(s.torque_n_m == pytest.approx(0.0) for s in phase1)

    # Phase 2: torque about z — fully along the axis.
    states.clear()
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        cmd_pub.put(_command((0.0, 0.0, 0.03)))
        time.sleep(0.03)
    applied = [s.torque_n_m for s in states if s.torque_n_m != 0.0]
    daemon.stop()
    thread.join(timeout=2.0)
    daemon.close()
    sub.undeclare()

    assert applied, "axis-aligned command never applied"
    assert all(t == pytest.approx(0.03) for t in applied)


@pytest.mark.verifies("FSW-ACT-006")
def test_actuator_daemon_publishes_health(sessions: tuple[zenoh.Session, zenoh.Session]) -> None:
    pub_session, daemon_session = sessions
    entry = _entry("wheel_health_test")
    received: list[health_pb2.ActuatorHealth] = []
    got = threading.Event()

    def on_health(sample: zenoh.Sample) -> None:
        received.append(health_pb2.ActuatorHealth.FromString(bytes(sample.payload.to_bytes())))
        got.set()

    sub = pub_session.declare_subscriber(health_topic(entry.name), on_health)
    time.sleep(0.5)

    daemon, thread = _run_daemon(entry, daemon_session)
    got.wait(RECV_TIMEOUT_S)
    daemon.stop()
    thread.join(timeout=2.0)
    daemon.close()
    sub.undeclare()

    assert received, "no ActuatorHealth published"
    health = received[0]
    assert health.driver == "sim_reaction_wheel"
    assert health.rate_hz == pytest.approx(entry.rate_hz)
    assert health.window_cycles > 0
    # No commands were ever sent — every cycle must show the protection on.
    assert health.stale_zeroed_cycles == health.window_cycles
