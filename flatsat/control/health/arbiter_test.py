"""FDIR arbiter over the real bus: detection, safing, one-way authority."""

import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import zenoh

from flatsat.control.health import fdir_config_pb2
from flatsat.control.health.arbiter import Fdir
from flatsat.mode import mode_config_pb2
from flatsat.mode.manager import ModeManager
from flatsat.msgs import hal_pb2, mode_pb2

# Test-only key bases; production keys belong to the live bench.
BASE = "test/fdir/sys/mode"
TOPIC = "test/fdir/arb/imu"


def _config(window_s: float = 1.0, min_trip_s: float = 0.3) -> fdir_config_pb2.FdirConfig:
    return fdir_config_pb2.FdirConfig(
        rules=[
            fdir_config_pb2.FdirRule(
                name="imu_stale",
                min_trip_s=min_trip_s,
                flag_ratio=fdir_config_pb2.FlagRatioRule(
                    topic=TOPIC, flag_mask=8, window_s=window_s, max_ratio=0.5
                ),
            )
        ]
    )


@pytest.fixture(name="rig")
def fixture_rig(tmp_path: Path) -> Iterator[tuple[ModeManager, Fdir, zenoh.Session]]:
    """A manager (booted NOMINAL) + arbiter + publisher session.

    Yields:
        Tuple of (manager, fdir, publisher session).
    """
    manager_session = zenoh.open(zenoh.Config())
    pub_session = zenoh.open(zenoh.Config())
    marker = tmp_path / "marker"
    marker.touch()  # clean boot -> NOMINAL, so there is safety to lose
    entry = mode_config_pb2.ModeConfig(
        apps=["fdir"],
        ack_timeout_s=0.5,
        min_dwell_s=0.05,
        clean_shutdown_marker=str(marker),
    )
    manager = ModeManager(entry, manager_session, base_topic=BASE)
    fdir = Fdir(_config(), manager_session, base_topic=BASE)
    time.sleep(0.5)  # discovery
    yield manager, fdir, pub_session
    fdir.stop()
    fdir.close()
    manager.close()
    manager_session.close()
    pub_session.close()


def _publish(session: zenoh.Session, validity: int, count: int, spacing_s: float = 0.02) -> None:
    pub = session.declare_publisher(TOPIC)
    for _ in range(count):
        msg = hal_pb2.ImuSample()
        msg.header.validity = validity
        pub.put(msg.SerializeToString())
        time.sleep(spacing_s)


@pytest.mark.verifies("FSW-FDIR-002")
def test_sustained_fault_ends_in_safe(rig: tuple[ModeManager, Fdir, zenoh.Session]) -> None:
    manager, fdir, pub_session = rig
    assert manager.state().mode == mode_pb2.SYSTEM_MODE_NOMINAL

    _publish(pub_session, validity=0, count=10)  # healthy baseline
    assert fdir.tick() == [], "healthy traffic must not trip"

    _publish(pub_session, validity=8, count=50)  # sustained STALE (1 s @ 50)
    deadline = time.monotonic() + 5.0
    while manager.state().mode != mode_pb2.SYSTEM_MODE_SAFE and time.monotonic() < deadline:
        fdir.tick()
        time.sleep(0.05)

    state = manager.state()
    assert state.mode == mode_pb2.SYSTEM_MODE_SAFE, "FDIR never safed the vehicle"
    assert state.reason == "fdir: imu_stale", "source prefixed exactly once"
    assert fdir.safe_requests >= 1


@pytest.mark.verifies("FSW-FDIR-003")
def test_fdir_holds_fire_when_already_safe(rig: tuple[ModeManager, Fdir, zenoh.Session]) -> None:
    manager, fdir, pub_session = rig
    manager.request(
        mode_pb2.ModeRequest(source="fdir", requested=mode_pb2.SYSTEM_MODE_SAFE, reason="pre")
    )
    time.sleep(0.3)  # let the broadcast reach fdir's mode client
    _publish(pub_session, validity=8, count=50)
    for _ in range(10):
        fdir.tick()
        time.sleep(0.05)
    assert fdir.safe_requests == 0, "already SAFE — no requests to make"
    assert fdir.tripped_rules(), "the rule itself should still show tripped"


@pytest.mark.verifies("FSW-FDIR-004")
def test_fdir_health_reports_tripped_rules(rig: tuple[ModeManager, Fdir, zenoh.Session]) -> None:
    manager, fdir, pub_session = rig
    _publish(pub_session, validity=8, count=50)
    deadline = time.monotonic() + 3.0
    while not fdir.tripped_rules() and time.monotonic() < deadline:
        time.sleep(0.05)
    health = fdir.publish_health()
    assert health.rules == 1
    assert list(health.tripped) == ["imu_stale"]
