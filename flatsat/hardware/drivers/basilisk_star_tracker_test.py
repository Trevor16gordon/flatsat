"""Basilisk star tracker driver: truth attitude in, honest MRP out."""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.core.config import load_star_tracker_spec
from flatsat.hardware.drivers.basilisk_star_tracker import BasiliskStarTrackerDriver
from flatsat.msgs import hal_pb2, sim_pb2

TRUTH_TOPIC = "test/st/truth"


@pytest.fixture(name="truth_session")
def fixture_truth_session() -> Iterator[zenoh.Session]:
    """One zenoh session standing in for the plant, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def _driver(stale_after_s: float = 0.5) -> BasiliskStarTrackerDriver:
    spec, _ = load_star_tracker_spec()
    return BasiliskStarTrackerDriver(
        spec=spec, truth_topic=TRUTH_TOPIC, stale_after_s=stale_after_s, seed=3
    )


def _read_fresh(driver: BasiliskStarTrackerDriver) -> tuple[hal_pb2.StarTrackerSample, int]:
    deadline = time.monotonic() + 5.0
    flags = int(hal_pb2.VALIDITY_FLAG_STALE)
    msg = hal_pb2.StarTrackerSample()
    while flags & hal_pb2.VALIDITY_FLAG_STALE and time.monotonic() < deadline:
        time.sleep(0.02)
        msg, flags = driver.read()  # type: ignore[assignment]
    return msg, flags


def test_no_truth_reads_stale_flagged_invalid() -> None:
    driver = _driver()
    try:
        msg, flags = driver.read()
    finally:
        driver.close()
    assert flags & hal_pb2.VALIDITY_FLAG_STALE
    assert isinstance(msg, hal_pb2.StarTrackerSample)
    assert not msg.star_valid


def test_truth_attitude_flows_through_the_device_model(truth_session: zenoh.Session) -> None:
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)  # discovery
        truth = sim_pb2.TruthState()
        truth.sigma_x, truth.sigma_y, truth.sigma_z = 0.1, -0.2, 0.15
        # No sun, no position: darkness, nothing to blind on.
        pub.put(truth.SerializeToString())
        msg, flags = _read_fresh(driver)
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    assert msg.star_valid
    # st0 noise is arcseconds: the attitude comes through at the
    # device's fidelity, not verbatim and not mangled.
    assert msg.sigma_x == pytest.approx(0.1, abs=1e-3)
    assert msg.sigma_y == pytest.approx(-0.2, abs=1e-3)
    assert msg.sigma_z == pytest.approx(0.15, abs=1e-3)


def test_sun_in_the_boresight_blinds_honestly(truth_session: zenoh.Session) -> None:
    """A fresh blinded read is UNFLAGGED invalid — condition, not fault."""
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)
        truth = sim_pb2.TruthState()
        truth.sigma_x = 0.1
        # st0 boresight is -z; put the sun straight down it.
        truth.sun_x, truth.sun_y, truth.sun_z = 0.0, 0.0, -1.0
        pub.put(truth.SerializeToString())
        msg, flags = _read_fresh(driver)
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    assert not msg.star_valid
    assert (msg.sigma_x, msg.sigma_y, msg.sigma_z) == (0.0, 0.0, 0.0)
