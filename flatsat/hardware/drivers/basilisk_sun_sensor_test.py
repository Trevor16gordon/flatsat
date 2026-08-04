"""Basilisk sun sensor driver: truth sun in, honest CSS reading out."""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.core.config import load_sun_sensor_spec
from flatsat.hardware.drivers.basilisk_sun_sensor import BasiliskSunSensorDriver
from flatsat.msgs import hal_pb2, sim_pb2

TRUTH_TOPIC = "test/css/truth"


@pytest.fixture(name="truth_session")
def fixture_truth_session() -> Iterator[zenoh.Session]:
    """One zenoh session standing in for the plant, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def _driver(stale_after_s: float = 0.5) -> BasiliskSunSensorDriver:
    spec, _ = load_sun_sensor_spec()
    return BasiliskSunSensorDriver(
        spec=spec, truth_topic=TRUTH_TOPIC, stale_after_s=stale_after_s, seed=3
    )


def _read_fresh(driver: BasiliskSunSensorDriver) -> tuple[hal_pb2.SunSensorSample, int]:
    deadline = time.monotonic() + 5.0
    flags = int(hal_pb2.VALIDITY_FLAG_STALE)
    msg = hal_pb2.SunSensorSample()
    while flags & hal_pb2.VALIDITY_FLAG_STALE and time.monotonic() < deadline:
        time.sleep(0.02)
        msg, flags = driver.read()  # type: ignore[assignment]
    return msg, flags


def test_no_truth_reads_stale_flagged_darkness() -> None:
    driver = _driver()
    try:
        msg, flags = driver.read()
    finally:
        driver.close()
    assert flags & hal_pb2.VALIDITY_FLAG_STALE
    assert isinstance(msg, hal_pb2.SunSensorSample)
    assert not msg.sun_visible


def test_truth_sun_flows_through_the_device_model(truth_session: zenoh.Session) -> None:
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)  # discovery
        truth = sim_pb2.TruthState()
        truth.sun_x, truth.sun_y, truth.sun_z = 0.6, 0.0, 0.8
        truth.in_eclipse = False
        pub.put(truth.SerializeToString())
        msg, flags = _read_fresh(driver)
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    assert msg.sun_visible
    # css0 angular noise is 0.02 rad (~1.1 deg): the direction must come
    # through at the device's fidelity, not verbatim and not mangled.
    assert msg.sun_x == pytest.approx(0.6, abs=0.1)
    assert msg.sun_z == pytest.approx(0.8, abs=0.1)


def test_fresh_eclipse_is_valid_darkness(truth_session: zenoh.Session) -> None:
    """Darkness is a measurement; only silence is a fault."""
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)
        truth = sim_pb2.TruthState()
        truth.sun_x, truth.sun_y, truth.sun_z = 0.6, 0.0, 0.8
        truth.in_eclipse = True
        pub.put(truth.SerializeToString())
        msg, flags = _read_fresh(driver)
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    assert not msg.sun_visible
    assert (msg.sun_x, msg.sun_y, msg.sun_z) == (0.0, 0.0, 0.0)


def test_orbitless_truth_reports_darkness(truth_session: zenoh.Session) -> None:
    """A plant with no orbit publishes a zero sun: valid, not visible."""
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)
        pub.put(sim_pb2.TruthState().SerializeToString())
        msg, flags = _read_fresh(driver)
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    assert not msg.sun_visible
