"""Basilisk magnetometer driver: truth field in, honest device data out."""

import time
from collections.abc import Iterator

import pytest
import zenoh

from flatsat.core.config import load_magnetometer_spec
from flatsat.hardware.drivers.basilisk_magnetometer import BasiliskMagnetometerDriver
from flatsat.msgs import hal_pb2, sim_pb2

TRUTH_TOPIC = "test/mag/truth"


@pytest.fixture(name="truth_session")
def fixture_truth_session() -> Iterator[zenoh.Session]:
    """One zenoh session standing in for the plant, closed on teardown.

    Yields:
        The open session.
    """
    session = zenoh.open(zenoh.Config())
    yield session
    session.close()


def _driver(stale_after_s: float = 0.5) -> BasiliskMagnetometerDriver:
    spec, _ = load_magnetometer_spec()
    return BasiliskMagnetometerDriver(
        spec=spec, truth_topic=TRUTH_TOPIC, stale_after_s=stale_after_s, seed=3
    )


def test_no_truth_reads_stale_flagged_zeros() -> None:
    driver = _driver()
    try:
        msg, flags = driver.read()
    finally:
        driver.close()
    assert flags & hal_pb2.VALIDITY_FLAG_STALE
    assert isinstance(msg, hal_pb2.MagnetometerSample)
    assert (msg.mag_x_t, msg.mag_y_t, msg.mag_z_t) == (0.0, 0.0, 0.0)


def test_truth_field_flows_through_the_device_model(truth_session: zenoh.Session) -> None:
    driver = _driver()
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)  # discovery
        truth = sim_pb2.TruthState()
        truth.mag_field_x_t = 2.6e-5
        truth.mag_field_y_t = -1.0e-5
        truth.mag_field_z_t = 5.0e-6
        pub.put(truth.SerializeToString())
        deadline = time.monotonic() + 5.0
        flags = int(hal_pb2.VALIDITY_FLAG_STALE)
        msg = hal_pb2.MagnetometerSample()
        while flags & hal_pb2.VALIDITY_FLAG_STALE and time.monotonic() < deadline:
            time.sleep(0.02)
            msg, flags = driver.read()  # type: ignore[assignment]
    finally:
        driver.close()
    assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
    # mag0 noise is ~15 nT; the 26 uT field must come through at the
    # device's fidelity, not verbatim and not mangled.
    assert msg.mag_x_t == pytest.approx(2.6e-5, abs=1e-7)
    assert msg.mag_y_t == pytest.approx(-1.0e-5, abs=1e-7)
    assert msg.mag_z_t == pytest.approx(5.0e-6, abs=1e-7)


def test_stale_truth_goes_dark_honestly(truth_session: zenoh.Session) -> None:
    driver = _driver(stale_after_s=0.2)
    pub = truth_session.declare_publisher(TRUTH_TOPIC)
    try:
        time.sleep(0.5)
        truth = sim_pb2.TruthState()
        truth.mag_field_x_t = 2.6e-5
        pub.put(truth.SerializeToString())
        deadline = time.monotonic() + 5.0
        flags = int(hal_pb2.VALIDITY_FLAG_STALE)
        while flags & hal_pb2.VALIDITY_FLAG_STALE and time.monotonic() < deadline:
            time.sleep(0.02)
            _, flags = driver.read()
        assert not flags & hal_pb2.VALIDITY_FLAG_STALE, "truth never arrived"
        time.sleep(0.4)  # exceed the stale threshold with no fresh truth
        msg, flags = driver.read()
    finally:
        driver.close()
    assert flags & hal_pb2.VALIDITY_FLAG_STALE
    assert isinstance(msg, hal_pb2.MagnetometerSample)
    assert (msg.mag_x_t, msg.mag_y_t, msg.mag_z_t) == (0.0, 0.0, 0.0)
