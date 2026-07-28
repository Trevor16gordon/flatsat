"""HIL contract: the bridge's safety sink and the shared-plant property.

Imports the bridge module directly (no Basilisk needed) — the properties
under test are the feedback-timeout rule and plant-from-vehicle-file
derivation, not the physics.
"""

import time

import pytest
import zenoh

from flatsat.core.config import load_vehicle
from flatsat.msgs import sim_pb2
from flatsat.sim.basilisk_hil import WheelTorqueSink, plant_from_vehicle


@pytest.mark.verifies("FSW-SIM-002")
def test_wheel_sink_zeroes_when_feedback_stops() -> None:
    """Defense in depth behind the daemon's own stale-command zeroing.

    Learned from a real run: when the flight loop exited, the sim held the
    final torque and spun the vehicle back up.
    """
    session = zenoh.open(zenoh.Config())
    sink = WheelTorqueSink(session, "wheel_sink_test", timeout_s=0.2)
    pub = session.declare_publisher("sim/wheel/wheel_sink_test/torque")

    msg = sim_pb2.WheelAxisTorque()
    msg.wheel = "wheel_sink_test"
    msg.torque_n_m = 0.01
    pub.put(msg.SerializeToString())

    deadline = time.monotonic() + 5.0
    while sink.messages_received == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert sink.messages_received >= 1, "feedback never arrived"
    assert sink.latest() == pytest.approx(0.01)

    time.sleep(0.35)  # exceed the timeout with no further feedback
    assert sink.latest() == 0.0, "stale feedback must not keep torquing the plant"
    session.close()


def test_sink_with_no_feedback_ever_is_zero() -> None:
    session = zenoh.open(zenoh.Config())
    sink = WheelTorqueSink(session, "wheel_never_test", timeout_s=10.0)
    assert sink.latest() == 0.0
    session.close()


@pytest.mark.verifies("FSW-SIM-003")
def test_plant_is_built_from_the_vehicle_file() -> None:
    """Sim/flight agreement by construction: same file, same numbers."""
    vehicle = load_vehicle()
    mass_kg, inertia, wheels = plant_from_vehicle(vehicle)

    body = vehicle.require_body()
    flat = list(body.inertia_kg_m2)
    assert mass_kg == body.mass_kg
    assert inertia == [flat[0:3], flat[3:6], flat[6:9]]
    assert wheels == [(a.name, tuple(a.mounting.axis)) for a in vehicle.actuators]
    assert wheels, "default vehicle must declare at least one wheel"
