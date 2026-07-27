"""HIL contract: the simulated actuator's safety behavior.

Imports the ground-side sink directly (no Basilisk needed) — the property
under test is the command-timeout rule, not the physics.
"""

import time

import pytest
import zenoh

from flatsat.msgs import adcs_pb2
from flatsat.sim.basilisk_hil import TorqueSink

TOPIC = "test/hil/wheel_torque"


@pytest.mark.verifies("FSW-SIM-002")
def test_torque_sink_zeroes_when_commands_stop() -> None:
    """An actuator must not keep flying a dead controller's last order.

    Learned from a real run: when the flight loop exited, the sim held the
    final torque and spun the vehicle back up.
    """
    session = zenoh.open(zenoh.Config())
    sink = TorqueSink(session, TOPIC, timeout_s=0.2)
    pub = session.declare_publisher(TOPIC)

    cmd = adcs_pb2.WheelTorqueCommand()
    cmd.torque_x_n_m = 0.01
    cmd.torque_y_n_m = -0.02
    cmd.torque_z_n_m = 0.03
    pub.put(cmd.SerializeToString())

    deadline = time.monotonic() + 5.0
    while sink.commands_received == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert sink.commands_received >= 1, "command never arrived"
    assert sink.latest() == pytest.approx((0.01, -0.02, 0.03))

    time.sleep(0.35)  # exceed the timeout with no further commands
    assert sink.latest() == (0.0, 0.0, 0.0), "stale command must not keep being applied"
    session.close()
