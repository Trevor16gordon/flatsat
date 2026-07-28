"""Control loop app: composition, health provenance, command flagging."""

import pytest
import zenoh

from flatsat.apps.control_loop import ControlLoop, LoopReport
from flatsat.control.attitude import control_options_pb2
from flatsat.core.config import ControlEntry
from flatsat.core.registry import (
    get_controller_class,
    get_estimator_class,
    get_guidance_class,
)
from flatsat.msgs import health_pb2


def _entry() -> ControlEntry:
    return ControlEntry(
        strategy="rate_damping",
        objective="constant_rate",
        estimator="passthrough",
        rate_hz=100.0,
        input_topic="test/health/in",
        output_topic="test/health/out",
        stale_after_s=0.05,
        options=control_options_pb2.RateDampingOptions(kp=0.02, kd=0.005),
        objective_options=control_options_pb2.ConstantRateOptions(),
        estimator_options=control_options_pb2.PassthroughOptions(),
    )


def _build_loop(session: zenoh.Session, entry: ControlEntry) -> ControlLoop:
    return ControlLoop(
        session,
        entry,
        get_controller_class(entry.strategy).from_config(entry.options),
        get_guidance_class(entry.objective).from_config(entry.objective_options),
        get_estimator_class(entry.estimator).from_config(entry.estimator_options),
        vehicle_name="test-vehicle",
        config_checksum="deadbeef1234",
    )


@pytest.mark.verifies("FSW-ADCS-010", "FSW-CFG-002")
def test_loop_health_carries_provenance_and_counters() -> None:
    entry = _entry()
    session = zenoh.open(zenoh.Config())
    loop = _build_loop(session, entry)
    loop.scheduling = "SCHED_FIFO priority 80"
    loop.cpu_affinity = "3"

    report = LoopReport(
        cycles=6000,
        wakeup_lateness_us=[10.0, 20.0, 30.0],
        exec_time_us=[5.0, 6.0, 7.0],
        stale_cycles=12,
        saturated_cycles=3,
    )
    msg = report.to_proto(loop, loop.scheduling, loop.cpu_affinity)
    loop.close()
    session.close()

    # Provenance: a recorded window must trace to what produced it.
    assert msg.vehicle == "test-vehicle"
    assert msg.config_checksum == "deadbeef1234"
    assert msg.strategy == "rate_damping"
    assert msg.objective == "constant_rate"
    assert msg.scheduling == "SCHED_FIFO priority 80"
    assert msg.cpu_affinity == "3"
    # Counters and distributions survive serialization intact.
    back = health_pb2.LoopHealth.FromString(msg.SerializeToString())
    assert back.window_cycles == 6000
    assert back.stale_cycles == 12
    assert back.saturated_cycles == 3
    assert back.wakeup_lateness_us.max == pytest.approx(30.0)
    assert back.exec_time_us.count == 3
