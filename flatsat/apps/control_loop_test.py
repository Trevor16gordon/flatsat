"""Control loop app: composition, health provenance, command flagging."""

import pytest
import zenoh

from flatsat import vehicle_pb2
from flatsat.apps.control_loop import ControlLoop, LoopReport
from flatsat.control.attitude import control_options_pb2
from flatsat.core.registry import (
    get_controller_class,
    get_estimator_class,
    get_guidance_class,
)
from flatsat.msgs import health_pb2


def _entry() -> vehicle_pb2.ControlConfig:
    return vehicle_pb2.ControlConfig(
        rate_hz=100.0,
        input_topic="test/health/in",
        output_topic="test/health/out",
        stale_after_s=0.05,
        rate_damping=control_options_pb2.RateDampingOptions(kp=0.02, kd=0.005),
        constant_rate=control_options_pb2.ConstantRateOptions(),
        passthrough=control_options_pb2.PassthroughOptions(),
    )


def _build_loop(session: zenoh.Session, entry: vehicle_pb2.ControlConfig) -> ControlLoop:
    return ControlLoop(
        session,
        entry,
        get_controller_class("rate_damping").from_config(entry.rate_damping),
        get_guidance_class("constant_rate").from_config(entry.constant_rate),
        get_estimator_class("passthrough").from_config(entry.passthrough),
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


# ------------------------------------------------ magnetic / dual output --


def _dump_entry() -> vehicle_pb2.ControlConfig:
    return vehicle_pb2.ControlConfig(
        rate_hz=50.0,
        input_topic="test/dual/in",
        mag_input_topic="test/dual/mag",
        output_topic="test/dual/torque",
        dipole_output_topic="test/dual/dipole",
        stale_after_s=0.5,
        momentum_dump=control_options_pb2.MomentumDumpOptions(
            kp=0.05, kd=0.005, max_torque_n_m=0.05, dump_gain=0.15, max_dipole_a_m2=25.0
        ),
        constant_rate=control_options_pb2.ConstantRateOptions(),
        passthrough=control_options_pb2.PassthroughOptions(),
    )


def _build_dump_loop(
    session: zenoh.Session,
    entry: vehicle_pb2.ControlConfig,
    wheels: list[tuple[str, tuple[float, float, float]]] | None,
) -> ControlLoop:
    return ControlLoop(
        session,
        entry,
        get_controller_class("momentum_dump").from_config(entry.momentum_dump),
        get_guidance_class("constant_rate").from_config(entry.constant_rate),
        get_estimator_class("passthrough").from_config(entry.passthrough),
        wheels=wheels,
    )


def test_dual_output_strategy_refuses_a_missing_dipole_topic() -> None:
    """A dump law whose dipole silently went nowhere must not start."""
    entry = _dump_entry()
    entry.dipole_output_topic = ""
    session = zenoh.open(zenoh.Config())
    try:
        with pytest.raises(ValueError, match="dipole_output_topic"):
            _build_dump_loop(session, entry, wheels=None)
    finally:
        session.close()


def test_dual_output_publishes_torque_and_dipole() -> None:
    """One step of a momentum_dump loop puts BOTH commands on the bus."""
    import time as time_module

    from flatsat.msgs import adcs_pb2, hal_pb2

    entry = _dump_entry()
    wheels: list[tuple[str, tuple[float, float, float]]] = [
        ("test/dual/wheel_x/state", (1.0, 0.0, 0.0)),
        ("test/dual/wheel_z/state", (0.0, 0.0, 1.0)),
    ]
    observer = zenoh.open(zenoh.Config())
    loop_session = zenoh.open(zenoh.Config())
    torques: list[adcs_pb2.WheelTorqueCommand] = []
    dipoles: list[adcs_pb2.DipoleCommand] = []
    subs = [
        observer.declare_subscriber(
            entry.output_topic,
            lambda s: torques.append(
                adcs_pb2.WheelTorqueCommand.FromString(bytes(s.payload.to_bytes()))
            ),
        ),
        observer.declare_subscriber(
            entry.dipole_output_topic,
            lambda s: dipoles.append(
                adcs_pb2.DipoleCommand.FromString(bytes(s.payload.to_bytes()))
            ),
        ),
    ]
    loop = _build_dump_loop(loop_session, entry, wheels=wheels)
    time_module.sleep(0.5)  # discovery
    try:
        # Inject fresh inputs directly: this test pins the STEP, not the
        # subscriber plumbing (the daemons' side is mission-tested).
        now = time_module.monotonic_ns()
        imu = hal_pb2.ImuSample()
        imu.gyro_x_rad_s = 0.1
        mag = hal_pb2.MagnetometerSample()
        mag.mag_y_t = 2.6e-5
        deadline = time_module.monotonic() + 5.0
        seq = 0
        # Step repeatedly: the observer session may still be discovering
        # the loop's publishers when the first commands go out.
        while (not torques or not dipoles) and time_module.monotonic() < deadline:
            now = time_module.monotonic_ns()
            with loop._lock:  # noqa: SLF001 — white-box injection
                loop._latest = imu  # noqa: SLF001
                loop._latest_recv_ns = now  # noqa: SLF001
                loop._latest_mag = mag  # noqa: SLF001
                loop._latest_mag_recv_ns = now  # noqa: SLF001
                loop._wheel_momentum["test/dual/wheel_x/state"] = (0.004, now)  # noqa: SLF001
                loop._wheel_momentum["test/dual/wheel_z/state"] = (0.002, now)  # noqa: SLF001
            seq += 1
            loop._step(seq, 0.0)  # noqa: SLF001
            time_module.sleep(0.05)
    finally:
        loop.close()
        for sub in subs:
            sub.undeclare()
        loop_session.close()
        observer.close()
    assert torques, "torque command never published"
    assert dipoles, "dipole command never published"
    assert torques[0].torque_x_n_m < 0.0, "damping torque must oppose the spin"
    # h=(0.004, 0, 0.002), B along y: m = k (h x B)/|B|^2 has -x and +z
    # components... h x B = (hy*bz-hz*by, hz*bx-hx*bz, hx*by-hy*bx)
    # = (-hz*By, 0, hx*By) -> m = k*(-hz, 0, hx)/By.
    assert dipoles[0].dipole_x_a_m2 < 0.0
    assert dipoles[0].dipole_z_a_m2 > 0.0


def test_partial_wheel_silence_blanks_the_momentum() -> None:
    """One silent wheel means NO momentum estimate, never a partial sum."""
    import time as time_module

    entry = _dump_entry()
    wheels: list[tuple[str, tuple[float, float, float]]] = [
        ("test/blank/wheel_x/state", (1.0, 0.0, 0.0)),
        ("test/blank/wheel_y/state", (0.0, 1.0, 0.0)),
    ]
    session = zenoh.open(zenoh.Config())
    loop = _build_dump_loop(session, entry, wheels=wheels)
    try:
        now = time_module.monotonic_ns()
        with loop._lock:  # noqa: SLF001
            loop._wheel_momentum["test/blank/wheel_x/state"] = (0.004, now)  # noqa: SLF001
            assert loop._wheel_momentum_body() is None  # noqa: SLF001
            loop._wheel_momentum["test/blank/wheel_y/state"] = (0.002, now)  # noqa: SLF001
            assert loop._wheel_momentum_body() == pytest.approx((0.004, 0.002, 0.0))  # noqa: SLF001
            # Stale is silence too: age one wheel beyond the threshold.
            old = now - int(2e9)
            loop._wheel_momentum["test/blank/wheel_y/state"] = (0.002, old)  # noqa: SLF001
            assert loop._wheel_momentum_body() is None  # noqa: SLF001
    finally:
        loop.close()
        session.close()


def test_flagged_mag_sample_is_not_fresh() -> None:
    """A validity-flagged field must reach the strategy as not-fresh."""
    import time as time_module

    from flatsat.msgs import adcs_pb2, hal_pb2

    entry = vehicle_pb2.ControlConfig(
        rate_hz=50.0,
        input_topic="test/maggate/in",
        mag_input_topic="test/maggate/mag",
        output_topic="test/maggate/out",
        stale_after_s=0.5,
        bdot=control_options_pb2.BdotOptions(gain=1.0e7, max_dipole_a_m2=25.0, filter_tau_s=0.0),
        constant_rate=control_options_pb2.ConstantRateOptions(),
        passthrough=control_options_pb2.PassthroughOptions(),
    )
    observer = zenoh.open(zenoh.Config())
    loop_session = zenoh.open(zenoh.Config())
    dipoles: list[adcs_pb2.DipoleCommand] = []
    sub = observer.declare_subscriber(
        entry.output_topic,
        lambda s: dipoles.append(adcs_pb2.DipoleCommand.FromString(bytes(s.payload.to_bytes()))),
    )
    loop = ControlLoop(
        loop_session,
        entry,
        get_controller_class("bdot").from_config(entry.bdot),
        get_guidance_class("constant_rate").from_config(entry.constant_rate),
        get_estimator_class("passthrough").from_config(entry.passthrough),
    )
    time_module.sleep(0.5)
    try:
        imu = hal_pb2.ImuSample()
        deadline = time_module.monotonic() + 5.0
        seq = 0
        # Step repeatedly with a CHANGING flagged field: a fresh-gated
        # bdot stays quiet; without the gate the swinging field would
        # command a huge dipole. Repetition also rides out discovery.
        while len(dipoles) < 2 and time_module.monotonic() < deadline:
            flagged = hal_pb2.MagnetometerSample()
            flagged.mag_x_t = 1.0e-5 if seq % 2 == 0 else 5.0e-5
            flagged.header.validity = int(hal_pb2.VALIDITY_FLAG_RANGE)
            now = time_module.monotonic_ns()
            with loop._lock:  # noqa: SLF001
                loop._latest = imu  # noqa: SLF001
                loop._latest_recv_ns = now  # noqa: SLF001
                loop._latest_mag = flagged  # noqa: SLF001
                loop._latest_mag_recv_ns = now  # noqa: SLF001
            seq += 1
            loop._step(seq, seq * 0.02)  # noqa: SLF001
            time_module.sleep(0.05)
    finally:
        loop.close()
        sub.undeclare()
        loop_session.close()
        observer.close()
    assert len(dipoles) >= 2, "commands never published"
    for cmd in dipoles:
        assert (cmd.dipole_x_a_m2, cmd.dipole_y_a_m2, cmd.dipole_z_a_m2) == (0.0, 0.0, 0.0), (
            "a flagged field must not be differentiated into a dipole"
        )


def test_sun_sample_rides_the_state_into_the_strategy() -> None:
    """A fresh sun on the bus becomes alignment torque; a stale one holds.

    White-box injection like the mag tests: this pins the STEP's sun
    plumbing (attach, fresh gate), not the subscriber machinery.
    """
    import time as time_module

    from flatsat.msgs import adcs_pb2, hal_pb2

    entry = vehicle_pb2.ControlConfig(
        rate_hz=50.0,
        input_topic="test/sungate/in",
        sun_input_topic="test/sungate/css",
        output_topic="test/sungate/torque",
        dipole_output_topic="test/sungate/dipole",
        stale_after_s=0.5,
        sun_point=control_options_pb2.SunPointOptions(
            point_axis=[0.0, 0.0, 1.0],
            k_align=0.001,
            kp=0.02,
            kd=0.0,
            max_torque_n_m=0.05,
            dump_gain=0.15,
            max_dipole_a_m2=1.0,
        ),
        constant_rate=control_options_pb2.ConstantRateOptions(),
        passthrough=control_options_pb2.PassthroughOptions(),
    )
    observer = zenoh.open(zenoh.Config())
    loop_session = zenoh.open(zenoh.Config())
    torques: list[adcs_pb2.WheelTorqueCommand] = []
    sub = observer.declare_subscriber(
        entry.output_topic,
        lambda s: torques.append(
            adcs_pb2.WheelTorqueCommand.FromString(bytes(s.payload.to_bytes()))
        ),
    )
    loop = ControlLoop(
        loop_session,
        entry,
        get_controller_class("sun_point").from_config(entry.sun_point),
        get_guidance_class("constant_rate").from_config(entry.constant_rate),
        get_estimator_class("passthrough").from_config(entry.passthrough),
    )
    time_module.sleep(0.5)  # discovery
    try:
        imu = hal_pb2.ImuSample()  # zero rates: any torque is alignment
        sun = hal_pb2.SunSensorSample()
        sun.sun_x, sun.sun_y, sun.sun_z = 1.0, 0.0, 0.0
        sun.sun_visible = True
        deadline = time_module.monotonic() + 5.0
        seq = 0
        while not torques and time_module.monotonic() < deadline:
            now = time_module.monotonic_ns()
            with loop._lock:  # noqa: SLF001 — white-box injection
                loop._latest = imu  # noqa: SLF001
                loop._latest_recv_ns = now  # noqa: SLF001
                loop._latest_sun = sun  # noqa: SLF001
                loop._latest_sun_recv_ns = now  # noqa: SLF001
            seq += 1
            loop._step(seq, 0.0)  # noqa: SLF001
            time_module.sleep(0.05)
        assert torques, "torque command never published"
        # a=+z aimed at s=+x: k_align * (a x s) = 0.001 along +y.
        assert torques[0].torque_y_n_m == pytest.approx(0.001)
        assert torques[0].torque_x_n_m == pytest.approx(0.0)
        # Now age the sun beyond the threshold: alignment must pause.
        torques.clear()
        deadline = time_module.monotonic() + 5.0
        while not torques and time_module.monotonic() < deadline:
            now = time_module.monotonic_ns()
            with loop._lock:  # noqa: SLF001
                loop._latest = imu  # noqa: SLF001
                loop._latest_recv_ns = now  # noqa: SLF001
                loop._latest_sun_recv_ns = now - int(2e9)  # noqa: SLF001
            seq += 1
            loop._step(seq, 0.0)  # noqa: SLF001
            time_module.sleep(0.05)
        assert torques, "torque command never published after staling"
        assert torques[0].torque_y_n_m == pytest.approx(0.0), (
            "a stale sun must not keep steering the vehicle"
        )
    finally:
        loop.close()
        sub.undeclare()
        loop_session.close()
        observer.close()
