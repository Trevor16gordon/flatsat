"""Round-trip tests for the generated flatsat.v1 message bindings."""

from flatsat.msgs import hal_pb2, mode_pb2


def test_imu_sample_roundtrip() -> None:
    msg = hal_pb2.ImuSample()
    msg.header.source = "imu0"
    msg.header.sample_time_ns = 1_000_000_123
    msg.header.publish_time_ns = 1_000_000_456
    msg.header.seq = 4127
    msg.header.validity = hal_pb2.VALIDITY_FLAG_STALE | hal_pb2.VALIDITY_FLAG_RANGE
    msg.gyro_x_rad_s = 0.25
    msg.accel_z_m_s2 = -9.81
    msg.temperature_c = 21.5

    back = hal_pb2.ImuSample.FromString(msg.SerializeToString())

    assert back == msg
    assert back.header.seq == 4127
    assert back.header.validity & hal_pb2.VALIDITY_FLAG_STALE
    assert back.header.validity & hal_pb2.VALIDITY_FLAG_RANGE
    assert not back.header.validity & hal_pb2.VALIDITY_FLAG_CRC


def test_valid_header_is_zero_flags() -> None:
    header = hal_pb2.Header(source="imu0", seq=1)
    assert header.validity == hal_pb2.VALIDITY_FLAG_VALID
    assert header.validity == 0


def test_mode_state_roundtrip() -> None:
    state = mode_pb2.ModeState(
        mode=mode_pb2.SYSTEM_MODE_SAFE,
        mode_seq=3,
        reason="boot: unexpected reset flag",
        transition_time_ns=42,
    )

    back = mode_pb2.ModeState.FromString(state.SerializeToString())

    assert back.mode == mode_pb2.SYSTEM_MODE_SAFE
    assert back.mode_seq == 3
    assert back.reason == "boot: unexpected reset flag"


def test_unknown_mode_defaults_to_unspecified() -> None:
    state = mode_pb2.ModeState()
    assert state.mode == mode_pb2.SYSTEM_MODE_UNSPECIFIED
