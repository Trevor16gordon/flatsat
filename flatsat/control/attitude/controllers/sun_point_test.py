"""Sun-pointing law: alignment torque, eclipse pause, dump parity."""

import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import AttitudeReference, AttitudeState
from flatsat.control.attitude.controllers.sun_point import SunPointController

DT = 0.01
DETUMBLE = AttitudeReference()


def _controller(k_align: float = 0.001, dump_gain: float = 0.15) -> SunPointController:
    return SunPointController(
        point_axis=(0.0, 0.0, 1.0),
        k_align=k_align,
        kp=0.02,
        kd=0.0,
        max_torque_n_m=0.05,
        dump_gain=dump_gain,
        max_dipole_a_m2=1.0,
    )


def _state(
    rates: tuple[float, float, float] = (0.0, 0.0, 0.0),
    sun: tuple[float, float, float] | None = None,
    sun_visible: bool = False,
    field: tuple[float, float, float] | None = None,
    momentum: tuple[float, float, float] | None = None,
) -> AttitudeState:
    return AttitudeState(
        body_rates_rad_s=rates,
        valid=True,
        mag_field_t=field,
        mag_fresh=field is not None,
        wheel_momentum_n_m_s=momentum,
        sun_body=sun,
        sun_visible=sun_visible,
    )


@pytest.mark.verifies("FSW-ADCS-014")
def test_alignment_torque_is_k_align_axis_cross_sun() -> None:
    output = _controller().update(_state(sun=(1.0, 0.0, 0.0), sun_visible=True), DETUMBLE, DT)
    # a x s with a=+z, s=+x is +y, scaled by the gain.
    assert output.torque_n_m == pytest.approx((0.0, 0.001, 0.0))


def test_no_torque_when_already_pointing() -> None:
    output = _controller().update(_state(sun=(0.0, 0.0, 1.0), sun_visible=True), DETUMBLE, DT)
    assert output.torque_n_m == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.verifies("FSW-ADCS-014")
def test_eclipse_pauses_alignment_and_keeps_damping() -> None:
    """Chasing a zero vector would torque toward garbage; hold instead."""
    dark = _state(rates=(0.5, 0.0, 0.0), sun=(0.0, 0.0, 0.0), sun_visible=False)
    output = _controller().update(dark, DETUMBLE, DT)
    assert output.torque_n_m[0] == pytest.approx(-0.02 * 0.5)  # pure rate damping
    assert output.torque_n_m[1] == pytest.approx(0.0)
    assert output.torque_n_m[2] == pytest.approx(0.0)


def test_dump_dipole_matches_the_momentum_dump_law() -> None:
    """H along +x, B along +z: m = gain * (h x B)/|B|^2 points along -y."""
    field = (0.0, 0.0, 2.0e-5)
    momentum = (0.01, 0.0, 0.0)
    output = _controller().update(_state(field=field, momentum=momentum), DETUMBLE, DT)
    expected = -0.15 * 0.01 / 2.0e-5
    clipped = max(-1.0, expected)
    assert output.dipole_a_m2 == pytest.approx((0.0, clipped, 0.0))
    assert output.saturated  # the unclipped value is far past 1 A·m²


def test_stale_field_or_missing_momentum_means_zero_dipole() -> None:
    no_field = _controller().update(_state(momentum=(0.01, 0.0, 0.0)), DETUMBLE, DT)
    assert no_field.dipole_a_m2 == (0.0, 0.0, 0.0)
    no_momentum = _controller().update(_state(field=(0.0, 0.0, 2.0e-5)), DETUMBLE, DT)
    assert no_momentum.dipole_a_m2 == (0.0, 0.0, 0.0)


def test_from_config_requires_axis_and_gains() -> None:
    with pytest.raises(ValueError, match="point_axis"):
        SunPointController.from_config(
            control_options_pb2.SunPointOptions(k_align=0.001, kp=0.02, kd=0.005, dump_gain=0.1)
        )
    with pytest.raises(ValueError, match="requires"):
        SunPointController.from_config(
            control_options_pb2.SunPointOptions(point_axis=[0.0, 0.0, 1.0], kp=0.02)
        )


def test_point_axis_is_normalized() -> None:
    controller = SunPointController(
        point_axis=(0.0, 0.0, 10.0),
        k_align=0.001,
        kp=0.02,
        kd=0.005,
        max_torque_n_m=0.05,
        dump_gain=0.1,
        max_dipole_a_m2=1.0,
    )
    assert controller.point_axis == pytest.approx((0.0, 0.0, 1.0))
