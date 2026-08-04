"""Controller contract tests — run against EVERY registered strategy.

This is what keeps :class:`AttitudeController` an abstraction rather than
one implementation wearing a base class: a new strategy inherits these
checks automatically the moment it is registered. Strategy-specific
numerical tests live beside each law's module.
"""

import math

import pytest
from google.protobuf.message import Message

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
    clip_torque,
)
from flatsat.core.registry import CONTROLLERS, get_controller_class

DT = 0.01
DETUMBLE = AttitudeReference()


# One typed options instance per registered strategy — a newly registered
# controller must add its entry here (the KeyError below is the reminder).
def _options(max_torque_n_m: float = 1e6) -> dict[str, Message]:
    return {
        "rate_damping": control_options_pb2.RateDampingOptions(
            kp=0.02, kd=0.005, max_torque_n_m=max_torque_n_m
        ),
        "pid": control_options_pb2.PidOptions(
            kp=0.02, ki=0.001, kd=0.005, max_torque_n_m=max_torque_n_m
        ),
        # The generic limit doubles as the dipole clip for the magnetic law.
        "bdot": control_options_pb2.BdotOptions(
            gain=1.0e7, max_dipole_a_m2=max_torque_n_m, filter_tau_s=0.0
        ),
        "momentum_dump": control_options_pb2.MomentumDumpOptions(
            kp=0.02, kd=0.005, max_torque_n_m=max_torque_n_m, dump_gain=0.15, max_dipole_a_m2=1.0
        ),
        "nadir_point": control_options_pb2.NadirPointOptions(
            orbit="config/orbits/starlink_leo.txtpb",
            point_axis=[0.0, 0.0, 1.0],
            k_align=0.001,
            kp=0.02,
            kd=0.005,
            max_torque_n_m=max_torque_n_m,
            dump_gain=0.15,
            max_dipole_a_m2=1.0,
        ),
        "sun_point": control_options_pb2.SunPointOptions(
            point_axis=[0.0, 0.0, 1.0],
            k_align=0.001,
            kp=0.02,
            kd=0.005,
            max_torque_n_m=max_torque_n_m,
            dump_gain=0.15,
            max_dipole_a_m2=1.0,
        ),
    }


UNIVERSAL_OPTIONS = _options()


def state(
    rates: tuple[float, float, float],
    valid: bool = True,
    field: tuple[float, float, float] | None = None,
) -> AttitudeState:
    return AttitudeState(
        body_rates_rad_s=rates, valid=valid, mag_field_t=field, mag_fresh=field is not None
    )


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
def test_registry_builds_every_strategy(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS[name])
    assert isinstance(controller, AttitudeController)
    assert controller.describe()


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-001")
def test_every_strategy_is_quiet_at_the_reference(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS[name])
    output = controller.update(state((0.0, 0.0, 0.0)), DETUMBLE, DT)
    assert output.torque_n_m == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-002", "FSW-ADCS-003")
def test_every_strategy_opposes_error_and_respects_limits(name: str) -> None:
    controller = get_controller_class(name).from_config(_options(max_torque_n_m=1e-4)[name])
    if type(controller).output_kind == "dipole":
        # A magnetic law's "error" is the field's rotation: feed a rising
        # Bx and expect the opposing dipole, clipped at the tiny limit.
        controller.update(state((0.0, 0.0, 0.0), field=(1.0e-5, 0.0, 0.0)), DETUMBLE, DT)
        output = controller.update(state((0.0, 0.0, 0.0), field=(2.0e-5, 0.0, 0.0)), DETUMBLE, DT)
        assert output.dipole_a_m2[0] == pytest.approx(-1e-4)
        assert output.saturated
        return
    output = controller.update(state((5.0, 0.0, 0.0)), DETUMBLE, DT)
    assert output.torque_n_m[0] == pytest.approx(-1e-4)
    assert output.saturated


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-004")
def test_every_strategy_survives_bad_timestep(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS[name])
    output = controller.update(state((0.1, 0.0, 0.0)), DETUMBLE, 0.0)
    assert all(math.isfinite(v) for v in output.torque_n_m)


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
@pytest.mark.verifies("FSW-ADCS-006")
def test_reset_clears_history(name: str) -> None:
    controller = get_controller_class(name).from_config(UNIVERSAL_OPTIONS[name])
    for _ in range(100):
        controller.update(state((0.3, 0.0, 0.0)), DETUMBLE, DT)
    controller.reset()
    fresh = get_controller_class(name).from_config(UNIVERSAL_OPTIONS[name])
    assert controller.update(state((0.1, 0.0, 0.0)), DETUMBLE, DT).torque_n_m == pytest.approx(
        fresh.update(state((0.1, 0.0, 0.0)), DETUMBLE, DT).torque_n_m
    )


def test_clip_torque_reports_saturation() -> None:
    limits = ControlLimits(max_torque_n_m=0.1)
    clipped = clip_torque((0.5, -0.05, 0.0), limits)
    assert clipped.torque_n_m == pytest.approx((0.1, -0.05, 0.0))
    assert clipped.saturated
    quiet = clip_torque((0.05, 0.0, 0.0), limits)
    assert not quiet.saturated
