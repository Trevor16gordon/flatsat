"""Actuator contract tests: projection math + every registered driver."""

import pytest

from flatsat.core.config import Mounting
from flatsat.core.registry import ACTUATORS, get_actuator_class
from flatsat.hardware.actuator import ActuatorDriver, project_body_torque

WHEEL_OPTIONS = {"device": "config/devices/wheel0.toml"}


def _mounting(axis: tuple[float, float, float]) -> Mounting:
    return Mounting(position_m=(0.0, 0.0, 0.0), axis=axis)


# ---------------------------------------------------- mounting projection --


@pytest.mark.verifies("FSW-ACT-002")
def test_projection_axis_aligned_takes_that_component() -> None:
    assert project_body_torque(_mounting((1.0, 0.0, 0.0)), (0.02, -0.5, 3.0)) == pytest.approx(0.02)
    assert project_body_torque(_mounting((0.0, 0.0, 1.0)), (0.02, -0.5, 3.0)) == pytest.approx(3.0)


@pytest.mark.verifies("FSW-ACT-002")
def test_projection_reversed_axis_flips_sign() -> None:
    assert project_body_torque(_mounting((-1.0, 0.0, 0.0)), (0.02, 0.0, 0.0)) == pytest.approx(
        -0.02
    )


@pytest.mark.verifies("FSW-ACT-002")
def test_projection_oblique_axis_is_the_dot_product() -> None:
    s = 2.0**-0.5
    torque = (0.1, 0.1, 0.0)
    assert project_body_torque(_mounting((s, s, 0.0)), torque) == pytest.approx(0.2 * s)


def test_projection_orthogonal_command_is_zero() -> None:
    """A wheel cannot produce torque off its axis — projection says so."""
    assert project_body_torque(_mounting((1.0, 0.0, 0.0)), (0.0, 0.5, -0.3)) == pytest.approx(0.0)


# ------------------------------------------- contract, every actuator driver --


@pytest.mark.parametrize("name", sorted(ACTUATORS))
def test_registry_builds_every_actuator_driver(name: str) -> None:
    driver = get_actuator_class(name).from_config("test_act", WHEEL_OPTIONS)
    assert isinstance(driver, ActuatorDriver)
    assert driver.describe()


@pytest.mark.parametrize("name", sorted(ACTUATORS))
@pytest.mark.verifies("FSW-ACT-003")
def test_every_actuator_applies_and_reports_without_raising(name: str) -> None:
    driver = get_actuator_class(name).from_config("test_act", WHEEL_OPTIONS)
    flags = driver.apply(0.01)
    assert isinstance(flags, int)
    msg, state_flags = driver.state()
    assert msg.SerializeToString() is not None
    assert isinstance(state_flags, int)


@pytest.mark.parametrize("name", sorted(ACTUATORS))
@pytest.mark.verifies("FSW-ACT-003")
def test_every_actuator_survives_absurd_commands(name: str) -> None:
    """Infinite or huge commands degrade to flags, never to exceptions."""
    driver = get_actuator_class(name).from_config("test_act", WHEEL_OPTIONS)
    for command in (1e9, -1e9, 0.0):
        flags = driver.apply(command)
        assert isinstance(flags, int)
