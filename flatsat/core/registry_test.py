"""Registry: every registered name must resolve to a buildable class.

These tests parameterize over the registry tables themselves, so a new
driver, strategy, guidance source, or estimator is held to them the moment
it is registered — no test edit required.
"""

import pytest

from flatsat.core.registry import (
    ACTUATORS,
    CONTROLLERS,
    DRIVERS,
    ESTIMATORS,
    GUIDANCE,
    get_actuator_class,
    get_controller_class,
    get_driver_class,
    get_estimator_class,
    get_guidance_class,
)


@pytest.mark.parametrize("name", sorted(DRIVERS))
def test_every_registered_driver_resolves(name: str) -> None:
    assert callable(get_driver_class(name).from_config)


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
def test_every_registered_controller_resolves(name: str) -> None:
    assert callable(get_controller_class(name).from_config)


@pytest.mark.parametrize("name", sorted(GUIDANCE))
def test_every_registered_guidance_resolves(name: str) -> None:
    assert callable(get_guidance_class(name).from_config)


@pytest.mark.parametrize("name", sorted(ESTIMATORS))
def test_every_registered_estimator_resolves(name: str) -> None:
    assert callable(get_estimator_class(name).from_config)


def test_unknown_name_lists_what_is_registered() -> None:
    with pytest.raises(KeyError, match="registered"):
        get_driver_class("no_such_driver")


def test_registry_module_imports_no_domain() -> None:
    """Core imports no domain at runtime — the lazy-import property itself.

    A fresh interpreter importing the registry must not pull in driver or
    controller modules; a board without CUDA must be able to consult the
    registry without importing an ML policy's dependency chain.
    """
    import subprocess
    import sys

    probe = (
        "import sys; import flatsat.core.registry; "
        "bad = [m for m in sys.modules if m.startswith(('flatsat.hardware', "
        "'flatsat.control'))]; "
        "sys.exit(1 if bad else 0)"
    )
    result = subprocess.run([sys.executable, "-c", probe], check=False)
    assert result.returncode == 0, "importing the registry imported a domain module"


@pytest.mark.parametrize("name", sorted(ACTUATORS))
def test_every_registered_actuator_resolves(name: str) -> None:
    assert callable(get_actuator_class(name).from_config)
