"""Name → implementation lookup for drivers, controllers, guidance, estimators.

Composition works by NAME: a vehicle file says ``driver = "basilisk_imu"`` or
``strategy = "pid"``, and this module resolves that to a class. Nothing in
the applications imports a concrete driver or controller, which is what
makes "different sensors / different control law / simulated instead of
real" a configuration change.

Imports are lazy and per-entry on purpose. A flight computer with no CUDA
must not fail to start a thermal daemon because an ML policy's import
chain is unavailable — only the strategies a vehicle actually uses get
imported. This is also the hook for remotely-installed components: a
deployed package can register additional entries at import time without
this file changing.

Import rule: ``core`` imports no domain at RUNTIME. The tables below are
strings, and the contract types appear only under ``TYPE_CHECKING`` — mypy
still checks every resolution site, but importing this module pulls in no
driver, controller, or their dependency chains.
"""

from __future__ import annotations

import importlib
from collections.abc import Mapping
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # contract types for signatures only — never imported at runtime
    from flatsat.comms.framing.framer import Framer
    from flatsat.comms.modem import Modem
    from flatsat.control.attitude.controller import AttitudeController
    from flatsat.control.attitude.estimators.estimator import StateEstimator
    from flatsat.control.attitude.guidance import ReferenceSource
    from flatsat.hardware.actuator import ActuatorDriver
    from flatsat.hardware.sensor import SensorDriver

# name -> "module:ClassName"
DRIVERS: dict[str, str] = {
    "jetson_thermal": "flatsat.hardware.drivers.jetson_thermal:JetsonThermalDriver",
    "basilisk_imu": "flatsat.hardware.drivers.basilisk_imu:BasiliskImuDriver",
    "basilisk_magnetometer": (
        "flatsat.hardware.drivers.basilisk_magnetometer:BasiliskMagnetometerDriver"
    ),
}

ACTUATORS: dict[str, str] = {
    "sim_reaction_wheel": "flatsat.hardware.drivers.sim_reaction_wheel:SimReactionWheelDriver",
    "basilisk_reaction_wheel": (
        "flatsat.hardware.drivers.basilisk_reaction_wheel:BasiliskReactionWheelDriver"
    ),
    "basilisk_magnetorquer": (
        "flatsat.hardware.drivers.basilisk_magnetorquer:BasiliskMagnetorquerDriver"
    ),
}

CONTROLLERS: dict[str, str] = {
    "rate_damping": "flatsat.control.attitude.controllers.rate_damping:RateDampingController",
    "pid": "flatsat.control.attitude.controllers.pid:PidRateController",
    "bdot": "flatsat.control.attitude.controllers.bdot:BdotController",
    "momentum_dump": ("flatsat.control.attitude.controllers.momentum_dump:MomentumDumpController"),
}

GUIDANCE: dict[str, str] = {
    "constant_rate": "flatsat.control.attitude.guidance:ConstantRateReference",
}

ESTIMATORS: dict[str, str] = {
    "passthrough": "flatsat.control.attitude.estimators.passthrough:PassthroughEstimator",
}

MODEMS: dict[str, str] = {
    "loopback": "flatsat.comms.phy.loopback:LoopbackModem",
    "pluto_gmsk": "flatsat.comms.phy.pluto_gmsk:PlutoGmskModem",
    # No DSP of its own: the seam where an external radio process —
    # gr-satellites, a GNU Radio design, a hardware TNC — drives this
    # link over KISS.
    "kiss": "flatsat.comms.phy.kiss:KissModem",
}

FRAMERS: dict[str, str] = {
    "ccsds": "flatsat.comms.framing.ccsds:CcsdsFramer",
}


def _resolve(table: Mapping[str, str], key: str, kind: str) -> type:
    """Import and return the class registered under ``key``.

    Args:
        table: Registry mapping names to ``module:Class`` targets.
        key: Registered name from a configuration file.
        kind: Human-readable category, used in the error message.

    Returns:
        The registered class.

    Raises:
        KeyError: If the name is not registered, listing what is.
    """
    if key not in table:
        raise KeyError(f"unknown {kind} {key!r}; registered: {sorted(table)}")
    module_name, _, class_name = table[key].partition(":")
    return getattr(importlib.import_module(module_name), class_name)  # type: ignore[no-any-return]


def get_driver_class(name: str) -> type[SensorDriver]:
    """Resolve a sensor driver by registry name.

    Args:
        name: Value of a vehicle file's ``driver`` key.

    Returns:
        The driver class (call ``from_config`` to instantiate).
    """
    resolved: type[SensorDriver] = _resolve(DRIVERS, name, "driver")
    return resolved


def get_actuator_class(name: str) -> type[ActuatorDriver]:
    """Resolve an actuator driver by registry name.

    Args:
        name: Value of a vehicle file's actuator ``driver`` key.

    Returns:
        The driver class (call ``from_config`` to instantiate).
    """
    resolved: type[ActuatorDriver] = _resolve(ACTUATORS, name, "actuator")
    return resolved


def get_controller_class(name: str) -> type[AttitudeController]:
    """Resolve a control strategy by registry name.

    Args:
        name: Value of a vehicle file's ``strategy`` key.

    Returns:
        The controller class (call ``from_config`` to instantiate).
    """
    resolved: type[AttitudeController] = _resolve(CONTROLLERS, name, "controller")
    return resolved


def get_guidance_class(name: str) -> type[ReferenceSource]:
    """Resolve a guidance/reference source by registry name.

    Args:
        name: Value of a vehicle file's ``objective`` key.

    Returns:
        The reference-source class (call ``from_config`` to instantiate).
    """
    resolved: type[ReferenceSource] = _resolve(GUIDANCE, name, "guidance")
    return resolved


def get_modem_class(name: str) -> type[Modem]:
    """Resolve a modem (PHY) by registry name.

    Args:
        name: The comms block's filled ``modem`` oneof field.

    Returns:
        The modem class (call ``from_config`` to instantiate).
    """
    resolved: type[Modem] = _resolve(MODEMS, name, "modem")
    return resolved


def get_framer_class(name: str) -> type[Framer]:
    """Resolve a framer by registry name.

    Args:
        name: The comms block's filled ``framer`` oneof field.

    Returns:
        The framer class (call ``from_config`` to instantiate).
    """
    resolved: type[Framer] = _resolve(FRAMERS, name, "framer")
    return resolved


def get_estimator_class(name: str) -> type[StateEstimator]:
    """Resolve a state estimator by registry name.

    Args:
        name: Value of a vehicle file's ``estimator`` key.

    Returns:
        The estimator class (call ``from_config`` to instantiate).
    """
    resolved: type[StateEstimator] = _resolve(ESTIMATORS, name, "estimator")
    return resolved
