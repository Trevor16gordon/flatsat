"""Basilisk-fed reaction wheel: applied torque closes the loop with the sim.

Runs under the ORDINARY actuator daemon on the flight computer. The device
physics — envelopes, momentum bookkeeping — are the shared
:class:`~flatsat.hardware.models.wheel.WheelModel`, identical to the local
fake; what this driver adds is the feedback path: every applied torque
(POST-envelope — the sim feels what the device could actually do) is
published for the bridge to fold into its plant through the vehicle
file's mounting.

Quiet-state behavior (no bridge running) is CORRECT behavior: the daemon's
stale-command zeroing keeps the wheel quiet, the published applied-torque
messages fall on deaf ears, state telemetry keeps flowing.
"""

from __future__ import annotations

import time

import zenoh

from flatsat.core.bus import HalMessage, SamplePublisher, bus_config
from flatsat.core.config import Provenance, describe_wheel_spec, load_wheel_spec
from flatsat.hardware.actuator import ActuatorDriver
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.wheel import WheelModel
from flatsat.msgs import hal_pb2, sim_pb2


def wheel_torque_topic(name: str) -> str:
    """Bus key one wheel publishes its applied axis torque on.

    Args:
        name: Actuator instance name.

    Returns:
        The key expression the bridge subscribes to for this wheel.
    """
    return f"sim/wheel/{name}/torque"


class BasiliskReactionWheelDriver(ActuatorDriver):
    """Wheel driver whose applied torque feeds the Basilisk plant."""

    def __init__(
        self,
        name: str,
        model: WheelModel,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Bind the device model and the feedback publisher.

        Args:
            name: Actuator instance name (names the feedback topic).
            model: Wheel physics bounded by the device spec.
            session: Zenoh session to reuse (tests); the driver opens its
                own when omitted — the bus IS this device's wire.
            provenance: The device file's provenance, for describe().
        """
        self._name = name
        self._model = model
        self._provenance = provenance
        self._owns_session = session is None
        self._session = session if session is not None else zenoh.open(bus_config())
        self._feedback = SamplePublisher(self._session, wheel_torque_topic(name), name)
        self._last_apply_monotonic: float | None = None

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskReactionWheelOptions
    ) -> BasiliskReactionWheelDriver:
        """Build from a vehicle-file actuator entry.

        Args:
            name: Instance name.
            options: Typed options; ``device`` is required.

        Returns:
            The configured driver.

        Raises:
            ValueError: If no device file is configured.
        """
        if not options.device:
            raise ValueError(f"actuator {name!r}: basilisk_reaction_wheel requires a device file")
        spec, prov = load_wheel_spec(options.device)
        return cls(name=name, model=WheelModel(spec), provenance=prov)

    def apply(self, torque_n_m: float) -> int:
        """Run the device model, then feed the applied torque to the sim.

        Args:
            torque_n_m: Commanded torque about the spin axis.

        Returns:
            Validity flags from the shared wheel model (RANGE, SATURATED).
        """
        now = time.monotonic()
        dt_s = 0.0 if self._last_apply_monotonic is None else now - self._last_apply_monotonic
        self._last_apply_monotonic = now
        flags = self._model.apply(torque_n_m, dt_s)

        feedback = sim_pb2.WheelAxisTorque()
        feedback.wheel = self._name
        feedback.torque_n_m = self._model.applied_torque_n_m
        self._feedback.publish(feedback, validity=flags)
        return flags

    def state(self) -> tuple[HalMessage, int]:
        """Report the wheel's current state.

        Returns:
            Tuple of (WheelState, validity flags).
        """
        return self._model.state_message(), int(hal_pb2.VALIDITY_FLAG_VALID)

    def close(self) -> None:
        """Close any owned session."""
        if self._owns_session:
            self._session.close()

    def describe(self) -> list[str]:
        """Describe the feedback path and the device spec in force.

        Returns:
            Lines naming the topic and spec provenance.
        """
        spec_lines = (
            describe_wheel_spec(self._model.spec, self._provenance) if self._provenance else []
        )
        return [
            f"driver: basilisk_reaction_wheel feedback={wheel_torque_topic(self._name)}",
            *spec_lines,
        ]
