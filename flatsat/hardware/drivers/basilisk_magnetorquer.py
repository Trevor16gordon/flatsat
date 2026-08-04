"""Basilisk-fed magnetorquer: applied dipole closes the loop with the sim.

Runs under the ORDINARY actuator daemon on the flight computer. The device
physics — the dipole envelope — is the shared
:class:`~flatsat.hardware.models.magnetorquer.MagnetorquerModel`; what this
driver adds is the feedback path: every applied dipole (POST-envelope) is
published for the plant to turn into torque ``m x B`` through the vehicle
file's mounting and ITS OWN local field. The rod never claims a torque —
it has none to claim.

Quiet-state behavior (no plant running) is CORRECT behavior: the daemon's
stale-command zeroing keeps the rod quiet, the published applied-dipole
messages fall on deaf ears, state telemetry keeps flowing.
"""

from __future__ import annotations

from typing import ClassVar

import zenoh

from flatsat.core.bus import HalMessage, SamplePublisher, bus_config
from flatsat.core.config import Provenance, describe_magnetorquer_spec, load_magnetorquer_spec
from flatsat.hardware.actuator import ActuatorDriver
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.magnetorquer import MagnetorquerModel
from flatsat.msgs import hal_pb2, sim_pb2


def magnetorquer_dipole_topic(name: str) -> str:
    """Bus key one rod publishes its applied axis dipole on.

    Args:
        name: Actuator instance name.

    Returns:
        The key expression the plant subscribes to for this rod.
    """
    return f"sim/mtq/{name}/dipole"


class BasiliskMagnetorquerDriver(ActuatorDriver):
    """Rod driver whose applied dipole feeds the plant."""

    command_kind: ClassVar[str] = "dipole"

    def __init__(
        self,
        name: str,
        model: MagnetorquerModel,
        session: zenoh.Session | None = None,
        provenance: Provenance | None = None,
    ) -> None:
        """Bind the device model and the feedback publisher.

        Args:
            name: Actuator instance name (names the feedback topic).
            model: Rod envelope bounded by the device spec.
            session: Zenoh session to reuse (tests); the driver opens its
                own when omitted — the bus IS this device's wire.
            provenance: The device file's provenance, for describe().
        """
        self._name = name
        self._model = model
        self._provenance = provenance
        self._owns_session = session is None
        self._session = session if session is not None else zenoh.open(bus_config())
        self._feedback = SamplePublisher(self._session, magnetorquer_dipole_topic(name), name)

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.BasiliskMagnetorquerOptions
    ) -> BasiliskMagnetorquerDriver:
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
            raise ValueError(f"actuator {name!r}: basilisk_magnetorquer requires a device file")
        spec, prov = load_magnetorquer_spec(options.device)
        return cls(name=name, model=MagnetorquerModel(spec), provenance=prov)

    def apply(self, dipole_a_m2: float) -> int:
        """Run the device model, then feed the applied dipole to the sim.

        Args:
            dipole_a_m2: Commanded dipole along the rod axis, after the
                daemon's mounting projection of the body DipoleCommand
                (this driver declares ``command_kind = "dipole"``).

        Returns:
            Validity flags from the shared magnetorquer model (RANGE).
        """
        flags = self._model.apply(dipole_a_m2)

        feedback = sim_pb2.MagnetorquerDipole()
        feedback.rod = self._name
        feedback.dipole_a_m2 = self._model.applied_dipole_a_m2
        self._feedback.publish(feedback, validity=flags)
        return flags

    def state(self) -> tuple[HalMessage, int]:
        """Report the rod's current state.

        Returns:
            Tuple of (MagnetorquerState, validity flags).
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
            describe_magnetorquer_spec(self._model.spec, self._provenance)
            if self._provenance
            else []
        )
        return [
            f"driver: basilisk_magnetorquer feedback={magnetorquer_dipole_topic(self._name)}",
            *spec_lines,
        ]
