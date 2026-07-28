"""Simulated reaction wheel: a local, open-loop device fake.

Interchangeable with a hardware wheel driver by construction — same
``apply``/``state`` contract, same message, same validity semantics. The
device physics live in the shared :class:`~flatsat.hardware.models.wheel.
WheelModel`, driven by the spec in ``config/devices/*.toml``, so this
fake and the Basilisk-fed one saturate exactly where the wheel that
exists would.

Local-only alternative to ``basilisk_reaction_wheel``: no external
dependencies, applied torque goes nowhere (no physics feedback) — the
right choice when no ground machine is running and closed-loop dynamics
are not the point.
"""

from __future__ import annotations

import time

from flatsat.core.bus import HalMessage
from flatsat.core.config import Provenance, describe_wheel_spec, load_wheel_spec
from flatsat.hardware.actuator import ActuatorDriver
from flatsat.hardware.drivers import driver_options_pb2
from flatsat.hardware.models.wheel import WheelModel
from flatsat.msgs import hal_pb2


class SimReactionWheelDriver(ActuatorDriver):
    """Wheel fake integrating the shared model against the wall clock."""

    def __init__(self, model: WheelModel, provenance: Provenance | None = None) -> None:
        """Bind the fake to its device model.

        Args:
            model: Wheel physics bounded by the device spec.
            provenance: The device file's provenance, for describe().
        """
        self._model = model
        self._provenance = provenance
        self._last_apply_monotonic: float | None = None

    @classmethod
    def from_config(
        cls, name: str, options: driver_options_pb2.SimReactionWheelOptions
    ) -> SimReactionWheelDriver:
        """Build from a vehicle-file actuator entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Typed options; ``device`` is required.

        Returns:
            The configured simulated wheel.

        Raises:
            ValueError: If no device file is configured.
        """
        if not options.device:
            raise ValueError(f"actuator {name!r}: sim_reaction_wheel requires a device file")
        spec, prov = load_wheel_spec(options.device)
        return cls(model=WheelModel(spec), provenance=prov)

    def apply(self, torque_n_m: float) -> int:
        """Integrate the commanded torque into stored momentum.

        Args:
            torque_n_m: Commanded torque about the spin axis.

        Returns:
            Validity flags from the shared wheel model (RANGE, SATURATED).
        """
        now = time.monotonic()
        dt_s = 0.0 if self._last_apply_monotonic is None else now - self._last_apply_monotonic
        self._last_apply_monotonic = now
        return self._model.apply(torque_n_m, dt_s)

    def state(self) -> tuple[HalMessage, int]:
        """Report the wheel's current state.

        Returns:
            Tuple of (WheelState, validity flags) — always VALID for the
            fake; a hardware driver flags COMM on a failed readback.
        """
        return self._model.state_message(), int(hal_pb2.VALIDITY_FLAG_VALID)

    def describe(self) -> list[str]:
        """Describe the simulated device and its spec provenance.

        Returns:
            Lines naming the fake and the device spec in force.
        """
        spec_lines = (
            describe_wheel_spec(self._model.spec, self._provenance) if self._provenance else []
        )
        return ["driver: sim_reaction_wheel (local fake, no physics feedback)", *spec_lines]
