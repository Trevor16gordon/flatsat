"""Simulated reaction wheel: a device fake with real device envelopes.

Interchangeable with a hardware wheel driver by construction — same
``apply``/``state`` contract, same message, same validity semantics. The
device's torque and momentum envelopes come from its spec in
``config/devices/*.toml``, so this fake saturates exactly where the wheel
that exists would.

Physics kept: torque integrates into stored momentum against the wall
clock, momentum clamps at the envelope (SATURATED — the honest signal
that torque authority in that direction is gone), commands beyond the
torque envelope clip (RANGE — the device cannot have applied this).
Physics deliberately absent until a physical part exists to characterize:
friction, torque ripple, zero-crossing deadband.
"""

from __future__ import annotations

import time
from collections.abc import Mapping

from flatsat.core.bus import HalMessage
from flatsat.core.config import WheelSpec, load_wheel_spec
from flatsat.hardware.actuator import ActuatorDriver
from flatsat.msgs import hal_pb2


class SimReactionWheelDriver(ActuatorDriver):
    """Momentum-integrating wheel fake bounded by its device spec."""

    def __init__(self, spec: WheelSpec) -> None:
        """Bind the fake to its device envelopes.

        Args:
            spec: Device spec supplying torque/momentum limits and rotor
                inertia.
        """
        self._spec = spec
        self._momentum_n_m_s = 0.0
        self._applied_torque_n_m = 0.0
        self._last_apply_monotonic: float | None = None

    @classmethod
    def from_config(cls, name: str, options: Mapping[str, object]) -> SimReactionWheelDriver:
        """Build from a vehicle-file actuator entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Must contain ``device`` — the device spec path.

        Returns:
            The configured simulated wheel.
        """
        return cls(spec=load_wheel_spec(str(options["device"])))

    def apply(self, torque_n_m: float) -> int:
        """Integrate the commanded torque into stored momentum.

        Args:
            torque_n_m: Commanded torque about the spin axis.

        Returns:
            Validity flags: RANGE when the command exceeded the torque
            envelope (clipped), SATURATED when the momentum envelope
            absorbed the command (no further torque in that direction).
        """
        flags = int(hal_pb2.VALIDITY_FLAG_VALID)
        limit = self._spec.max_torque_n_m
        torque = torque_n_m
        if torque > limit:
            torque, flags = limit, flags | hal_pb2.VALIDITY_FLAG_RANGE
        elif torque < -limit:
            torque, flags = -limit, flags | hal_pb2.VALIDITY_FLAG_RANGE

        now = time.monotonic()
        dt_s = 0.0 if self._last_apply_monotonic is None else now - self._last_apply_monotonic
        self._last_apply_monotonic = now

        envelope = self._spec.max_momentum_n_m_s
        proposed = self._momentum_n_m_s + torque * dt_s
        if proposed >= envelope:
            proposed = envelope
            if torque > 0.0:  # pushing further INTO the rail applies nothing
                torque = 0.0
                flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        elif proposed <= -envelope:
            proposed = -envelope
            if torque < 0.0:
                torque = 0.0
                flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        self._momentum_n_m_s = proposed
        self._applied_torque_n_m = torque
        return flags

    def state(self) -> tuple[HalMessage, int]:
        """Report the wheel's current state.

        Returns:
            Tuple of (WheelState, validity flags) — always VALID for the
            fake; a hardware driver flags COMM on a failed readback.
        """
        msg = hal_pb2.WheelState()
        msg.momentum_n_m_s = self._momentum_n_m_s
        msg.speed_rad_s = self._momentum_n_m_s / self._spec.rotor_inertia_kg_m2
        msg.torque_n_m = self._applied_torque_n_m
        msg.saturated = abs(self._momentum_n_m_s) >= self._spec.max_momentum_n_m_s
        return msg, int(hal_pb2.VALIDITY_FLAG_VALID)

    def describe(self) -> list[str]:
        """Describe the simulated device and its spec provenance.

        Returns:
            Lines naming the fake and the device spec in force.
        """
        return ["driver: sim_reaction_wheel", *self._spec.describe()]
