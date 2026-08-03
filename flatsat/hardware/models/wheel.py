"""Reaction-wheel model: commanded torque -> what the device would do.

PURE state machine driven by a device spec
(``flatsat/hardware/devices.proto`` WheelDevice), shared by every wheel fake —
``sim_reaction_wheel`` (local, open-loop) and ``basilisk_reaction_wheel``
(HIL) enforce the SAME envelopes, so swapping them cannot change where
the device saturates.

Modeled: torque clipping at the datasheet envelope (RANGE — the device
cannot have applied this), momentum integration, and the momentum rail
(SATURATED — no further torque into the rail; torque out of it is fine).
Deliberately absent until a physical wheel exists to characterize:
friction, torque ripple, zero-crossing deadband.
"""

from __future__ import annotations

from flatsat.hardware import devices_pb2
from flatsat.msgs import hal_pb2


class WheelModel:
    """Momentum-integrating wheel bounded by its device spec."""

    def __init__(self, spec: devices_pb2.WheelDevice) -> None:
        """Bind the model to its device envelopes.

        Args:
            spec: Device spec supplying torque/momentum limits and rotor
                inertia.
        """
        self.spec = spec
        self.momentum_n_m_s = 0.0
        self.applied_torque_n_m = 0.0

    def apply(self, torque_n_m: float, dt_s: float) -> int:
        """Apply one torque command over one time step.

        Args:
            torque_n_m: Commanded torque about the spin axis.
            dt_s: Time the torque acts before the next command (0 on the
                first call, when no time base exists yet).

        Returns:
            Validity flags: RANGE when the command exceeded the torque
            envelope (clipped), SATURATED when the momentum envelope
            absorbed the command (no further torque in that direction).
        """
        flags = int(hal_pb2.VALIDITY_FLAG_VALID)
        limit = self.spec.max_torque_n_m
        torque = torque_n_m
        if torque > limit:
            torque, flags = limit, flags | hal_pb2.VALIDITY_FLAG_RANGE
        elif torque < -limit:
            torque, flags = -limit, flags | hal_pb2.VALIDITY_FLAG_RANGE

        # The rotor stores the REACTION to what the wheel applies: +torque
        # to the body spins the rotor the other way. Getting this sign
        # wrong turns a momentum-dump law into positive feedback — found
        # the day the first dump law consumed this field and the wheels
        # spun UP instead of down.
        envelope = self.spec.max_momentum_n_m_s
        proposed = self.momentum_n_m_s - torque * dt_s
        if proposed >= envelope:
            proposed = envelope
            if torque < 0.0:  # pushing further INTO the rail applies nothing
                torque = 0.0
                flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        elif proposed <= -envelope:
            proposed = -envelope
            if torque > 0.0:
                torque = 0.0
                flags |= hal_pb2.VALIDITY_FLAG_SATURATED
        self.momentum_n_m_s = proposed
        self.applied_torque_n_m = torque
        return flags

    @property
    def saturated(self) -> bool:
        """Whether stored momentum sits at the envelope."""
        return abs(self.momentum_n_m_s) >= self.spec.max_momentum_n_m_s

    def state_message(self) -> hal_pb2.WheelState:
        """Render the current state as a WheelState message.

        Returns:
            The state message, header unstamped (the publisher fills it).
        """
        msg = hal_pb2.WheelState()
        msg.momentum_n_m_s = self.momentum_n_m_s
        msg.speed_rad_s = self.momentum_n_m_s / self.spec.rotor_inertia_kg_m2
        msg.torque_n_m = self.applied_torque_n_m
        msg.saturated = self.saturated
        return msg
