"""Magnetorquer model: commanded dipole -> what the rod would drive.

PURE state machine driven by a device spec
(``flatsat/hardware/devices.proto`` MagnetorquerDevice). The ONLY envelope
is the dipole moment: a magnetorquer has no maximum torque, because torque
is ``m x B`` and the field is the orbit's business, not the device's
(docs/ARCHITECTURE.md, "A magnetorquer has no maximum torque"). The plant
computes the torque; this model only decides how much dipole the rod can
actually drive.

Deliberately absent until a physical rod exists to characterize: coil
inductance (rise time), residual/hysteresis dipole, temperature derating.
"""

from __future__ import annotations

from flatsat.hardware import devices_pb2
from flatsat.msgs import hal_pb2


class MagnetorquerModel:
    """Dipole-clipping rod bounded by its device spec."""

    def __init__(self, spec: devices_pb2.MagnetorquerDevice) -> None:
        """Bind the model to its device envelope.

        Args:
            spec: Device spec supplying the dipole limit.
        """
        self.spec = spec
        self.applied_dipole_a_m2 = 0.0

    def apply(self, dipole_a_m2: float) -> int:
        """Apply one dipole command.

        Args:
            dipole_a_m2: Commanded dipole along the rod axis.

        Returns:
            Validity flags: RANGE when the command exceeded the dipole
            envelope (clipped).
        """
        flags = int(hal_pb2.VALIDITY_FLAG_VALID)
        limit = self.spec.max_dipole_a_m2
        dipole = dipole_a_m2
        if dipole > limit:
            dipole, flags = limit, flags | hal_pb2.VALIDITY_FLAG_RANGE
        elif dipole < -limit:
            dipole, flags = -limit, flags | hal_pb2.VALIDITY_FLAG_RANGE
        self.applied_dipole_a_m2 = dipole
        return flags

    def state_message(self) -> hal_pb2.MagnetorquerState:
        """Render the current state as a MagnetorquerState message.

        Returns:
            The state message, header unstamped (the publisher fills it).
        """
        msg = hal_pb2.MagnetorquerState()
        msg.dipole_a_m2 = self.applied_dipole_a_m2
        return msg
