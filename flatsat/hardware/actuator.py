"""What an actuator driver IS — the write-side twin of the sensor contract.

A driver owns access to exactly one device and answers two questions:
"apply this effort" and "what is your state." It knows nothing about the
bus, cadence, or who commands it — ``apps/actuator_daemon.py`` supplies
all of that, plus the two protections every actuator inherits:
**stale-command zeroing** (an actuator must zero when commands stop) and
the **mounting projection** (body-frame commands are projected through
THIS device's mounting geometry before they reach the driver).

Contract for implementers:
  * ``apply()`` and ``state()`` MUST NOT raise. A failure to actuate or to
    read back is reported as validity flags (flag and forward) — a raise
    would stop the cadence, which downstream cannot distinguish from the
    process dying.
  * ``apply()`` receives the command already projected onto the device's
    own axis, in device terms (a wheel: torque about its spin axis). The
    driver applies its internal calibration; it never sees the body frame.
  * ``from_config()`` builds the driver from its TYPED options message
    (its oneof block in vehicle.proto), failing loudly on missing
    required fields.

Registry: actuators are looked up by the ``driver`` key in a vehicle's
actuator entry, so adding an actuator to a spacecraft is a config edit
plus one file here — never a change to the daemon or to any consumer.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from flatsat import vehicle_pb2
from flatsat.core.bus import HalMessage

Vec3 = tuple[float, float, float]


def project_body_torque(mounting: vehicle_pb2.MountingConfig, torque_body_n_m: Vec3) -> float:
    """Project a body-frame torque command onto one device's axis.

    The controller publishes body-frame torque; each actuator daemon
    projects it through ITS OWN mounting geometry (no central mixer until
    joint allocation — saturation redistribution, null space — is actually
    needed). For a reaction wheel this is the component of the commanded
    torque along the spin axis.

    Args:
        mounting: The device's placement (the loader normalized its axis
            to unit length in place).
        torque_body_n_m: Commanded torque in the body frame.

    Returns:
        The torque the device should produce about its own axis.
    """
    ax, ay, az = (float(v) for v in mounting.axis)
    tx, ty, tz = torque_body_n_m
    return ax * tx + ay * ty + az * tz


class ActuatorDriver(ABC):
    """Owns driver-level access to one actuator device."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        name: str,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> ActuatorDriver:
        """Build a driver instance from a vehicle-file actuator entry.

        Args:
            name: Instance name (also the message source).
            options: This driver's TYPED options message (the oneof block
                the vehicle file filled — see driver_options.proto).

        Returns:
            A ready-to-command driver.
        """

    @abstractmethod
    def apply(self, torque_n_m: float) -> int:
        """Apply one effort command about the device's own axis.

        Args:
            torque_n_m: Commanded torque about the device axis, after the
                daemon's mounting projection.

        Returns:
            Validity flags describing the application (0 == applied as
            commanded). Never raises: failures come back as flags.
        """

    @abstractmethod
    def state(self) -> tuple[HalMessage, int]:
        """Read back the device's current state.

        Returns:
            Tuple of (state message with body fields filled, validity
            flags). Never raises: failures come back as flags on a
            publishable message.
        """

    def close(self) -> None:
        """Release device resources (bus subscriptions, file handles).

        Drivers whose device is reached over a bus (the basilisk_* fakes)
        override this; local devices inherit the no-op.
        """
        return

    def describe(self) -> list[str]:
        """Human-readable configuration lines for startup logs/telemetry.

        Returns:
            Lines describing what this driver is bound to.
        """
        return [f"driver: {type(self).__name__}"]
