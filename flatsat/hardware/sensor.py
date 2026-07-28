"""What a sensor driver IS, and how one gets built from configuration.

A driver owns access to exactly one device and answers one question: "give
me a reading." It knows nothing about the bus, timing, or who consumes the
data — the ``sensor_daemon`` application supplies all of that. That split is
what lets a simulated device and a real one be interchangeable: both are
just a ``read()`` producing the same message type.

Contract for implementers:
  * ``read()`` MUST NOT raise. An acquisition failure is reported as a
    validity flag on a publishable message (flag and forward, PLAN §4) —
    a raise would stop the cadence, which downstream cannot distinguish
    from the process dying, a different fault with a different response.
  * ``from_config()`` builds the driver from its TYPED options message
    (its oneof block in vehicle.proto), failing loudly on missing
    required fields.

Registry: drivers are looked up by the ``driver`` key in a vehicle's sensor
entry, so adding a device to a spacecraft is a config edit plus one file
here — never a change to the daemon or to any consumer.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from flatsat.core.bus import HalMessage


class SensorDriver(ABC):
    """Owns driver-level access to one device (PLAN §4 HAL daemon core)."""

    @classmethod
    @abstractmethod
    def from_config(
        cls,
        name: str,
        options: Any,  # noqa: ANN401 — each implementation narrows to its own options message
    ) -> SensorDriver:
        """Build a driver instance from a vehicle-file sensor entry.

        Args:
            name: Instance name (also the message source).
            options: This driver's TYPED options message (the oneof block
                the vehicle file filled — see driver_options.proto).
                ``Any`` at the contract so each implementation narrows to
                its own message type.

        Returns:
            A ready-to-read driver.
        """

    @abstractmethod
    def read(self) -> tuple[HalMessage, int]:
        """Acquire one sample.

        Returns:
            Tuple of (message with body fields filled, validity flags).
            Never raises: failures come back as flags on a valid message.
        """

    def close(self) -> None:
        """Release device resources (file handles, bus subscriptions).

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
