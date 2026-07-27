"""Simulated IMU driver: analytic tumble truth through the device model.

Interchangeable with a real IMU driver by construction — same ``read()``
contract, same message, same validity semantics. Truth is generated
analytically here; the device's noise, saturation, and quantization come
from the shared spec in ``config/imu0.toml`` via
:func:`flight.hal.models.imu.apply_gyro_model`, so this driver and a
Basilisk-fed feed corrupt truth identically.

This is the seam slot a hardware IMU driver takes over: swap the vehicle
file's ``driver = "sim_gyro"`` for the real one and nothing else changes.
"""

from __future__ import annotations

import math
import random
import time
from collections.abc import Mapping

from flight.core.bus import HalMessage
from flight.core.config import ImuSpec, load_imu_spec
from flight.hal.driver import SensorDriver
from flight.hal.models.imu import apply_gyro_model
from flight.msgs import hal_pb2


class SimGyroDriver(SensorDriver):
    """Analytic tumbling body-rate source passed through the sensor model."""

    def __init__(
        self,
        spec: ImuSpec,
        amplitude_rad_s: float = 0.05,
        period_s: float = 20.0,
        seed: int | None = None,
    ) -> None:
        """Configure the synthetic truth signal.

        Args:
            spec: Device spec supplying noise, range, and resolution.
            amplitude_rad_s: Peak true body rate of the sinusoid.
            period_s: Period of the sinusoid.
            seed: RNG seed; None uses the shared generator.
        """
        self._spec = spec
        self._amplitude = amplitude_rad_s
        self._period = period_s
        self._rng = random.Random(seed) if seed is not None else None
        self._t0 = time.monotonic()

    @classmethod
    def from_config(cls, name: str, options: Mapping[str, object]) -> SimGyroDriver:
        """Build from a vehicle-file sensor entry.

        Args:
            name: Instance name (unused; the spec names the device).
            options: Optional ``spec`` path, ``amplitude_rad_s``,
                ``period_s``, ``seed``.

        Returns:
            The configured simulated driver.
        """
        spec_path = options.get("spec")
        spec = load_imu_spec(str(spec_path)) if spec_path else load_imu_spec()
        return cls(
            spec=spec,
            amplitude_rad_s=float(options.get("amplitude_rad_s", 0.05)),  # type: ignore[arg-type]
            period_s=float(options.get("period_s", 20.0)),  # type: ignore[arg-type]
            seed=int(options["seed"]) if "seed" in options else None,  # type: ignore[call-overload]
        )

    def read(self) -> tuple[HalMessage, int]:
        """Synthesize truth, then apply the device model.

        Returns:
            Tuple of (ImuSample, validity flags from the sensor model).
        """
        t = time.monotonic() - self._t0
        phase = 2.0 * math.pi * t / self._period
        truth = (
            self._amplitude * math.sin(phase),
            self._amplitude * math.cos(phase),
            0.3 * self._amplitude * math.sin(0.5 * phase),
        )
        (gx, gy, gz), flags = apply_gyro_model(truth, self._spec, self._rng)

        msg = hal_pb2.ImuSample()
        msg.gyro_x_rad_s = gx
        msg.gyro_y_rad_s = gy
        msg.gyro_z_rad_s = gz
        msg.temperature_c = self._spec.temperature_c
        return msg, flags

    def describe(self) -> list[str]:
        """Describe the simulated device and its spec provenance.

        Returns:
            Lines naming the signal and the device spec in force.
        """
        return [
            f"driver: sim_gyro amplitude={self._amplitude:g} rad/s period={self._period:g} s",
            *self._spec.describe(),
        ]
