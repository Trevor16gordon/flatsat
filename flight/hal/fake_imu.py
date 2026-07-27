#!/usr/bin/env python3
"""Synthetic IMU daemon: analytic tumble truth through the real sensor model.

Generates a slowly-tumbling body-rate TRUTH signal analytically, then passes
it through :func:`flight.hal.sensor_model.apply_gyro_model` using the same
``config/imu0.toml`` device spec the Basilisk feed and (eventually) the real
driver use. Noise, saturation, and quantization therefore match everywhere —
there is no second noise constant to drift.

This daemon occupies the seam slot that a Basilisk-fed feed takes over
(PLAN §10): same message, same topic, indistinguishable downstream.

Usage:
  python -m flight.hal.fake_imu --rate 100
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import zenoh

from flight.config import ImuSpec, load_imu_spec
from flight.hal.daemon import SensorConfig, SensorDaemon
from flight.hal.publisher import HalMessage
from flight.hal.sensor_model import apply_gyro_model
from flight.msgs import hal_pb2


class FakeImuDaemon(SensorDaemon):
    """Publishes a synthetic tumble through the configured sensor model."""

    def __init__(
        self,
        config: SensorConfig,
        session: zenoh.Session,
        spec: ImuSpec,
        amplitude_rad_s: float = 0.05,
        period_s: float = 20.0,
    ) -> None:
        """Configure the synthetic truth signal.

        Args:
            config: Instance configuration (name, topic, rate).
            session: Open zenoh session.
            spec: Device spec supplying noise, range, and resolution.
            amplitude_rad_s: Peak true body rate of the sinusoid.
            period_s: Period of the sinusoid.
        """
        super().__init__(config, session)
        self._spec = spec
        self._amplitude = amplitude_rad_s
        self._period = period_s
        self._t0 = time.monotonic()

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
        (gx, gy, gz), flags = apply_gyro_model(truth, self._spec)

        msg = hal_pb2.ImuSample()
        msg.gyro_x_rad_s = gx
        msg.gyro_y_rad_s = gy
        msg.gyro_z_rad_s = gz
        msg.temperature_c = self._spec.temperature_c
        return msg, flags


def main() -> int:
    """Run the fake IMU daemon from the command line.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Synthetic IMU HAL daemon.")
    parser.add_argument(
        "--rate", type=float, default=None, help="publish rate [Hz]; default: spec rate"
    )
    parser.add_argument("--name", default=None, help="instance name; default: spec name")
    args = parser.parse_args()

    spec = load_imu_spec()
    name = args.name or spec.name
    rate = args.rate or spec.rate_hz
    config = SensorConfig(name=name, topic=f"hal/{name}/sample", rate_hz=rate)

    session = zenoh.open(zenoh.Config())
    daemon = FakeImuDaemon(config, session, spec)
    for line in spec.describe():
        print(f"[{name}] {line}", flush=True)
    print(f"[{name}] synthetic gyro on {config.topic} at {rate:g} Hz", flush=True)
    try:
        daemon.run()
    except KeyboardInterrupt:
        daemon.stop()
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
