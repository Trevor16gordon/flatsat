#!/usr/bin/env python3
"""Synthetic IMU daemon: a deterministic-ish gyro signal for loop development.

Publishes ImuSample at rate with a slow sinusoidal body rate plus Gaussian
noise — enough structure for a control law to chew on. This daemon occupies
the seam slot that a Basilisk-fed daemon takes over later (PLAN §10): same
message, same topic shape, indistinguishable downstream.

Usage:
  python -m flight.hal.fake_imu --rate 100
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time

import zenoh

from flight.hal.daemon import HalMessage, SensorConfig, SensorDaemon
from flight.msgs import hal_pb2


class FakeImuDaemon(SensorDaemon):
    """Publishes a synthetic slowly-tumbling gyro signal."""

    def __init__(
        self,
        config: SensorConfig,
        session: zenoh.Session,
        amplitude_rad_s: float = 0.05,
        period_s: float = 20.0,
        noise_rad_s: float = 0.002,
    ) -> None:
        """Configure the synthetic signal.

        Args:
            config: Instance configuration (name, topic, rate).
            session: Open zenoh session.
            amplitude_rad_s: Peak body rate of the sinusoid.
            period_s: Period of the sinusoid.
            noise_rad_s: Gaussian noise sigma per axis.
        """
        super().__init__(config, session)
        self._amplitude = amplitude_rad_s
        self._period = period_s
        self._noise = noise_rad_s
        self._t0 = time.monotonic()

    def read(self) -> tuple[HalMessage, int]:
        """Synthesize one gyro sample (never fails — it is made of math).

        Returns:
            Tuple of (ImuSample, validity flags).
        """
        t = time.monotonic() - self._t0
        phase = 2.0 * math.pi * t / self._period
        msg = hal_pb2.ImuSample()
        msg.gyro_x_rad_s = self._amplitude * math.sin(phase) + random.gauss(0.0, self._noise)
        msg.gyro_y_rad_s = self._amplitude * math.cos(phase) + random.gauss(0.0, self._noise)
        msg.gyro_z_rad_s = 0.3 * self._amplitude * math.sin(0.5 * phase) + random.gauss(
            0.0, self._noise
        )
        msg.temperature_c = 25.0
        return msg, hal_pb2.VALIDITY_FLAG_VALID


def main() -> int:
    """Run the fake IMU daemon from the command line.

    Returns:
        0 on clean exit.
    """
    parser = argparse.ArgumentParser(description="Synthetic IMU HAL daemon.")
    parser.add_argument("--rate", type=float, default=100.0, help="publish rate [Hz]")
    parser.add_argument("--name", default="imu0", help="daemon instance name")
    args = parser.parse_args()

    config = SensorConfig(name=args.name, topic=f"hal/{args.name}/sample", rate_hz=args.rate)
    session = zenoh.open(zenoh.Config())
    daemon = FakeImuDaemon(config, session)
    print(f"[{args.name}] synthetic gyro on {config.topic} at {args.rate:g} Hz")
    try:
        daemon.run()
    except KeyboardInterrupt:
        daemon.stop()
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
