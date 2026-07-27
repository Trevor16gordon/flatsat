"""Typed, file-backed configuration with provenance.

Every tunable in the flight software is data loaded from ``config/``, never
a literal in a function signature (PLAN §5: parameters are commandable, not
reflashed). Each loaded parameter set carries its own provenance — the file
it came from and a checksum of the bytes — so a recorded run can always be
traced back to the exact configuration that produced it.

The loaders return frozen dataclasses: mistyped or missing keys fail loudly
at startup rather than silently defaulting mid-flight.

This module is the seam a real parameter database plugs into later: swap the
file read for a bus query and nothing downstream changes.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:  # Python 3.11+ (the ground Mac)
    import tomllib
except ModuleNotFoundError:  # Python 3.10 (the onboard interpreter)
    import tomli as tomllib

CONFIG_ROOT = Path(__file__).resolve().parents[1] / "config"


@dataclass(frozen=True)
class Provenance:
    """Where a parameter set came from, and what exactly it contained.

    Attributes:
        path: Absolute path of the file the values were read from.
        checksum: First 12 hex chars of the SHA-256 of the file bytes.
    """

    path: str
    checksum: str

    def describe(self) -> str:
        """Render a one-line provenance string for logs and telemetry.

        Returns:
            Human-readable ``name@checksum`` summary.
        """
        return f"{Path(self.path).name}@{self.checksum}"


@dataclass(frozen=True)
class AdcsParams:
    """Control-loop parameters.

    Attributes:
        rate_hz: Control cadence in Hz.
        kp: Proportional gain on body rate.
        kd: Derivative gain on body rate.
        stale_after_s: Input age beyond which a cycle is flagged stale.
        sensor_topic: Topic supplying ImuSample.
        command_topic: Topic for WheelTorqueCommand output.
        provenance: Source file and checksum.
    """

    rate_hz: float
    kp: float
    kd: float
    stale_after_s: float
    sensor_topic: str
    command_topic: str
    provenance: Provenance

    def describe(self) -> list[str]:
        """Render the parameter set for logs/telemetry echo.

        Returns:
            Lines naming every value in effect and its provenance.
        """
        return [
            f"params: {self.provenance.describe()}",
            f"params: rate {self.rate_hz:g} Hz, kp {self.kp:g}, kd {self.kd:g}, "
            f"stale>{self.stale_after_s * 1000:g} ms",
            f"params: {self.sensor_topic} -> {self.command_topic}",
        ]


@dataclass(frozen=True)
class ImuSpec:
    """What an IMU device is: limits, resolution, and noise character.

    Read by BOTH the flight-side driver (to know the device) and the
    simulation's sensor model (to corrupt truth the way the device would).

    Attributes:
        name: Device instance name (becomes the message source).
        rate_hz: Native output data rate.
        gyro_noise_rad_s: White-noise sigma per gyro axis.
        gyro_full_scale_rad_s: Measurement limit; beyond it the device rails.
        gyro_lsb_rad_s: Quantization step of the gyro output.
        accel_noise_m_s2: White-noise sigma per accelerometer axis.
        accel_full_scale_m_s2: Accelerometer measurement limit.
        temperature_c: Nominal reported die temperature.
        provenance: Source file and checksum.
    """

    name: str
    rate_hz: float
    gyro_noise_rad_s: float
    gyro_full_scale_rad_s: float
    gyro_lsb_rad_s: float
    accel_noise_m_s2: float
    accel_full_scale_m_s2: float
    temperature_c: float
    provenance: Provenance

    def describe(self) -> list[str]:
        """Render the spec for logs/telemetry echo.

        Returns:
            Lines naming the device characteristics in effect.
        """
        return [
            f"imu spec: {self.provenance.describe()} ({self.name})",
            f"imu spec: noise {self.gyro_noise_rad_s:g} rad/s, "
            f"full scale ±{self.gyro_full_scale_rad_s:g} rad/s, "
            f"lsb {self.gyro_lsb_rad_s:g} rad/s",
        ]


def _load_toml(path: Path) -> tuple[dict[str, Any], Provenance]:
    """Read a TOML file and compute its provenance.

    Args:
        path: File to read.

    Returns:
        Tuple of (parsed mapping, provenance).

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    raw = path.read_bytes()
    checksum = hashlib.sha256(raw).hexdigest()[:12]
    return tomllib.loads(raw.decode("utf-8")), Provenance(str(path), checksum)


def load_adcs_params(path: Path | None = None) -> AdcsParams:
    """Load control-loop parameters.

    Args:
        path: Override file; defaults to ``config/adcs.toml``.

    Returns:
        The parameter set, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    data, prov = _load_toml(path or CONFIG_ROOT / "adcs.toml")
    return AdcsParams(
        rate_hz=float(data["rate_hz"]),
        kp=float(data["kp"]),
        kd=float(data["kd"]),
        stale_after_s=float(data["stale_after_s"]),
        sensor_topic=str(data["sensor_topic"]),
        command_topic=str(data["command_topic"]),
        provenance=prov,
    )


def load_imu_spec(path: Path | None = None) -> ImuSpec:
    """Load an IMU device specification.

    Args:
        path: Override file; defaults to ``config/imu0.toml``.

    Returns:
        The device spec, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    data, prov = _load_toml(path or CONFIG_ROOT / "imu0.toml")
    return ImuSpec(
        name=str(data["name"]),
        rate_hz=float(data["rate_hz"]),
        gyro_noise_rad_s=float(data["gyro_noise_rad_s"]),
        gyro_full_scale_rad_s=float(data["gyro_full_scale_rad_s"]),
        gyro_lsb_rad_s=float(data["gyro_lsb_rad_s"]),
        accel_noise_m_s2=float(data["accel_noise_m_s2"]),
        accel_full_scale_m_s2=float(data["accel_full_scale_m_s2"]),
        temperature_c=float(data["temperature_c"]),
        provenance=prov,
    )
