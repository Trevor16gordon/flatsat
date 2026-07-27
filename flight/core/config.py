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
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:  # Python 3.11+ (the ground Mac)
    import tomllib
except ModuleNotFoundError:  # Python 3.10 (the onboard interpreter)
    import tomli as tomllib

CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config"


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
class SensorEntry:
    """One sensor in a vehicle composition.

    Attributes:
        name: Instance name; becomes the message source and unit name.
        driver: Registry key naming the driver implementation.
        topic: Bus key expression the daemon publishes on.
        rate_hz: Publish cadence.
        options: Driver-specific settings passed to ``from_config``.
    """

    name: str
    driver: str
    topic: str
    rate_hz: float
    options: Mapping[str, object]


@dataclass(frozen=True)
class ControlEntry:
    """The control loop in a vehicle composition.

    Attributes:
        strategy: Registry key naming the control implementation.
        objective: Registry key naming the guidance/reference source.
        rate_hz: Control cadence.
        input_topic: Topic supplying the state estimate input.
        output_topic: Topic for actuator commands.
        stale_after_s: Input age beyond which the estimate is not valid.
        options: Strategy-specific settings (gains, limits, model paths).
        objective_options: Guidance-specific settings (targets).
    """

    strategy: str
    objective: str
    rate_hz: float
    input_topic: str
    output_topic: str
    stale_after_s: float
    options: Mapping[str, object]
    objective_options: Mapping[str, object]


@dataclass(frozen=True)
class VehicleSpec:
    """What a spacecraft IS: its sensors and how it is flown.

    A different vehicle — more sensors, simulated instead of real, an ML
    controller instead of PD — is a different file of this shape, not
    different code.

    Attributes:
        name: Vehicle identifier.
        description: Human-readable purpose.
        sensors: Sensor complement.
        control: Control loop composition.
        provenance: Source file and checksum.
    """

    name: str
    description: str
    sensors: tuple[SensorEntry, ...]
    control: ControlEntry
    provenance: Provenance

    def sensor(self, name: str) -> SensorEntry:
        """Look up one sensor entry by name.

        Args:
            name: Sensor instance name.

        Returns:
            The matching entry.

        Raises:
            KeyError: If the vehicle has no such sensor.
        """
        for entry in self.sensors:
            if entry.name == name:
                return entry
        raise KeyError(f"vehicle {self.name!r} has no sensor {name!r}")

    def describe(self) -> list[str]:
        """Render the composition for logs/telemetry echo.

        Returns:
            Lines naming the vehicle, its provenance, and its complement.
        """
        return [
            f"vehicle: {self.name} ({self.provenance.describe()})",
            f"vehicle: sensors {[s.name for s in self.sensors]}",
            f"vehicle: control {self.control.strategy} @ {self.control.rate_hz:g} Hz, "
            f"objective {self.control.objective}",
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


def load_vehicle(path: Path | str | None = None) -> VehicleSpec:
    """Load a vehicle composition file.

    Args:
        path: Override file; defaults to ``config/vehicles/flatsat_v1.toml``.

    Returns:
        The composition, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    target = Path(path) if path else CONFIG_ROOT / "vehicles" / "flatsat_v1.toml"
    data, prov = _load_toml(target)

    sensors: list[SensorEntry] = []
    for row in data.get("sensors", []):
        known = {"name", "driver", "topic", "rate_hz"}
        sensors.append(
            SensorEntry(
                name=str(row["name"]),
                driver=str(row["driver"]),
                topic=str(row["topic"]),
                rate_hz=float(row["rate_hz"]),
                options={k: v for k, v in row.items() if k not in known},
            )
        )

    ctrl = data["control"]
    known_ctrl = {
        "strategy",
        "objective",
        "rate_hz",
        "input_topic",
        "output_topic",
        "stale_after_s",
        "objective_options",
    }
    control = ControlEntry(
        strategy=str(ctrl["strategy"]),
        objective=str(ctrl.get("objective", "constant_rate")),
        rate_hz=float(ctrl["rate_hz"]),
        input_topic=str(ctrl["input_topic"]),
        output_topic=str(ctrl["output_topic"]),
        stale_after_s=float(ctrl["stale_after_s"]),
        options={k: v for k, v in ctrl.items() if k not in known_ctrl},
        objective_options=dict(ctrl.get("objective_options", {})),
    )
    return VehicleSpec(
        name=str(data["name"]),
        description=str(data.get("description", "")),
        sensors=tuple(sensors),
        control=control,
        provenance=prov,
    )


def load_imu_spec(path: Path | str | None = None) -> ImuSpec:
    """Load an IMU device specification.

    Args:
        path: Override file; defaults to ``config/imu0.toml``.

    Returns:
        The device spec, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    data, prov = _load_toml(Path(path) if path else CONFIG_ROOT / "imu0.toml")
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
