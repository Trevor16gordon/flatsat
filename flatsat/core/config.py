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
class Mounting:
    """Where and how a device sits in the body frame (integration truth).

    Attributes:
        position_m: Device position in the body frame.
        axis: Unit vector of the device's principal axis in the body frame
            (a wheel's spin axis). Normalized at load; a zero vector fails
            loudly.
    """

    position_m: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class ActuatorEntry:
    """One actuator in a vehicle composition.

    Attributes:
        name: Instance name; becomes the message source and unit name.
        driver: Registry key naming the driver implementation.
        command_topic: Bus key the daemon consumes body-frame commands from.
        state_topic: Bus key the daemon publishes device state on.
        rate_hz: Apply/publish cadence.
        stale_zero_s: Command age beyond which the daemon applies ZERO —
            an actuator must never keep flying a dead controller's last
            order.
        mounting: Device placement in the body frame; the daemon projects
            body-frame commands through it.
        options: Driver-specific settings passed to ``from_config``.
    """

    name: str
    driver: str
    command_topic: str
    state_topic: str
    rate_hz: float
    stale_zero_s: float
    mounting: Mounting
    options: Mapping[str, object]


@dataclass(frozen=True)
class BodySpec:
    """The vehicle's rigid-body physical model (integration truth).

    Read by the flight software for control scaling (later) and by the sim
    bridge to build its plant — the same numbers, so sim and flight cannot
    disagree about the spacecraft by accident.

    Attributes:
        mass_kg: Total vehicle mass.
        inertia_kg_m2: 3x3 inertia tensor about the body frame origin.
    """

    mass_kg: float
    inertia_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


@dataclass(frozen=True)
class ControlEntry:
    """The control loop in a vehicle composition.

    Attributes:
        strategy: Registry key naming the control implementation.
        objective: Registry key naming the guidance/reference source.
        estimator: Registry key naming the state estimator sitting between
            the sensor topic and the controller.
        rate_hz: Control cadence.
        input_topic: Topic supplying the measurement input.
        output_topic: Topic for actuator commands.
        stale_after_s: Input age beyond which the measurement is not fresh.
        options: Strategy-specific settings (gains, limits, model paths).
        objective_options: Guidance-specific settings (targets).
        estimator_options: Estimator-specific settings (filter tunings).
    """

    strategy: str
    objective: str
    estimator: str
    rate_hz: float
    input_topic: str
    output_topic: str
    stale_after_s: float
    options: Mapping[str, object]
    objective_options: Mapping[str, object]
    estimator_options: Mapping[str, object]


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
        actuators: Actuator complement.
        control: Control loop composition.
        body: Rigid-body physical model; None until a vehicle declares
            ``[body]`` (consumers that need it fail loudly).
        provenance: Source file and checksum.
    """

    name: str
    description: str
    sensors: tuple[SensorEntry, ...]
    actuators: tuple[ActuatorEntry, ...]
    control: ControlEntry
    body: BodySpec | None
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

    def actuator(self, name: str) -> ActuatorEntry:
        """Look up one actuator entry by name.

        Args:
            name: Actuator instance name.

        Returns:
            The matching entry.

        Raises:
            KeyError: If the vehicle has no such actuator.
        """
        for entry in self.actuators:
            if entry.name == name:
                return entry
        raise KeyError(f"vehicle {self.name!r} has no actuator {name!r}")

    def require_body(self) -> BodySpec:
        """Return the physical model, failing loudly when absent.

        Returns:
            The declared ``[body]`` section.

        Raises:
            KeyError: If the vehicle file declares no physical model.
        """
        if self.body is None:
            raise KeyError(f"vehicle {self.name!r} declares no [body] physical model")
        return self.body

    def describe(self) -> list[str]:
        """Render the composition for logs/telemetry echo.

        Returns:
            Lines naming the vehicle, its provenance, and its complement.
        """
        return [
            f"vehicle: {self.name} ({self.provenance.describe()})",
            f"vehicle: sensors {[s.name for s in self.sensors]} "
            f"actuators {[a.name for a in self.actuators]}",
            f"vehicle: control {self.control.strategy} @ {self.control.rate_hz:g} Hz, "
            f"objective {self.control.objective}, estimator {self.control.estimator}",
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


@dataclass(frozen=True)
class WheelSpec:
    """What a reaction wheel device is: torque and momentum envelopes.

    Device-intrinsic (datasheet) truth — true of the unit no matter which
    spacecraft it is bolted to. Calibration keys (measured alignment,
    friction) land here when a physical wheel is first characterized.

    Attributes:
        name: Device instance name (becomes the message source).
        max_torque_n_m: Torque the wheel can apply; commands clip to it.
        max_momentum_n_m_s: Momentum storage envelope; beyond it the wheel
            is saturated and torque authority in that direction is gone.
        rotor_inertia_kg_m2: Rotor inertia; speed = momentum / inertia.
        provenance: Source file and checksum.
    """

    name: str
    max_torque_n_m: float
    max_momentum_n_m_s: float
    rotor_inertia_kg_m2: float
    provenance: Provenance

    def describe(self) -> list[str]:
        """Render the spec for logs/telemetry echo.

        Returns:
            Lines naming the device envelopes in effect.
        """
        return [
            f"wheel spec: {self.provenance.describe()} ({self.name})",
            f"wheel spec: max torque {self.max_torque_n_m:g} N·m, "
            f"max momentum {self.max_momentum_n_m_s:g} N·m·s, "
            f"rotor inertia {self.rotor_inertia_kg_m2:g} kg·m²",
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


def _parse_mounting(raw: Mapping[str, Any], device: str) -> Mounting:
    """Parse and validate one device's mounting entry.

    Args:
        raw: The ``mounting`` table from the vehicle file.
        device: Device name, for error messages.

    Returns:
        The mounting with a normalized axis.

    Raises:
        ValueError: If the axis is (near) zero-length — a device pointing
            nowhere is a config error to catch at startup, not a NaN to
            chase through the control chain.
    """
    position = tuple(float(v) for v in raw["position_m"])
    axis = tuple(float(v) for v in raw["axis"])
    if len(position) != 3 or len(axis) != 3:
        raise ValueError(f"mounting for {device!r}: position_m and axis must have 3 elements")
    norm = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
    if norm < 1e-9:
        raise ValueError(f"mounting for {device!r}: axis must not be the zero vector")
    unit = (axis[0] / norm, axis[1] / norm, axis[2] / norm)
    return Mounting(position_m=(position[0], position[1], position[2]), axis=unit)


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
    actuators: list[ActuatorEntry] = []
    for row in data.get("actuators", []):
        known = {
            "name",
            "driver",
            "command_topic",
            "state_topic",
            "rate_hz",
            "stale_zero_s",
            "mounting",
        }
        actuators.append(
            ActuatorEntry(
                name=str(row["name"]),
                driver=str(row["driver"]),
                command_topic=str(row["command_topic"]),
                state_topic=str(row["state_topic"]),
                rate_hz=float(row["rate_hz"]),
                stale_zero_s=float(row["stale_zero_s"]),
                mounting=_parse_mounting(row["mounting"], str(row["name"])),
                options={k: v for k, v in row.items() if k not in known},
            )
        )
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
        "estimator",
        "rate_hz",
        "input_topic",
        "output_topic",
        "stale_after_s",
        "objective_options",
        "estimator_options",
    }
    control = ControlEntry(
        strategy=str(ctrl["strategy"]),
        objective=str(ctrl.get("objective", "constant_rate")),
        estimator=str(ctrl.get("estimator", "passthrough")),
        rate_hz=float(ctrl["rate_hz"]),
        input_topic=str(ctrl["input_topic"]),
        output_topic=str(ctrl["output_topic"]),
        stale_after_s=float(ctrl["stale_after_s"]),
        options={k: v for k, v in ctrl.items() if k not in known_ctrl},
        objective_options=dict(ctrl.get("objective_options", {})),
        estimator_options=dict(ctrl.get("estimator_options", {})),
    )
    body: BodySpec | None = None
    if "body" in data:
        raw_inertia = data["body"]["inertia_kg_m2"]
        rows = tuple(tuple(float(v) for v in line) for line in raw_inertia)
        if len(rows) != 3 or any(len(line) != 3 for line in rows):
            raise ValueError(f"{target}: [body] inertia_kg_m2 must be a 3x3 matrix")
        body = BodySpec(
            mass_kg=float(data["body"]["mass_kg"]),
            inertia_kg_m2=(rows[0], rows[1], rows[2]),  # type: ignore[arg-type]
        )

    return VehicleSpec(
        name=str(data["name"]),
        description=str(data.get("description", "")),
        sensors=tuple(sensors),
        actuators=tuple(actuators),
        control=control,
        body=body,
        provenance=prov,
    )


def load_imu_spec(path: Path | str | None = None) -> ImuSpec:
    """Load an IMU device specification.

    Args:
        path: Override file; defaults to ``config/devices/imu0.toml``.

    Returns:
        The device spec, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    data, prov = _load_toml(Path(path) if path else CONFIG_ROOT / "devices" / "imu0.toml")
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


def load_wheel_spec(path: Path | str) -> WheelSpec:
    """Load a reaction-wheel device specification.

    Args:
        path: Device file, e.g. ``config/devices/wheel0.toml``.

    Returns:
        The device spec, with provenance.

    Raises:
        KeyError: If a required key is missing (fail loud at startup).
    """
    data, prov = _load_toml(Path(path))
    return WheelSpec(
        name=str(data["name"]),
        max_torque_n_m=float(data["max_torque_n_m"]),
        max_momentum_n_m_s=float(data["max_momentum_n_m_s"]),
        rotor_inertia_kg_m2=float(data["rotor_inertia_kg_m2"]),
        provenance=prov,
    )
