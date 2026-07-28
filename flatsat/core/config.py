"""Typed, file-backed configuration with provenance — schemas are protos.

Every config file under ``config/`` is a TEXTPROTO instance of a proto
schema colocated with its owner (``flatsat/vehicle.proto``,
``flatsat/hardware/devices.proto``, ...). The proto is the single place
a field is defined: the editor resolves a ``.txtpb`` against it (its
``# proto-file:`` header), Python types flow from the generated stubs,
and C++ will generate from the same files. Parsing is STRICT — a
misspelled field is a startup failure, never a silently ignored key.

Each loaded parameter set carries provenance — the file it came from and
a checksum of the bytes — so a recorded run always traces back to the
exact configuration that produced it.

This module adds the thin behavior protos cannot: provenance, defaults
for absent optional fields, mounting normalization, and oneof→registry
resolution (the oneof field name IS the registry key). It defines no
schema of its own.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar

from google.protobuf import text_format
from google.protobuf.message import Message

from flatsat import vehicle_pb2
from flatsat.control.attitude import control_options_pb2
from flatsat.hardware import devices_pb2
from flatsat.mode import mode_config_pb2
from flatsat.telemetry import telemetry_config_pb2

CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config"

_M = TypeVar("_M", bound=Message)


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


def load_textproto(path: Path | str, message: _M) -> Provenance:
    """Parse a textproto file into a message, strictly, with provenance.

    Args:
        path: The ``.txtpb`` file.
        message: The message instance to fill (its type is the schema).

    Returns:
        The file's provenance; ``message`` is filled in place.

    Raises:
        FileNotFoundError: If the file does not exist.
        text_format.ParseError: On any unknown or malformed field — a
            typo fails at startup, never silently mid-flight.
    """
    target = Path(path)
    raw = target.read_bytes()
    checksum = hashlib.sha256(raw).hexdigest()[:12]
    text_format.Parse(raw.decode("utf-8"), message)
    return Provenance(str(target), checksum)


# --------------------------------------------------------------- vehicle --


@dataclass(frozen=True)
class Mounting:
    """Where and how a device sits in the body frame (integration truth).

    Attributes:
        position_m: Device position in the body frame.
        axis: Unit vector of the device's principal axis in the body
            frame (a wheel's spin axis). Normalized at load; a zero
            vector fails loudly.
    """

    position_m: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class SensorEntry:
    """One sensor, resolved from its config entry.

    Attributes:
        name: Instance name; becomes the message source and unit name.
        driver: Registry key — the oneof field the config filled.
        topic: Bus key expression the daemon publishes on.
        rate_hz: Publish cadence.
        options: The driver's TYPED options message.
    """

    name: str
    driver: str
    topic: str
    rate_hz: float
    options: Message


@dataclass(frozen=True)
class ActuatorEntry:
    """One actuator, resolved from its config entry.

    Attributes:
        name: Instance name; becomes the message source and unit name.
        driver: Registry key — the oneof field the config filled.
        command_topic: Bus key the daemon consumes body-frame commands from.
        state_topic: Bus key the daemon publishes device state on.
        rate_hz: Apply/publish cadence.
        stale_zero_s: Command age beyond which the daemon applies ZERO.
        mounting: Device placement (axis normalized).
        options: The driver's TYPED options message.
    """

    name: str
    driver: str
    command_topic: str
    state_topic: str
    rate_hz: float
    stale_zero_s: float
    mounting: Mounting
    options: Message


@dataclass(frozen=True)
class ControlEntry:
    """The control loop, resolved from its config entry.

    Attributes:
        strategy: Registry key — the strategy oneof field filled.
        objective: Registry key — the objective oneof field filled.
        estimator: Registry key — the estimator oneof field filled.
        rate_hz: Control cadence.
        input_topic: Topic supplying the measurement input.
        output_topic: Topic for actuator commands.
        stale_after_s: Input age beyond which the measurement is not fresh.
        options: The strategy's TYPED options message.
        objective_options: The guidance source's TYPED options message.
        estimator_options: The estimator's TYPED options message.
    """

    strategy: str
    objective: str
    estimator: str
    rate_hz: float
    input_topic: str
    output_topic: str
    stale_after_s: float
    options: Message
    objective_options: Message
    estimator_options: Message


@dataclass(frozen=True)
class BodySpec:
    """The vehicle's rigid-body physical model (integration truth).

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
class VehicleSpec:
    """What a spacecraft IS, loaded and validated.

    A different vehicle — more sensors, simulated instead of real, an ML
    controller instead of PD — is a different ``.txtpb`` of the
    VehicleConfig schema, not different code.

    Attributes:
        name: Vehicle identifier.
        description: Human-readable purpose.
        sensors: Sensor complement.
        actuators: Actuator complement.
        control: Control loop composition.
        body: Rigid-body physical model; None until the config declares
            ``body`` (consumers that need it fail loudly).
        mode: Mode-manager composition, defaults filled.
        telemetry: Recorder composition, defaults filled.
        provenance: Source file and checksum.
    """

    name: str
    description: str
    sensors: tuple[SensorEntry, ...]
    actuators: tuple[ActuatorEntry, ...]
    control: ControlEntry
    body: BodySpec | None
    mode: mode_config_pb2.ModeConfig
    telemetry: telemetry_config_pb2.TelemetryConfig
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
            The declared body section.

        Raises:
            KeyError: If the vehicle config declares no physical model.
        """
        if self.body is None:
            raise KeyError(f"vehicle {self.name!r} declares no body physical model")
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


def _parse_mounting(raw: vehicle_pb2.MountingConfig, device: str) -> Mounting:
    """Validate and normalize one device's mounting.

    Args:
        raw: The mounting message from the vehicle config.
        device: Device name, for error messages.

    Returns:
        The mounting with a normalized axis.

    Raises:
        ValueError: If dimensions are wrong or the axis is (near) zero —
            a device pointing nowhere is a config error to catch at
            startup, not a NaN to chase through the control chain.
    """
    position = tuple(raw.position_m)
    axis = tuple(raw.axis)
    if len(position) != 3 or len(axis) != 3:
        raise ValueError(f"mounting for {device!r}: position_m and axis must have 3 elements")
    norm = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
    if norm < 1e-9:
        raise ValueError(f"mounting for {device!r}: axis must not be the zero vector")
    unit = (axis[0] / norm, axis[1] / norm, axis[2] / norm)
    return Mounting(position_m=(position[0], position[1], position[2]), axis=unit)


def _which(message: Message, oneof: str, owner: str) -> str:
    """Resolve which oneof field a config entry filled.

    Args:
        message: The config message carrying the oneof.
        oneof: The oneof name (``options``, ``strategy``, ...).
        owner: Human-readable owner, for the error message.

    Returns:
        The filled field name — the registry key.

    Raises:
        ValueError: If no field of the oneof was filled.
    """
    which = message.WhichOneof(oneof)
    if which is None:
        raise ValueError(f"{owner}: no {oneof} selected — fill exactly one {oneof} block")
    return str(which)


def mode_with_defaults(cfg: mode_config_pb2.ModeConfig) -> mode_config_pb2.ModeConfig:
    """Fill an (possibly absent) mode config's defaults.

    Args:
        cfg: The declared config, possibly the empty default instance.

    Returns:
        A copy with every absent optional filled.
    """
    out = mode_config_pb2.ModeConfig()
    out.CopyFrom(cfg)
    if not out.HasField("ack_timeout_s"):
        out.ack_timeout_s = 2.0
    if not out.HasField("min_dwell_s"):
        out.min_dwell_s = 1.0
    if not out.clean_shutdown_marker:
        out.clean_shutdown_marker = "~/flatsat-state/clean-shutdown"
    return out


def telemetry_with_defaults(
    cfg: telemetry_config_pb2.TelemetryConfig,
) -> telemetry_config_pb2.TelemetryConfig:
    """Fill an (possibly absent) telemetry config's defaults.

    Args:
        cfg: The declared config, possibly the empty default instance.

    Returns:
        A copy with every absent optional filled.
    """
    out = telemetry_config_pb2.TelemetryConfig()
    out.CopyFrom(cfg)
    if not out.topics:
        out.topics.extend(["hal/**", "adcs/**", "health/**", "sys/**"])
    if not out.output_dir:
        out.output_dir = "~/flatsat-telemetry"
    if not out.HasField("max_file_bytes"):
        out.max_file_bytes = 64 * 1024 * 1024
    if not out.HasField("rotate_every_s"):
        out.rotate_every_s = 900.0
    if not out.HasField("max_total_bytes"):
        out.max_total_bytes = 4 * 1024 * 1024 * 1024
    return out


def load_vehicle(path: Path | str | None = None) -> VehicleSpec:
    """Load a vehicle composition file.

    Args:
        path: Override file; defaults to
            ``config/vehicles/flatsat_v1.txtpb``.

    Returns:
        The composition, validated, with provenance.

    Raises:
        ValueError: On an unselected oneof or invalid mounting.
        text_format.ParseError: On any unknown field (fail loud).
    """
    target = Path(path) if path else CONFIG_ROOT / "vehicles" / "flatsat_v1.txtpb"
    cfg = vehicle_pb2.VehicleConfig()
    prov = load_textproto(target, cfg)

    sensors = tuple(
        SensorEntry(
            name=s.name,
            driver=_which(s, "options", f"sensor {s.name!r}"),
            topic=s.topic,
            rate_hz=s.rate_hz,
            options=getattr(s, _which(s, "options", f"sensor {s.name!r}")),
        )
        for s in cfg.sensors
    )
    actuators = tuple(
        ActuatorEntry(
            name=a.name,
            driver=_which(a, "options", f"actuator {a.name!r}"),
            command_topic=a.command_topic,
            state_topic=a.state_topic,
            rate_hz=a.rate_hz,
            stale_zero_s=a.stale_zero_s,
            mounting=_parse_mounting(a.mounting, a.name),
            options=getattr(a, _which(a, "options", f"actuator {a.name!r}")),
        )
        for a in cfg.actuators
    )

    ctrl = cfg.control
    strategy = _which(ctrl, "strategy", "control")
    objective = ctrl.WhichOneof("objective") or "constant_rate"
    estimator = ctrl.WhichOneof("estimator") or "passthrough"
    control = ControlEntry(
        strategy=strategy,
        objective=objective,
        estimator=estimator,
        rate_hz=ctrl.rate_hz,
        input_topic=ctrl.input_topic,
        output_topic=ctrl.output_topic,
        stale_after_s=ctrl.stale_after_s,
        options=getattr(ctrl, strategy),
        objective_options=(
            getattr(ctrl, objective)
            if ctrl.WhichOneof("objective")
            else control_options_pb2.ConstantRateOptions()
        ),
        estimator_options=(
            getattr(ctrl, estimator)
            if ctrl.WhichOneof("estimator")
            else control_options_pb2.PassthroughOptions()
        ),
    )

    body: BodySpec | None = None
    if cfg.HasField("body"):
        flat = tuple(cfg.body.inertia_kg_m2)
        if len(flat) != 9:
            raise ValueError(f"{target}: body inertia_kg_m2 must have 9 values (3x3 row-major)")
        rows = (
            (flat[0], flat[1], flat[2]),
            (flat[3], flat[4], flat[5]),
            (flat[6], flat[7], flat[8]),
        )
        body = BodySpec(mass_kg=cfg.body.mass_kg, inertia_kg_m2=rows)

    return VehicleSpec(
        name=cfg.name,
        description=cfg.description,
        sensors=sensors,
        actuators=actuators,
        control=control,
        body=body,
        mode=mode_with_defaults(cfg.mode),
        telemetry=telemetry_with_defaults(cfg.telemetry),
        provenance=prov,
    )


# ---------------------------------------------------------------- devices --


def load_imu_spec(path: Path | str | None = None) -> tuple[devices_pb2.ImuDevice, Provenance]:
    """Load an IMU device specification.

    Args:
        path: Override file; defaults to ``config/devices/imu0.txtpb``.

    Returns:
        Tuple of (device spec, provenance).
    """
    spec = devices_pb2.ImuDevice()
    prov = load_textproto(Path(path) if path else CONFIG_ROOT / "devices" / "imu0.txtpb", spec)
    return spec, prov


def describe_imu_spec(spec: devices_pb2.ImuDevice, prov: Provenance) -> list[str]:
    """Render an IMU spec for logs/telemetry echo.

    Args:
        spec: The device spec.
        prov: Its provenance.

    Returns:
        Lines naming the device characteristics in effect.
    """
    return [
        f"imu spec: {prov.describe()} ({spec.name})",
        f"imu spec: noise {spec.gyro_noise_rad_s:g} rad/s, "
        f"full scale ±{spec.gyro_full_scale_rad_s:g} rad/s, "
        f"lsb {spec.gyro_lsb_rad_s:g} rad/s",
    ]


def load_wheel_spec(path: Path | str) -> tuple[devices_pb2.WheelDevice, Provenance]:
    """Load a reaction-wheel device specification.

    Args:
        path: Device file, e.g. ``config/devices/wheel0.txtpb``.

    Returns:
        Tuple of (device spec, provenance).
    """
    spec = devices_pb2.WheelDevice()
    prov = load_textproto(Path(path), spec)
    return spec, prov


def describe_wheel_spec(spec: devices_pb2.WheelDevice, prov: Provenance) -> list[str]:
    """Render a wheel spec for logs/telemetry echo.

    Args:
        spec: The device spec.
        prov: Its provenance.

    Returns:
        Lines naming the device envelopes in effect.
    """
    return [
        f"wheel spec: {prov.describe()} ({spec.name})",
        f"wheel spec: max torque {spec.max_torque_n_m:g} N·m, "
        f"max momentum {spec.max_momentum_n_m_s:g} N·m·s, "
        f"rotor inertia {spec.rotor_inertia_kg_m2:g} kg·m²",
    ]
