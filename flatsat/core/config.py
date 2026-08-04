"""Typed, file-backed configuration with provenance — schemas are protos.

Every config file under ``config/`` is a TEXTPROTO instance of a proto
schema colocated with its owner (``flatsat/vehicle.proto``,
``flatsat/hardware/devices.proto``, ...). The proto is the ONLY place a
field is defined: the editor resolves a ``.txtpb`` against it (its
``# proto-file:`` header), Python types flow from the generated stubs,
and C++ will generate from the same files. Parsing is STRICT — a
misspelled field is a startup failure, never a silently ignored key.

This module defines no parallel types. Loaders return the parsed proto
messages themselves, validated and NORMALIZED IN PLACE (mounting axes
unit-length, absent oneofs and optionals filled with their documented
defaults), wrapped only where behavior is genuinely additive:
:class:`Provenance` (file identity + checksum) and :class:`VehicleSpec`
(the parsed VehicleConfig + provenance + lookup helpers).

Implementation selection is a oneof: :func:`which_impl` resolves the
filled field name, which IS the registry key
(``flatsat/core/registry.py``) — the selector and its typed options
cannot disagree.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar

from google.protobuf import text_format
from google.protobuf.message import Message

from flatsat import vehicle_pb2
from flatsat.comms import comms_config_pb2
from flatsat.control.health import fdir_config_pb2
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


def which_impl(message: Message, oneof: str, owner: str) -> str:
    """Resolve which implementation a config entry's oneof selected.

    The filled field name is the registry key; the field value is that
    implementation's typed options message (``getattr(message, name)``).

    Args:
        message: The config message carrying the oneof.
        oneof: The oneof name (``options``, ``strategy``, ...).
        owner: Human-readable owner, for the error message.

    Returns:
        The filled field name.

    Raises:
        ValueError: If no field of the oneof was filled.
    """
    which = message.WhichOneof(oneof)
    if which is None:
        raise ValueError(f"{owner}: no {oneof} selected — fill exactly one {oneof} block")
    return str(which)


# --------------------------------------------------------------- vehicle --


@dataclass(frozen=True)
class VehicleSpec:
    """The parsed, validated VehicleConfig plus its provenance.

    Every field lives on the proto (``flatsat/vehicle.proto``) — this
    wrapper adds only lookups and logging. A different vehicle is a
    different ``.txtpb`` of that schema, not different code.

    Attributes:
        config: The parsed VehicleConfig, normalized in place (mounting
            axes unit-length, default oneofs and optionals filled).
        provenance: Source file and checksum.
    """

    config: vehicle_pb2.VehicleConfig
    provenance: Provenance

    @property
    def name(self) -> str:
        """Vehicle identifier."""
        return self.config.name

    @property
    def sensors(self) -> list[vehicle_pb2.SensorConfig]:
        """Sensor complement."""
        return list(self.config.sensors)

    @property
    def actuators(self) -> list[vehicle_pb2.ActuatorConfig]:
        """Actuator complement."""
        return list(self.config.actuators)

    @property
    def control(self) -> vehicle_pb2.ControlConfig:
        """Control loop composition."""
        return self.config.control

    @property
    def mode(self) -> mode_config_pb2.ModeConfig:
        """Mode-manager composition, defaults filled."""
        return self.config.mode

    @property
    def telemetry(self) -> telemetry_config_pb2.TelemetryConfig:
        """Recorder composition, defaults filled."""
        return self.config.telemetry

    @property
    def fdir(self) -> fdir_config_pb2.FdirConfig:
        """FDIR rulebook; empty when the vehicle declares no rules."""
        return self.config.fdir

    @property
    def comms(self) -> comms_config_pb2.CommsConfig:
        """Link composition; empty when the vehicle declares no link."""
        return self.config.comms

    def sensor(self, name: str) -> vehicle_pb2.SensorConfig:
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

    def actuator(self, name: str) -> vehicle_pb2.ActuatorConfig:
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

    def require_body(self) -> vehicle_pb2.BodyConfig:
        """Return the physical model, failing loudly when absent.

        Returns:
            The declared body section.

        Raises:
            KeyError: If the vehicle config declares no physical model.
        """
        if not self.config.HasField("body"):
            raise KeyError(f"vehicle {self.name!r} declares no body physical model")
        return self.config.body

    def describe(self) -> list[str]:
        """Render the composition for logs/telemetry echo.

        Returns:
            Lines naming the vehicle, its provenance, and its complement.
        """
        ctrl = self.config.control
        return [
            f"vehicle: {self.name} ({self.provenance.describe()})",
            f"vehicle: sensors {[s.name for s in self.config.sensors]} "
            f"actuators {[a.name for a in self.config.actuators]}",
            f"vehicle: control {which_impl(ctrl, 'strategy', 'control')} "
            f"@ {ctrl.rate_hz:g} Hz, "
            f"objective {which_impl(ctrl, 'objective', 'control')}, "
            f"estimator {which_impl(ctrl, 'estimator', 'control')}",
        ]


def _normalize_mounting(raw: vehicle_pb2.MountingConfig, device: str) -> None:
    """Validate one device's mounting and normalize its axis IN PLACE.

    Args:
        raw: The mounting message from the vehicle config.
        device: Device name, for error messages.

    Raises:
        ValueError: If dimensions are wrong or the axis is (near) zero —
            a device pointing nowhere is a config error to catch at
            startup, not a NaN to chase through the control chain.
    """
    if len(raw.position_m) != 3 or len(raw.axis) != 3:
        raise ValueError(f"mounting for {device!r}: position_m and axis must have 3 elements")
    norm = (raw.axis[0] ** 2 + raw.axis[1] ** 2 + raw.axis[2] ** 2) ** 0.5
    if norm < 1e-9:
        raise ValueError(f"mounting for {device!r}: axis must not be the zero vector")
    raw.axis[:] = [raw.axis[0] / norm, raw.axis[1] / norm, raw.axis[2] / norm]


def _fill_defaults(cfg: vehicle_pb2.VehicleConfig) -> None:
    """Fill absent optionals and default oneofs IN PLACE.

    After this, every documented default is an explicit value on the
    message — consumers never re-implement defaulting.

    Args:
        cfg: The parsed vehicle config.
    """
    if cfg.control.WhichOneof("objective") is None:
        cfg.control.constant_rate.SetInParent()  # detumble
    if cfg.control.WhichOneof("estimator") is None:
        cfg.control.passthrough.SetInParent()

    mode = cfg.mode
    if not mode.HasField("ack_timeout_s"):
        mode.ack_timeout_s = 2.0
    if not mode.HasField("min_dwell_s"):
        mode.min_dwell_s = 1.0
    if not mode.clean_shutdown_marker:
        mode.clean_shutdown_marker = "~/flatsat-state/clean-shutdown"

    telemetry = cfg.telemetry
    if not telemetry.topics:
        telemetry.topics.extend(["hal/**", "adcs/**", "health/**", "sys/**", "mission/**"])
    if not telemetry.output_dir:
        telemetry.output_dir = "~/flatsat-telemetry"
    if not telemetry.HasField("max_file_bytes"):
        telemetry.max_file_bytes = 64 * 1024 * 1024
    if not telemetry.HasField("rotate_every_s"):
        telemetry.rotate_every_s = 900.0
    if not telemetry.HasField("max_total_bytes"):
        telemetry.max_total_bytes = 4 * 1024 * 1024 * 1024


def load_vehicle(path: Path | str | None = None) -> VehicleSpec:
    """Load a vehicle composition file.

    Args:
        path: Override file; defaults to
            ``config/vehicles/flatsat_v1.txtpb``.

    Returns:
        The composition, validated and normalized, with provenance.

    Raises:
        ValueError: On an unselected oneof or invalid mounting/body.
        text_format.ParseError: On any unknown field (fail loud).
    """
    target = Path(path) if path else CONFIG_ROOT / "vehicles" / "flatsat_v1.txtpb"
    cfg = vehicle_pb2.VehicleConfig()
    prov = load_textproto(target, cfg)

    for sensor in cfg.sensors:
        which_impl(sensor, "options", f"sensor {sensor.name!r}")
    for actuator in cfg.actuators:
        which_impl(actuator, "options", f"actuator {actuator.name!r}")
        _normalize_mounting(actuator.mounting, actuator.name)
    which_impl(cfg.control, "strategy", "control")
    if cfg.HasField("body") and len(cfg.body.inertia_kg_m2) != 9:
        raise ValueError(f"{target}: body inertia_kg_m2 must have 9 values (3x3 row-major)")
    _check_command_wiring(cfg, target)
    _fill_defaults(cfg)

    return VehicleSpec(config=cfg, provenance=prov)


def _check_command_wiring(cfg: vehicle_pb2.VehicleConfig, source: Path) -> None:
    """Fail loudly when a control output feeds actuators of the wrong kind.

    Torque and dipole commands are different messages with disjoint field
    numbers, so a mis-wired topic decodes as zeros rather than lies — but
    zeros are still a silently dead actuator. The wiring is checkable at
    load time from names alone: the strategy's output kind comes from the
    registry's string table (no controller import), the actuator's from
    its driver oneof.

    Args:
        cfg: Parsed vehicle config (oneofs already validated as filled).
        source: The file, for the error message.

    Raises:
        ValueError: If a control output topic is consumed by an actuator
            that speaks the other command kind.
    """
    from flatsat.core.registry import CONTROLLER_OUTPUT_KINDS

    strategy = cfg.control.WhichOneof("strategy")
    strategy_kind = CONTROLLER_OUTPUT_KINDS.get(str(strategy), "torque")
    outputs: dict[str, str] = {}
    if cfg.control.output_topic:
        outputs[cfg.control.output_topic] = "dipole" if strategy_kind == "dipole" else "torque"
    if cfg.control.dipole_output_topic:
        outputs[cfg.control.dipole_output_topic] = "dipole"
    for actuator in cfg.actuators:
        driver = str(actuator.WhichOneof("options"))
        actuator_kind = "dipole" if driver.endswith("magnetorquer") else "torque"
        expected = outputs.get(actuator.command_topic)
        if expected is not None and expected != actuator_kind:
            raise ValueError(
                f"{source}: actuator {actuator.name!r} ({driver}) consumes "
                f"{actuator_kind} commands but its command_topic "
                f"{actuator.command_topic!r} carries {expected} commands from the "
                f"{strategy!r} strategy"
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


def load_magnetorquer_spec(
    path: Path | str,
) -> tuple[devices_pb2.MagnetorquerDevice, Provenance]:
    """Load a magnetorquer device specification.

    Args:
        path: Device file, e.g. ``config/devices/mtq0.txtpb``.

    Returns:
        Tuple of (device spec, provenance).
    """
    spec = devices_pb2.MagnetorquerDevice()
    prov = load_textproto(Path(path), spec)
    return spec, prov


def describe_magnetorquer_spec(spec: devices_pb2.MagnetorquerDevice, prov: Provenance) -> list[str]:
    """Render a magnetorquer spec for logs/telemetry echo.

    The one envelope is the dipole moment — deliberately no torque figure,
    because a magnetorquer has none (torque is m x B at wherever the
    vehicle happens to be).

    Args:
        spec: The device spec.
        prov: Its provenance.

    Returns:
        Lines naming the device envelope in effect.
    """
    return [
        f"magnetorquer spec: {prov.describe()} ({spec.name})",
        f"magnetorquer spec: max dipole {spec.max_dipole_a_m2:g} A·m²",
    ]


def load_sun_sensor_spec(
    path: Path | str | None = None,
) -> tuple[devices_pb2.SunSensorDevice, Provenance]:
    """Load a sun sensor device specification.

    Args:
        path: Override file; defaults to ``config/devices/css0.txtpb``.

    Returns:
        Tuple of (device spec, provenance).
    """
    spec = devices_pb2.SunSensorDevice()
    prov = load_textproto(Path(path) if path else CONFIG_ROOT / "devices" / "css0.txtpb", spec)
    return spec, prov


def describe_sun_sensor_spec(spec: devices_pb2.SunSensorDevice, prov: Provenance) -> list[str]:
    """Render a sun sensor spec for logs/telemetry echo.

    Args:
        spec: The device spec.
        prov: Its provenance.

    Returns:
        Lines naming the device characteristics in effect.
    """
    return [
        f"sun sensor spec: {prov.describe()} ({spec.name})",
        f"sun sensor spec: angular noise {spec.noise_rad:g} rad",
    ]


def load_magnetometer_spec(
    path: Path | str | None = None,
) -> tuple[devices_pb2.MagnetometerDevice, Provenance]:
    """Load a magnetometer device specification.

    Args:
        path: Override file; defaults to ``config/devices/mag0.txtpb``.

    Returns:
        Tuple of (device spec, provenance).
    """
    spec = devices_pb2.MagnetometerDevice()
    prov = load_textproto(Path(path) if path else CONFIG_ROOT / "devices" / "mag0.txtpb", spec)
    return spec, prov


def describe_magnetometer_spec(spec: devices_pb2.MagnetometerDevice, prov: Provenance) -> list[str]:
    """Render a magnetometer spec for logs/telemetry echo.

    Args:
        spec: The device spec.
        prov: Its provenance.

    Returns:
        Lines naming the device characteristics in effect.
    """
    return [
        f"magnetometer spec: {prov.describe()} ({spec.name})",
        f"magnetometer spec: noise {spec.noise_t:g} T, "
        f"full scale ±{spec.full_scale_t:g} T, lsb {spec.lsb_t:g} T",
    ]
