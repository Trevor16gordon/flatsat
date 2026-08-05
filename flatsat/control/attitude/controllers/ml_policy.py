"""ml_policy: a learned control law behind the classical contract.

This strategy is the Tier-2 deployment story made concrete. The network
is an uplinked ARTIFACT — staged, sha256-verified, activated by ground
command — and this module is its consumer: at startup it reads the
active slot pointer and loads whatever version operations has made
live. No active artifact means the vehicle flies the fallback PD gains
written in the vehicle file; a learned law degrades to a classical one,
never to nothing. Adoption is restart-shaped by design (slots move
pointers, they never hot-patch a running loop — a bad activation costs
a restart, not a running system).

The network itself is deliberately modest: one hidden tanh layer
mapping body-rate error to torque, evaluated in pure Python — no numpy,
no torch, nothing on the 100 Hz path but multiplies and adds. Weights
travel as JSON: inspectable, diffable, and hashed by the uplink path
like any other artifact.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeController,
    AttitudeReference,
    AttitudeState,
    ControlLimits,
    ControlOutput,
    Vec3,
    clip_torque,
)

# Mirrors the uplink service's default staging root. Duplicated here
# because control must not import from apps; the proto comment on
# MlPolicyOptions.slots_dir names the same default as the contract.
_DEFAULT_SLOTS_DIR = "~/flatsat-uplink/slots"

_FORMAT = "flatsat-mlp-v1"


@dataclass(frozen=True)
class MlpWeights:
    """A loaded, shape-checked network.

    Attributes:
        w0: Hidden-layer weights, ``w0[j][i]`` maps input i to unit j.
        b0: Hidden-layer biases, one per unit.
        w1: Output weights, ``w1[k][j]`` maps unit j to output k.
        b1: Output biases, one per axis.
        input_scale: Rate error is divided by this before the network.
        output_scale: Network output is multiplied by this into N·m.
        trained_on: Provenance string (run id the policy learned from).
        sha256: Hash of the artifact bytes, for describe() and telemetry.
    """

    w0: tuple[tuple[float, ...], ...]
    b0: tuple[float, ...]
    w1: tuple[tuple[float, ...], ...]
    b1: tuple[float, ...]
    input_scale: float
    output_scale: float
    trained_on: str
    sha256: str


def parse_weights(raw: bytes) -> MlpWeights:
    """Parse and shape-check an ml_policy artifact.

    Args:
        raw: The artifact bytes (JSON, format ``flatsat-mlp-v1``).

    Returns:
        The validated network.

    Raises:
        ValueError: On wrong format tag, wrong shapes, or non-finite
            values — a policy that cannot be validated must fail at
            startup, not at 100 Hz.
    """
    doc = json.loads(raw)
    if doc.get("format") != _FORMAT:
        raise ValueError(f"ml_policy artifact format {doc.get('format')!r}, expected {_FORMAT!r}")
    w0 = tuple(tuple(float(x) for x in row) for row in doc["w0"])
    b0 = tuple(float(x) for x in doc["b0"])
    w1 = tuple(tuple(float(x) for x in row) for row in doc["w1"])
    b1 = tuple(float(x) for x in doc["b1"])
    hidden = len(b0)
    if len(w0) != hidden or any(len(row) != 3 for row in w0):
        raise ValueError(f"w0 must be {hidden}x3")
    if len(w1) != 3 or any(len(row) != hidden for row in w1) or len(b1) != 3:
        raise ValueError(f"w1 must be 3x{hidden} and b1 length 3")
    flat = [x for row in w0 for x in row] + list(b0) + [x for row in w1 for x in row] + list(b1)
    if not all(math.isfinite(x) for x in flat):
        raise ValueError("ml_policy weights contain non-finite values")
    input_scale = float(doc.get("input_scale", 1.0))
    output_scale = float(doc.get("output_scale", 1.0))
    if input_scale <= 0.0 or output_scale <= 0.0:
        raise ValueError("input_scale and output_scale must be positive")
    return MlpWeights(
        w0=w0,
        b0=b0,
        w1=w1,
        b1=b1,
        input_scale=input_scale,
        output_scale=output_scale,
        trained_on=str(doc.get("trained_on", "unknown")),
        sha256=hashlib.sha256(raw).hexdigest(),
    )


class MlPolicyController(AttitudeController):
    """Learned rate controller with a classical PD fallback."""

    def __init__(
        self,
        weights: MlpWeights | None,
        kp: float,
        kd: float,
        limits: ControlLimits | None = None,
        artifact: str = "",
        version: str = "",
    ) -> None:
        """Assemble the controller.

        Args:
            weights: The loaded network, or None to fly the PD fallback.
            kp: Fallback proportional gain [N·m / (rad/s)].
            kd: Fallback derivative gain [N·m / (rad/s²)].
            limits: Actuator envelope; defaults apply when omitted.
            artifact: Slot name, for describe() and telemetry.
            version: Active artifact version, for describe().
        """
        self.weights = weights
        self.kp = kp
        self.kd = kd
        self.limits = limits or ControlLimits()
        self.artifact = artifact
        self.version = version
        self._prev_error: Vec3 = (0.0, 0.0, 0.0)

    @classmethod
    def from_config(cls, options: control_options_pb2.MlPolicyOptions) -> MlPolicyController:
        """Build from vehicle-file options, consuming the active slot.

        Args:
            options: Typed options; ``artifact``, ``kp`` and ``kd`` are
                required — the fallback law must always be flyable.

        Returns:
            The configured controller, flying the active artifact when
            one exists and the PD fallback otherwise.

        Raises:
            ValueError: If a required field is absent, or the active
                artifact fails validation (fail at startup, loudly).
        """
        if not options.HasField("artifact"):
            raise ValueError("ml_policy requires an artifact slot name")
        if not options.HasField("kp") or not options.HasField("kd"):
            raise ValueError("ml_policy requires fallback kp and kd")
        limit = options.max_torque_n_m if options.HasField("max_torque_n_m") else 1.0
        slots_dir = options.slots_dir if options.HasField("slots_dir") else _DEFAULT_SLOTS_DIR

        # The slot pointer is owned by the comms domain; this strategy is
        # deliberately its consumer. Imported lazily so the controller
        # stays unit-testable (and Monte-Carlo-able) with no comms in
        # sight — weights inject straight into __init__.
        from flatsat.comms.slots import SlotManager

        manager = SlotManager(Path(slots_dir).expanduser())
        weights: MlpWeights | None = None
        version = ""
        active = manager.active_path(options.artifact)
        if active is not None:
            weights = parse_weights(active.read_bytes())
            version = manager.state(options.artifact).active_version
        return cls(
            weights=weights,
            kp=options.kp,
            kd=options.kd,
            limits=ControlLimits(max_torque_n_m=limit),
            artifact=options.artifact,
            version=version,
        )

    def update(
        self,
        state: AttitudeState,
        reference: AttitudeReference,
        dt_s: float,
    ) -> ControlOutput:
        """Compute one step of torque, learned or fallback.

        An invalid estimate commands zero torque: a network fed garbage
        emits garbage with full confidence, so unlike the PD (whose
        response to bad input is at least proportional to it), the
        learned path goes quiet and lets the loop's staleness flagging
        do its job.

        Args:
            state: Current estimated state.
            reference: Target body rates.
            dt_s: Time since the previous step in seconds.

        Returns:
            The clipped torque command.
        """
        if not state.valid:
            return ControlOutput(torque_n_m=(0.0, 0.0, 0.0))
        error = tuple(
            rate - target
            for rate, target in zip(state.body_rates_rad_s, reference.body_rates_rad_s, strict=True)
        )
        if self.weights is None:
            torque: list[float] = []
            for value, prev in zip(error, self._prev_error, strict=True):
                derivative = (value - prev) / dt_s if dt_s > 0.0 else 0.0
                torque.append(-self.kp * value - self.kd * derivative)
            self._prev_error = (error[0], error[1], error[2])
            return clip_torque((torque[0], torque[1], torque[2]), self.limits)

        net = self.weights
        x = (
            error[0] / net.input_scale,
            error[1] / net.input_scale,
            error[2] / net.input_scale,
        )
        hidden = [
            math.tanh(b + row[0] * x[0] + row[1] * x[1] + row[2] * x[2])
            for row, b in zip(net.w0, net.b0, strict=True)
        ]
        out = [
            net.output_scale * (b + sum(w * h for w, h in zip(row, hidden, strict=True)))
            for row, b in zip(net.w1, net.b1, strict=True)
        ]
        return clip_torque((out[0], out[1], out[2]), self.limits)

    def reset(self) -> None:
        """Forget the fallback PD's previous error."""
        self._prev_error = (0.0, 0.0, 0.0)

    def describe(self) -> list[str]:
        """Describe what is actually in authority.

        Returns:
            One line naming the active network (version, provenance,
            hash prefix) or stating plainly that the fallback PD flies.
        """
        if self.weights is None:
            return [
                f"controller: ml_policy artifact={self.artifact!r} NO ACTIVE VERSION — "
                f"fallback PD kp={self.kp:g} kd={self.kd:g} "
                f"limit={self.limits.max_torque_n_m:g} N·m"
            ]
        return [
            f"controller: ml_policy artifact={self.artifact!r} version={self.version} "
            f"hidden={len(self.weights.b0)} trained_on={self.weights.trained_on} "
            f"sha256={self.weights.sha256[:12]} limit={self.limits.max_torque_n_m:g} N·m"
        ]
