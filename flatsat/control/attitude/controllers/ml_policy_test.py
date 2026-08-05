"""Unit tests for the ml_policy controller."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from flatsat.control.attitude import control_options_pb2
from flatsat.control.attitude.controller import (
    AttitudeReference,
    AttitudeState,
    ControlLimits,
)
from flatsat.control.attitude.controllers.ml_policy import (
    MlPolicyController,
    parse_weights,
)
from flatsat.control.attitude.controllers.rate_damping import RateDampingController


def _identity_artifact(hidden: int = 4, gain: float = -0.02) -> bytes:
    """Build a tiny artifact approximating torque = gain * error.

    With input_scale=1 and small inputs, tanh(w*x) ~ w*x, so one unit
    per axis with small weights gives a near-linear map the tests can
    predict analytically.

    Args:
        hidden: Hidden units (first 3 carry the signal, rest are dead).
        gain: End-to-end torque per unit rate error.

    Returns:
        Artifact bytes.
    """
    eps = 0.01  # keep tanh in its linear region
    w0 = [[0.0, 0.0, 0.0] for _ in range(hidden)]
    for axis in range(3):
        w0[axis][axis] = eps
    w1 = [[0.0] * hidden for _ in range(3)]
    for axis in range(3):
        w1[axis][axis] = gain / eps
    return json.dumps(
        {
            "format": "flatsat-mlp-v1",
            "w0": w0,
            "b0": [0.0] * hidden,
            "w1": w1,
            "b1": [0.0, 0.0, 0.0],
            "input_scale": 1.0,
            "output_scale": 1.0,
            "trained_on": "unit-test",
        }
    ).encode()


class TestParseWeights:
    """The artifact gate: bad networks fail at startup, loudly."""

    def test_round_trip(self) -> None:
        """A valid artifact parses with provenance and hash."""
        net = parse_weights(_identity_artifact())
        assert len(net.b0) == 4
        assert net.trained_on == "unit-test"
        assert len(net.sha256) == 64

    def test_wrong_format_rejected(self) -> None:
        """An unknown format tag is refused."""
        doc = json.loads(_identity_artifact())
        doc["format"] = "not-a-network"
        with pytest.raises(ValueError, match="format"):
            parse_weights(json.dumps(doc).encode())

    def test_wrong_shape_rejected(self) -> None:
        """A shape mismatch is refused."""
        doc = json.loads(_identity_artifact())
        doc["w0"] = [[1.0, 2.0]]  # not Nx3
        with pytest.raises(ValueError, match="w0"):
            parse_weights(json.dumps(doc).encode())

    def test_non_finite_rejected(self) -> None:
        """NaN weights are refused — they would fly silently otherwise."""
        doc = json.loads(_identity_artifact())
        doc["b1"] = [0.0, float("nan"), 0.0]
        with pytest.raises(ValueError, match="non-finite"):
            parse_weights(json.dumps(doc).encode())


class TestFallback:
    """No active artifact = the vehicle flies the configured PD."""

    def test_matches_rate_damping(self) -> None:
        """The fallback path IS the classical law, step for step."""
        ml = MlPolicyController(weights=None, kp=0.02, kd=0.005)
        pd = RateDampingController(kp=0.02, kd=0.005)
        state = AttitudeState(body_rates_rad_s=(0.05, -0.04, 0.03))
        ref = AttitudeReference()
        for _ in range(3):
            got = ml.update(state, ref, dt_s=0.01)
            want = pd.update(state, ref, dt_s=0.01)
            assert got.torque_n_m == pytest.approx(want.torque_n_m)

    def test_describe_says_so(self) -> None:
        """describe() states plainly that no version is active."""
        ml = MlPolicyController(weights=None, kp=0.02, kd=0.005, artifact="ml_detumble")
        assert "NO ACTIVE VERSION" in ml.describe()[0]


class TestNetwork:
    """The learned path: predictable on a hand-built network."""

    def test_near_linear_response(self) -> None:
        """A tiny-weight network reproduces its designed linear gain."""
        net = parse_weights(_identity_artifact(gain=-0.02))
        ml = MlPolicyController(weights=net, kp=0.0, kd=0.0)
        state = AttitudeState(body_rates_rad_s=(0.05, -0.04, 0.03))
        out = ml.update(state, AttitudeReference(), dt_s=0.01)
        for axis, rate in enumerate((0.05, -0.04, 0.03)):
            assert out.torque_n_m[axis] == pytest.approx(-0.02 * rate, rel=1e-3)

    def test_tracks_reference_not_raw_rate(self) -> None:
        """The network sees the ERROR — a matched reference nulls it."""
        net = parse_weights(_identity_artifact())
        ml = MlPolicyController(weights=net, kp=0.0, kd=0.0)
        state = AttitudeState(body_rates_rad_s=(0.02, 0.02, 0.02))
        ref = AttitudeReference(body_rates_rad_s=(0.02, 0.02, 0.02))
        out = ml.update(state, ref, dt_s=0.01)
        assert out.torque_n_m == pytest.approx((0.0, 0.0, 0.0))

    def test_envelope_clips(self) -> None:
        """The actuator envelope binds the learned law like any other."""
        net = parse_weights(_identity_artifact(gain=-10.0))
        ml = MlPolicyController(weights=net, kp=0.0, kd=0.0, limits=ControlLimits(0.05))
        state = AttitudeState(body_rates_rad_s=(1.0, 0.0, 0.0))
        out = ml.update(state, AttitudeReference(), dt_s=0.01)
        assert out.torque_saturated
        assert abs(out.torque_n_m[0]) == pytest.approx(0.05)

    def test_invalid_state_goes_quiet(self) -> None:
        """Garbage in, ZERO out — the learned path never flies a guess."""
        net = parse_weights(_identity_artifact())
        ml = MlPolicyController(weights=net, kp=0.0, kd=0.0)
        state = AttitudeState(body_rates_rad_s=(0.5, 0.5, 0.5), valid=False)
        out = ml.update(state, AttitudeReference(), dt_s=0.01)
        assert out.torque_n_m == (0.0, 0.0, 0.0)

    def test_finite_on_wild_input(self) -> None:
        """Saturating activations keep output bounded on absurd rates."""
        net = parse_weights(_identity_artifact())
        ml = MlPolicyController(weights=net, kp=0.0, kd=0.0)
        state = AttitudeState(body_rates_rad_s=(1e6, -1e6, 1e6))
        out = ml.update(state, AttitudeReference(), dt_s=0.01)
        assert all(math.isfinite(t) for t in out.torque_n_m)


class TestFromConfig:
    """Config parsing and slot consumption."""

    def _options(self, tmp_path: Path) -> control_options_pb2.MlPolicyOptions:
        """Options pointing at a temp slots dir.

        Args:
            tmp_path: Test-local slots directory.

        Returns:
            Options with required fields set.
        """
        options = control_options_pb2.MlPolicyOptions()
        options.artifact = "ml_detumble"
        options.slots_dir = str(tmp_path)
        options.kp = 0.02
        options.kd = 0.005
        return options

    def test_missing_fields_fail_loudly(self) -> None:
        """artifact, kp and kd are all required."""
        with pytest.raises(ValueError, match="artifact"):
            MlPolicyController.from_config(control_options_pb2.MlPolicyOptions())
        options = control_options_pb2.MlPolicyOptions()
        options.artifact = "ml_detumble"
        with pytest.raises(ValueError, match="kp and kd"):
            MlPolicyController.from_config(options)

    def test_empty_slot_flies_fallback(self, tmp_path: Path) -> None:
        """No activated version: the controller reports the fallback."""
        controller = MlPolicyController.from_config(self._options(tmp_path))
        assert controller.weights is None
        assert "NO ACTIVE VERSION" in controller.describe()[0]

    def test_active_slot_loads_network(self, tmp_path: Path) -> None:
        """An activated artifact is loaded, with its version reported."""
        from flatsat.comms.slots import SlotManager

        staged = tmp_path / "staged.bin"
        staged.write_bytes(_identity_artifact())
        SlotManager(tmp_path).activate(
            "ml_detumble", "v1", staged, ground_authority=True
        )
        controller = MlPolicyController.from_config(self._options(tmp_path))
        assert controller.weights is not None
        assert controller.version == "v1"
        assert "trained_on=unit-test" in controller.describe()[0]

    def test_corrupt_active_artifact_fails_startup(self, tmp_path: Path) -> None:
        """A bad network refuses to fly rather than flying wrong."""
        from flatsat.comms.slots import SlotManager

        staged = tmp_path / "staged.bin"
        staged.write_bytes(b'{"format": "flatsat-mlp-v1", "w0": [], "b0": []}')
        SlotManager(tmp_path).activate(
            "ml_detumble", "v-bad", staged, ground_authority=True
        )
        with pytest.raises((ValueError, KeyError)):
            MlPolicyController.from_config(self._options(tmp_path))
