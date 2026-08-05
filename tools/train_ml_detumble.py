#!/usr/bin/env python3
"""Train the ml_detumble policy in the fastloop, gate it, emit the artifact.

The pipeline the ml_policy strategy exists to consume:

  1. EXPERT ROLLOUTS — the classical PD detumble law flies the fastloop
     (same registry composition, device models, and physics as flight)
     across randomized initial tumbles, well beyond the rates any demo
     releases at. Every control step yields a (rate error, torque) pair.
  2. OPTIONAL REAL TELEMETRY — pairs harvested from a recorded flight
     export join the dataset, so the policy is trained on the system's
     own telemetry, not only on synthetic rollouts.
  3. BEHAVIOR CLONING — a small tanh MLP (pure numpy, Adam) regresses
     torque on rate error. No torch: the network is a few hundred
     parameters and the flight side evaluates it in pure Python.
  4. CLOSED-LOOP GATE — the trained artifact is staged and ACTIVATED
     into a throwaway slots dir, and the fastloop flies the ml_policy
     strategy through the exact consumer path flight uses. Held-out
     initial conditions (including one faster than anything in
     training) must detumble to the same floor the PD reaches, or no
     artifact is written.

Usage:
  python tools/train_ml_detumble.py --version 2026-08-05a \
      [--telemetry ~/hil-trace/run-latest/mission3.json] \
      [--out build/ml_detumble/2026-08-05a.json]
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import random
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flatsat.control.attitude.controller import (  # noqa: E402
    AttitudeReference,
    AttitudeState,
    ControlOutput,
)
from flatsat.core.config import VehicleSpec, load_vehicle  # noqa: E402
from flatsat.sim.fastloop import run_fast_loop  # noqa: E402

Vec3 = tuple[float, float, float]

# The demo vehicle's detumble tuning — the expert being cloned and the
# fallback written into the vehicle file must match flatsat_v1.
KP = 0.02
KD = 0.005
MAX_TORQUE_N_M = 0.05

TRAIN_DURATION_S = 180.0
GATE_DURATION_S = 180.0
GATE_FLOOR_RAD_S = 0.005  # PD reaches ~2-3 mrad/s; require the clone under 5


def _detumble_vehicle(strategy: str, slots_dir: str = "") -> VehicleSpec:
    """Clone flatsat_v1 with its control block swapped for detumble.

    Args:
        strategy: "rate_damping" (the expert) or "ml_policy" (the student).
        slots_dir: Slots dir for the ml_policy consumer path.

    Returns:
        The modified spec (deep copy; the loaded original is untouched).
    """
    spec = load_vehicle()
    vehicle = copy.deepcopy(spec)
    control = vehicle.config.control
    for oneof in ("strategy", "objective", "estimator"):
        which = control.WhichOneof(oneof)
        if which is not None:
            control.ClearField(which)
    if strategy == "rate_damping":
        control.rate_damping.kp = KP
        control.rate_damping.kd = KD
        control.rate_damping.max_torque_n_m = MAX_TORQUE_N_M
    else:
        control.ml_policy.artifact = "ml_detumble"
        control.ml_policy.slots_dir = slots_dir
        control.ml_policy.kp = KP
        control.ml_policy.kd = KD
        control.ml_policy.max_torque_n_m = MAX_TORQUE_N_M
    control.constant_rate.target_rates_rad_s[:] = [0.0, 0.0, 0.0]
    control.passthrough.SetInParent()
    return vehicle


def _random_omega(rng: random.Random, magnitude_rad_s: float) -> Vec3:
    """A random direction scaled to the requested rate magnitude.

    Args:
        rng: Seeded generator.
        magnitude_rad_s: Desired |omega|.

    Returns:
        The initial body rate vector.
    """
    while True:
        v = (rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1))
        n = math.sqrt(sum(x * x for x in v))
        if n > 1e-9:
            return (
                v[0] / n * magnitude_rad_s,
                v[1] / n * magnitude_rad_s,
                v[2] / n * magnitude_rad_s,
            )


def harvest_rollouts(count: int, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Fly the PD expert across randomized tumbles and tap every step.

    Args:
        count: Number of rollouts.
        seed: Base seed; each rollout perturbs it deterministically.

    Returns:
        (errors, torques) arrays, one row per control step.
    """
    vehicle = _detumble_vehicle("rate_damping")
    rng = random.Random(seed)
    errors: list[Vec3] = []
    torques: list[Vec3] = []

    def tap(state: AttitudeState, reference: AttitudeReference, output: ControlOutput) -> None:
        """Record one (rate error, torque) training pair.

        Args:
            state: Estimated state at the step.
            reference: Guidance target at the step.
            output: What the expert commanded.
        """
        errors.append(
            (
                state.body_rates_rad_s[0] - reference.body_rates_rad_s[0],
                state.body_rates_rad_s[1] - reference.body_rates_rad_s[1],
                state.body_rates_rad_s[2] - reference.body_rates_rad_s[2],
            )
        )
        torques.append(output.torque_n_m)

    for i in range(count):
        magnitude = rng.uniform(0.01, 0.15)
        omega0 = _random_omega(rng, magnitude)
        run_fast_loop(
            vehicle,
            duration_s=TRAIN_DURATION_S,
            omega0=omega0,
            dt_s=0.02,
            seed=seed + i,
            step_tap=tap,
        )
        print(f"  rollout {i + 1:2d}/{count}  |omega0|={magnitude * 1000:6.1f} mrad/s", flush=True)
    return np.asarray(errors), np.asarray(torques)


def harvest_telemetry(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Harvest (rate, torque) pairs from a recorded flight export.

    The export decimates each series independently, so gyro and torque
    samples are joined on nearest timestamp within one control period's
    tolerance.

    Args:
        path: mission export JSON (flatsat.telemetry.export output).

    Returns:
        (errors, torques) arrays; the detumble reference is zero rate,
        so the gyro rates ARE the errors.
    """
    doc = json.loads(path.read_text())
    series = doc["series"]

    def joined(names: list[str]) -> tuple[np.ndarray, np.ndarray]:
        """Time-join a triplet of series on the first one's timestamps.

        Args:
            names: Three series names, x/y/z.

        Returns:
            (t_ns, values Nx3).
        """
        base_t = np.asarray(series[names[0]]["t_ns"], dtype=np.int64)
        columns = [np.asarray(series[n]["v"], dtype=np.float64) for n in names]
        ts = [np.asarray(series[n]["t_ns"], dtype=np.int64) for n in names]
        out = np.empty((len(base_t), 3))
        for k in range(3):
            idx = np.searchsorted(ts[k], base_t).clip(0, len(ts[k]) - 1)
            out[:, k] = columns[k][idx]
        return base_t, out

    gyro_t, gyro = joined(
        [f"hal/imu0/sample.gyro_{a}_rad_s" for a in "xyz"],
    )
    torque_t, torque = joined(
        [f"adcs/wheel_torque.torque_{a}_n_m" for a in "xyz"],
    )
    idx = np.searchsorted(torque_t, gyro_t).clip(0, len(torque_t) - 1)
    close = np.abs(torque_t[idx] - gyro_t) < int(20e6)  # within 20 ms
    return gyro[close], torque[idx][close]


def train_mlp(
    x: np.ndarray,
    y: np.ndarray,
    hidden: int,
    epochs: int,
    seed: int,
) -> dict[str, object]:
    """Behavior-clone the expert with a single-hidden-layer tanh MLP.

    Args:
        x: Rate errors, Nx3.
        y: Expert torques, Nx3.
        hidden: Hidden units.
        epochs: Full passes over the data.
        seed: Weight-init and shuffle seed.

    Returns:
        The artifact document (flatsat-mlp-v1), minus provenance fields.
    """
    input_scale = float(np.percentile(np.abs(x), 95))
    output_scale = float(np.percentile(np.abs(y), 95))
    xn = x / input_scale
    yn = y / output_scale

    rng = np.random.default_rng(seed)
    w0 = rng.normal(0, 0.5, (hidden, 3))
    b0 = np.zeros(hidden)
    w1 = rng.normal(0, 0.5, (3, hidden))
    b1 = np.zeros(3)
    params = [w0, b0, w1, b1]
    moments = [(np.zeros_like(p), np.zeros_like(p)) for p in params]
    lr, beta1, beta2, eps = 3e-3, 0.9, 0.999, 1e-8
    batch, step = 1024, 0

    for epoch in range(epochs):
        order = rng.permutation(len(xn))
        losses = []
        for start in range(0, len(order), batch):
            xb = xn[order[start : start + batch]]
            yb = yn[order[start : start + batch]]
            h = np.tanh(xb @ w0.T + b0)
            pred = h @ w1.T + b1
            diff = pred - yb
            losses.append(float(np.mean(diff**2)))
            # Backprop, by hand — four parameters do not need a framework.
            n = len(xb)
            g_pred = 2.0 * diff / (n * 3)
            g_w1 = g_pred.T @ h
            g_b1 = g_pred.sum(axis=0)
            g_h = g_pred @ w1
            g_pre = g_h * (1.0 - h**2)
            g_w0 = g_pre.T @ xb
            g_b0 = g_pre.sum(axis=0)
            step += 1
            for p, g, (m, v) in zip(params, [g_w0, g_b0, g_w1, g_b1], moments, strict=True):
                m[:] = beta1 * m + (1 - beta1) * g
                v[:] = beta2 * v + (1 - beta2) * g**2
                m_hat = m / (1 - beta1**step)
                v_hat = v / (1 - beta2**step)
                p -= lr * m_hat / (np.sqrt(v_hat) + eps)
        if epoch % 5 == 0 or epoch == epochs - 1:
            print(f"  epoch {epoch:3d}  mse {np.mean(losses):.6f}", flush=True)

    return {
        "format": "flatsat-mlp-v1",
        "hidden": hidden,
        "activation": "tanh",
        "input": "rate_error_rad_s",
        "output": "torque_n_m",
        "input_scale": input_scale,
        "output_scale": output_scale,
        "w0": w0.tolist(),
        "b0": b0.tolist(),
        "w1": w1.tolist(),
        "b1": b1.tolist(),
    }


def gate(artifact: bytes, seed: int) -> list[dict[str, float]]:
    """Fly the trained policy through the REAL consumer path and judge it.

    The artifact is staged and activated into a throwaway slots dir with
    the same SlotManager flight uses; the fastloop then resolves
    ml_policy from a vehicle config exactly as the control loop would.

    Args:
        artifact: The candidate artifact bytes.
        seed: Base seed for held-out initial conditions.

    Returns:
        Per-condition results (initial rate, PD final, ML final).

    Raises:
        SystemExit: When the policy misses the detumble floor on any
            held-out condition — no artifact leaves a failed gate.
    """
    from flatsat.comms.slots import SlotManager

    tmp = Path(tempfile.mkdtemp(prefix="ml-detumble-gate-"))
    staged = tmp / "candidate.json"
    staged.write_bytes(artifact)
    SlotManager(tmp).activate("ml_detumble", "gate", staged, ground_authority=True)

    pd_vehicle = _detumble_vehicle("rate_damping")
    ml_vehicle = _detumble_vehicle("ml_policy", slots_dir=str(tmp))
    rng = random.Random(seed + 1000)
    # Held-out conditions: the demo's release band, plus one HOTTER than
    # anything in training (0.18 vs 0.15 max) to check extrapolation.
    magnitudes = [0.03, 0.05, 0.07, 0.10, 0.15, 0.18]
    results = []
    failed = False
    for magnitude in magnitudes:
        omega0 = _random_omega(rng, magnitude)
        pd = run_fast_loop(pd_vehicle, GATE_DURATION_S, omega0, dt_s=0.02, seed=seed)
        ml = run_fast_loop(ml_vehicle, GATE_DURATION_S, omega0, dt_s=0.02, seed=seed)
        ok = ml.final_omega_mag_rad_s < GATE_FLOOR_RAD_S
        failed |= not ok
        results.append(
            {
                "omega0_mrad_s": magnitude * 1000,
                "pd_final_mrad_s": pd.final_omega_mag_rad_s * 1000,
                "ml_final_mrad_s": ml.final_omega_mag_rad_s * 1000,
            }
        )
        print(
            f"  |omega0| {magnitude * 1000:6.1f}  ->  "
            f"PD {pd.final_omega_mag_rad_s * 1000:6.2f}  "
            f"ML {ml.final_omega_mag_rad_s * 1000:6.2f} mrad/s  "
            f"{'ok' if ok else 'FAIL'}",
            flush=True,
        )
    if failed:
        raise SystemExit("gate FAILED: the policy does not detumble — artifact not written")
    return results


def main() -> int:
    """Run the pipeline.

    Returns:
        0 on success (artifact written and gated).
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="artifact version tag")
    parser.add_argument("--rollouts", type=int, default=48)
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--seed", type=int, default=20260805)
    parser.add_argument("--telemetry", type=Path, default=None, help="mission export JSON")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    print(f"[1/4] expert rollouts ({args.rollouts} initial conditions)", flush=True)
    x, y = harvest_rollouts(args.rollouts, args.seed)
    trained_on = f"fastloop-{args.rollouts}ic"
    if args.telemetry is not None:
        tx, ty = harvest_telemetry(args.telemetry)
        run_id = json.loads(args.telemetry.read_text())["run"]["run_id"]
        print(f"      + {len(tx)} pairs from {run_id}", flush=True)
        x = np.vstack([x, tx])
        y = np.vstack([y, ty])
        trained_on += f"+{run_id}"
    print(f"      dataset: {len(x)} pairs", flush=True)

    print("[2/4] behavior cloning", flush=True)
    doc = train_mlp(x, y, hidden=args.hidden, epochs=args.epochs, seed=args.seed)
    doc["trained_on"] = trained_on
    doc["created_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    artifact = json.dumps(doc).encode()

    print("[3/4] closed-loop gate (held-out conditions, real consumer path)", flush=True)
    gate(artifact, args.seed)

    out = args.out or Path("build/ml_detumble") / f"{args.version}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(artifact)
    sha = hashlib.sha256(artifact).hexdigest()
    print(f"[4/4] artifact {out}  ({len(artifact)} bytes, sha256 {sha[:12]}…)", flush=True)
    print(
        "\nnext:\n"
        f"  python -m flatsat.apps.uplink_send send ml_detumble {args.version} {out} --kind model\n"
        f"  python -m flatsat.apps.uplink_send activate ml_detumble {args.version} --ground\n"
        "  python -m flatsat.apps.uplink_send rollback ml_detumble",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
