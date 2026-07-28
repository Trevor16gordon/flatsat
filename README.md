# flatsat

A distributed flat-sat testbed: real flight software on real hardware, built
to learn — by construction — what modern satellite software development
actually requires. One Jetson Orin Nano flight computer on a PREEMPT_RT
kernel, two ADALM-Pluto SDRs, a physics simulator on the ground segment, and
two tracks (avionics / FDIR / space-MLOps, and adaptive ML radio) converging
at a single demo: **retrain an ML model from the system's own telemetry,
uplink it over the RF link during a contact window, activate it in a B slot,
and roll it back on command.** The satellite may never fly; the knowledge is
the product.

## The end state

What "done" looks like, concretely:

**A spacecraft is a config file.** `config/vehicles/*.txtpb` names the
sensors, actuators, control strategy, estimator, and objective — and carries
the physical model: mass, inertia, and where every device is mounted. Code
resolves those names through a registry. Swapping PD → PID, simulated → real
IMU, detumble → pointing is a config edit, never a new control loop.

**The same software flies sim, bench, and hardware.** Synthetic devices,
Basilisk-fed devices, and real devices implement the same driver contracts
and publish the same typed messages — downstream code cannot tell them
apart. The simulator builds its physics *from the same vehicle file* the
flight software reads, so the sim's spacecraft and the flight software's
model of it cannot diverge by accident — only deliberately, when a
robustness campaign injects the difference on purpose.

**Deployment is four tiers, each versioned, each rollback-able:**

| Tier | What | Changes |
|---|---|---|
| 0 | Board image (OS, RT kernel) | Rarely; A/B rootfs |
| 1 | Flight-software release — one artifact | As a unit; "the FSW version" in telemetry |
| 2 | Components installed via the registry (an uplinked ML policy is just a controller) | B-slot, activate by command, rollback on regression |
| 3 | Configuration (gains, rates, mode tables) | No code moves; config checksum in telemetry |

Safe mode's survival law ships in Tier 1 and is never hot-swapped. New
components earn authority through shadow operation and measurement — never
by deployment.

**The RF link behaves like a space link.** Intermittent, asymmetric, mostly
absent: store-and-forward CCSDS over scheduled contact windows; middleware
never tunnels over it. The PHY is swappable behind a modem contract — stock
GNU Radio blocks first (proven: framed GMSK at BER 0 over cable), then a
hand-built modem, then a learned receiver — all measured under identical
framing so the learned-vs-classical comparison is honest.

**Faults are first-class.** Every message carries validity; daemons flag and
forward, never silently repair. FDIR is a sense-decide-act loop whose plant
is the system itself: declarative limit rules → an arbiter with common-cause
grouping → a response ladder (restart → reconfigure → request Safe). ML
anomaly detectors run in shadow against injection ground truth until their
numbers justify promotion. After anything unexpected, the system rests in
its safest configuration; only deliberate human action makes it interesting
again.

**Everything is measured.** cyclictest gates under adversarial load, loop
lateness percentiles as telemetry, BER waterfalls, and a requirements system
where every claim carries its verification method and is traced to tests in
CI.

## Where things are

| Path | What |
|------|------|
| [`PLAN.md`](PLAN.md) | Architecture + working status + decision log — **§0 is the single source of truth** |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Package layout, contracts, config views, versioning tiers |
| `flatsat/` | The package — everything that ships to a flight computer |
| `config/` | Vehicles (what a spacecraft IS) and devices (datasheets + calibration) |
| `protos/` | Wire contracts between every process — source of truth |
| `requirements/` | Requirements with verification methods, enforced in CI |
| `radio/` | RF experiments bench — proven code graduates into `flatsat/comms/` |
| `tools/` | Setup, codegen, unit generation, traceability, benchmarks, host profiles |

## Setup

```bash
tools/jetson-setup.sh          # idempotent Jetson bring-up (JetPack 6.2.1)
pre-commit install             # once per clone — enables quality gates
```

## Code quality — non-negotiable

Every Python function has **typed inputs and outputs** and a **docstring**,
enforced twice (ruff `ANN`+`D`, mypy `disallow_untyped_defs`) at two points
(pre-commit, CI). Tests colocate with the code they test: one
`<name>_test.py` beside every module.

## Conventions

- System Python untouched; project venvs. No conda.
- **RF: never TX→RX without the 30 dB of pads inline. Cabled only —
  nothing transmits over the air.**
- Anything that transmits requires explicit per-instance approval.
- `extlinux.conf` edits always keep the stock kernel entry as fallback.

## Host

Jetson Orin Nano Super Dev Kit, hostname `jetson`, JetPack 6.2.1, PREEMPT_RT
kernel (`5.15.148-rt-tegra`) with stock fallback. Root on 500 GB NVMe; SD
retained as rescue boot.
