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

## Prior art, and where this could go

**Two open-source ground systems already solve much of the operations
problem**: [Yamcs](https://yamcs.org) and [OpenC3 COSMOS](https://openc3.com)
— telemetry archives, out-of-limit monitoring, command history, web UIs,
live and replay. Both are mature, both have flown, and neither should be
reinvented casually. What they would give this project today is the tedious
part rather than the novel part: fast range queries over a time-series
archive, limit monitoring with a UI, command verification stages, and years
of accumulated operational edge cases — clock correlation across domains,
out-of-order and duplicate packets, contact handover, partial frames.
Writing that code is cheap now; knowing *which cases exist* is not. Their
most valuable export here is a specification of the problem.

**What they assume is becoming untrue.** Both encode a mission-ops model in
which there is one spacecraft, the ground decides, telemetry flows down a
star topology, the parameter namespace is fixed before flight, and the
archive on the ground is eventually complete. Every one of those is under
pressure:

- **Constellations and inter-satellite links** make provenance a path rather
  than a link, and make cross-node time sync first-class.
- **Decisions taken on board** invert the model: the interesting record stops
  being a telemetry value and becomes *why* — model version, inputs,
  confidence, what was rejected.
- **ML in flight** means rollback happens in space with no ground in the
  loop. This repo's A/B slots already permit rollback without ground
  authority, for exactly that reason.
- **Agentic operations** — agents reading telemetry and commanding assets —
  need capability-scoped, auditable authority rather than human role
  accounts, and machine-readable semantics. Protobuf descriptors serve an
  agent far better than XTCE XML does.
- **Data centres in space** break "the ground eventually gets everything",
  requiring onboard triage and request-on-demand.
- **DTN / Bundle Protocol** is store-and-forward networking, not TM/TC.

**The ambition, stated plainly.** This is not a ground system and does not
pretend to be one — it has no heritage, no flown missions, and none of the
trust those buy. But its schemas are protobuf, its composition is
config-driven, sim / HIL / flight are peer implementations behind one
contract, and model artifacts are already versioned, uplinked, activated and
rolled back. Those are the pieces a successor would need. If this data model
grows run provenance, span-structured mission logs, decision records, and a
capability-scoped authority model, it becomes a plausible foundation for one
— built for constellations, onboard autonomy and agentic operation rather
than retrofitted to them. Until then, treat any existing ground tool as a
consumer of this format, never as its owner.

## Where things are

| Path | What |
|------|------|
| [`PLAN.md`](PLAN.md) | Architecture + working status + decision log — **§0 is the single source of truth** |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Package layout, contracts, config views, versioning tiers |
| `flatsat/` | The package — everything that ships to a flight computer |
| `config/` | Vehicles (what a spacecraft IS) and devices (datasheets + calibration) |
| `flatsat/**/*.proto` | ALL schemas — wire contracts (`flatsat/msgs/`) and config (colocated with owners) — source of truth |
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
