# Flat-Sat Software Architecture & Development Plan

v1.2 — July 2026 · Trevor · single merged plan (architecture + working status)

**Status (2026-07-28): architecture v2 flying on the bench; config is
protos end to end.** Jetson bring-up complete (PREEMPT_RT live, 69 µs
cyclictest under load) · radio proven one-radio end to end (framed data
at BER 0, BER-vs-SNR waterfall) · **cross-machine HIL closed** on the
pre-migration stack · architecture v2 landed 07-27: `flatsat/` package,
actuator layer, Basilisk-as-drivers, telemetry recorder, mode manager
(§7), scenario harness + mission profiles · 07-28: config schemas moved
into colocated protos, all config files strictly-parsed `.txtpb`,
oneof-selected implementations, CI drift guard (**142 tests, 55
requirements**) · units reinstalled, 6 services live (quiet state) ·
pending: Mac `git pull` + HIL re-validation on the new driver chain ·
next: mode-request CLI, FDIR, A1 core isolation, ground Pluto, P3.

---

## 0. Working status and decision log

### Done (condensed 2026-07-27 — full detail lives in git history)

**Platform (N1–N3, complete 2026-07-24).** JetPack 6.2.1; rootfs on NVMe
(SD retained as rescue boot); bring-up captured idempotently in
`tools/jetson-setup.sh` + committed manifests. PREEMPT_RT kernel live
(`5.15.148-rt-tegra` via NVIDIA's OTA repo) with stock-kernel fallback;
boot-path gotchas (bootloader reads the SD extlinux + kernel; postinst
writes the wrong `root=`) captured in `tools/n3-rt-kernel-runbook.md`.
**Acceptance gate: cyclictest 10 min under sustained CUDA + stress-ng load
— 69 µs worst-case** (target <~100 µs), no isolation tuning yet. PyTorch
2.8.0 cu126 in `~/venvs/flatsat-ml` (jetson-ai-lab wheels only — pypi.org
ships CPU-only torch under the same version string; numpy held <2 for
gnuradio); CUDA gate passed with gnuradio importable in the same process.
Quality gates live: ruff ANN/D + mypy strict, pre-commit + CI.

**Radio — one-radio end to end (2026-07-23).** GNU Radio 3.10.1.1 apt +
gr-iio; flat-sat Pluto enumerated (fw v0.37, reports ad9364; P1 partial).
RX smoke clean → TX tone loopback through the 30 dB pads (+100 kHz tone,
65.9 dB above floor, +1 Hz error) → **GMSK framed data at BER 0.00**
(113/113 frames) → BER-vs-SNR waterfall via calibrated offline AWGN
(capture archived at `~/flatsat-captures/`, re-sweepable without RF — the
B2 methodology in miniature). Governing lesson: **zero-IF ⇒ keep signal
energy off DC** (BER 1.8e-1 → 0.00).

**Foundation + first services (M1, 2026-07-24).** `protos/` as the single
source of truth (committed codegen + mypy stubs); zenoh validated on-device
incl. the §7 latched-mode query pattern; `SensorDaemon` base implements
contract mechanics once (header stamping, seq, validity, absolute-deadline
cadence); first real HAL daemon (`jetson_thermal` — flag-and-forward proven
against real EAGAIN). Bus latency measured under full load: one-way p50
~170 µs / max <700 µs ⇒ **100–200 Hz control across the bus has ≥10×
worst-case margin**; bus-first ADCS confirmed.

**Real-time control loop (A1 software half, closed 2026-07-24).** 100 Hz PD
loop as a systemd service with **verified** SCHED_FIFO 80 / core 3:
wakeup-lateness MAX under full GPU+CPU+mem load **124–137 µs** (vs 1699 µs
SCHED_OTHER — the equal-priority-preemption tail SCHED_FIFO exists to
remove, removed). Residual ~100 µs tail = the A1 isolation pass's target.
Orchestration: config → `tools/gen-units.py` → committed units; systemd
grants RT/affinity/memory declaratively; Restart= is FDIR Tier-1 for free.

**Architecture v1 + traceability (2026-07-27).** Library / composition /
application layers; contracts (`SensorDriver`, `AttitudeController` taking
a REFERENCE); vehicles as `config/vehicles/*.toml` resolved by registry;
**health as protobuf telemetry** (LoopHealth/SensorHealth with config
checksum, live on the bus); **requirements traceability** (30 requirements
with verification methods, linked to tests, `--strict` in CI); 46 tests
green; PD→PID swap by config edit verified live.

**HIL closed, on-box and cross-machine (2026-07-27) — A4 exceeded.**
Basilisk 2.11 on the Mac ↔ Jetson FIFO service over WiFi, zero-config zenoh
discovery: **detumble 78 → ~2 mrad/s in ~3.5 min at 100 Hz** (A4's target
was 1 Hz). WiFi stalls visible and handled — stale-flagged inputs,
self-flagged commands, loop never destabilized; RT held through it all
(lateness p50 26 µs, MAX <500 µs). Endgame floor = the injected gyro noise:
the missing-estimator signature, textbook. Lesson: **an actuator must zero
when commands stop** (TorqueSink 100 ms cutoff). Faster-than-real-time
campaigns remain a deliberate §10 future extension.

**Migration commit 1 — restructure (2026-07-27).** Single installable
`flatsat/` package per `docs/ARCHITECTURE.md`: core (bus, config, rt,
health, registry), hardware (sensor contract, drivers/, models/),
control/attitude (controller contract + controllers/, guidance,
estimators/), apps, msgs, sim (`ground/basilisk_hil.py` →
`flatsat/sim/`). Registry moved into core with TYPE_CHECKING-only contract
imports (core imports no domain at runtime — pinned by a subprocess test).
**Estimator seam made explicit**: `StateEstimator` contract +
`passthrough` (today's behavior as a named, swappable implementation;
vehicle files default to it, no edit needed). Tests colocate 1:1 as
`<name>_test.py` beside every module and ship in the package; pytest
`python_files` + ruff pattern updated; **71 tests green** (was 46 — new
estimator/registry/driver-level coverage). `deployment.toml` at repo root,
units at `units/generated/`, `pyproject` gained `[project]` packaging.
Requirement evidence strings updated to the new file names; traceability
`--strict` clean.

**Config-as-protos rewrite (2026-07-28).** All config schemas moved into
colocated protos; every config file is now a strictly-parsed `.txtpb`
(vehicle, devices, missions). Driver/strategy/estimator selection via
oneof (field name = registry key — selector and options cannot
disagree); `from_config` signatures fully typed; gen-units reads the
typed loader; `.vscode/` ships proto tooling config; CI drift guard
keeps committed bindings honest. FSW-CFG-004 (typo-proof config) added,
test-verified. **142 tests, 55 requirements.**

**Scenario harness + mission profiles (2026-07-28) — the system-level
test tier.** A mission is data: `config/missions/*.toml` declares
initial conditions, phases (each optionally commanding a mode), and
per-phase success criteria; `flatsat/sim/scenario.py` is one generic
runner that composes the REAL flight chain in-process — sensor daemon
(sim-fed IMU), three actuator daemons projecting through their
mountings, control loop, mode manager, mode clients — all resolved from
the vehicle file through the registry, closed against
`flatsat/sim/plant.py` (pure rigid body with Euler's gyroscopic term,
parameters via the same `plant_from_vehicle` the Basilisk bridge uses).
Two missions checked in: **detumble** (0.54 → 0.0007 rad/s in 8 s
through the full chain, all acks in, NOMINAL held) and **safe-entry**
(FDIR-sourced SAFE accepted without authority + acked by every app;
unauthorized RECOVERY refused). `config/vehicles/test_scenario.toml` =
3-wheel scenario vehicle on test-only topics. The same mission files
script future real-HIL campaigns (the Mac bridge stands behind the same
topics). FSW-SYS-001…003 test-verified; **138 tests, 54 requirements**.

**Mode manager (2026-07-27) — mission state as a service (§7 realized).**
`flatsat/mode/`: pure `ModeStateMachine` (explicit transition graph
Init → Nominal ⇄ (Safe → Recovery); toward-safety needs no authority and
no dwell, away-from-safety is ground-command-only and dwell-guarded;
Safe→Nominal must go through Recovery; safe-entry counters for flap
escalation — every rule pinned by clock-injected unit tests) +
`ModeManager` bus shell (latched `sys/mode` broadcast + queryable for
late joiners, `sys/mode/request` with the authority gate,
`sys/mode/ack` tracking with missing-ack surfacing in ModeHealth) +
`ModeClient` (late-join query, auto-ack) now wired into every app main.
Boot policy: clean-shutdown marker consumed at startup — unexpected
reset boots SAFE, only a deliberate shutdown allows Init→Nominal.
Recorder defaults now archive `sys/**` (transitions are data products).
Mode topics are parameterizable so tests use test-only keys.
`flatsat-mode.service` generated (7 units total). FSW-MODE-001…008
test-verified; **130 tests green**.

**Telemetry recorder (2026-07-27) — the first flight organ after
migration.** `flatsat/telemetry/recorder.py` + `apps/telemetry_recorder`:
subscribes the vehicle file's `[telemetry]` topics (default `hal/**`,
`adcs/**`, `health/**`) and archives bus traffic VERBATIM — topic,
arrival time, byte-exact payload, never parsed — as
`[len][RecordedSample]` frames in `telemetry-<stamp>-<seq>.rec` files.
Bounded by design: rotate at 64 MiB / 15 min, prune oldest past a 4 GiB
cap (never the live file) — an indefinite run cannot fill the disk.
`read_records()` treats a truncated tail (crash mid-write) as
end-of-file, pinned by test. RecorderHealth on `health/recorder` (which
the recorder itself records). `flatsat-recorder.service` generated.
FSW-TLM-001…005 added, test-verified; **113 tests green**. Regression
comparison, FDIR inputs, and the ML corpus are now unblocked.

**Migration commit 3 — Basilisk as drivers (2026-07-27).** The universe
fake now shows up to flight software as ordinary drivers: `basilisk_imu`
(subscribes the bridge's `sim/truth/state`, corrupts through the SHARED
`hardware/models/imu.py` + device spec; no fresh truth ⇒ STALE-flagged
zeros at full cadence) and `basilisk_reaction_wheel` (shared
`hardware/models/wheel.py` envelopes — extracted so both wheel fakes
saturate identically — publishing POST-envelope applied torque on
`sim/wheel/<name>/torque` for the bridge). `sim_gyro` deleted;
`sim_reaction_wheel` stays registered as the local-only alternative.
Bridge slimmed to physics + truth topics; its plant (mass, inertia,
per-wheel axis mapping) is DERIVED from the vehicle file
(`plant_from_vehicle`, pinned by test — FSW-SIM-003) and its per-wheel
sinks keep a stale cutoff as defense in depth behind the daemon's
zeroing. New `protos/sim.proto` (TruthState, WheelAxisTorque); driver
contracts gained a no-op `close()` for bus-reached devices. One vehicle
file now serves bench and HIL. FSW-SIM-003/004 added; **107 tests
green**.

**Migration commit 2 — actuator layer (2026-07-27).** The write side now
mirrors the read side: `ActuatorDriver` contract (`apply`/`state`, never
raises) + `sim_reaction_wheel` (momentum integrates against the wall
clock, clips+RANGE beyond the torque envelope, SATURATED at the momentum
rail — envelopes from `config/devices/wheel0.toml`). Generic
`actuator_daemon` owns the two protections every actuator inherits:
**stale-command zeroing** (moved from the sim's TorqueSink into the
flight path where it belongs) and the **mounting projection** (body-frame
torque → this device's axis; no central mixer). Vehicle file gained
`[body]` (mass + inertia, the sim-bridge plant source for commit 3) and
`[[actuators]]` with per-device `mounting`; `config/imu0.toml` moved to
`config/devices/` per the device-intrinsic split. New `WheelState` +
`ActuatorHealth` protos; units now generate for actuators
(`flatsat-wheel0.service`). Six new requirements (FSW-ACT-001…006) all
test-verified; **93 tests green**.

### Decision log

| Date | Decision | Rationale / consequence |
|------|----------|-------------------------|
| 2026-07-28 | **Config schemas are protos; config files are textproto.** Schemas colocated with their owners (`flatsat/vehicle.proto`, `devices.proto`, per-driver/strategy options protos); `config/*.txtpb` instances bound by `# proto-file:` headers; implementations selected by oneof — the FIELD NAME is the registry key. Strict parsing: unknown fields fail startup (FSW-CFG-004). Committed colocated bindings, CI drift guard. TOML remains only for deployment.toml + requirements (tooling, not flight config). | One edit point for wire AND config: proto → Python stubs (mypy/IDE) → C++ later → editor validation of configs. Kills the silent-typo options dict. Config-as-proto is also the Tier-3 uplink wire format for free. `Any` escape hatch reserved for uplinked Tier-2 components when C1 needs it. |
| 2026-07-23 | **No conda.** GNU Radio self-managed; stay on apt 3.10.1.1 for now (deviates from the radioconda 3.10.11+ pin in v1.0 §3). | Simpler env story (system python + venvs only). Open risk: Mac ground modem must version-match before any cross-machine BER baseline; revisit before B2. |
| 2026-07-23 | **Pluto firmware stays v0.37** for now; v0.39 upgrade deferred to just before comparative baselines. | Unit is modern + already reports ad9364; flashing is the riskiest Pluto op — defer until it buys something. P1 exit amended accordingly. |
| 2026-07-23 | **Serial→role assignment.** `104473b04a06000602001c00dd1f84cfaa` = **flat-sat radio** (on the Jetson). Second unit = **ground radio** (eventually on the Mac). | Recorded per §2; ground segment on Mac is future work — both radios live on the Jetson until the ground segment stands up. |
| 2026-07-23 | **Repo layout by architecture segment** (`flight/`, `ground/`, `radio/`, `tools/`); plans merged into this single PLAN.md. | Mirrors the two-segment architecture from day one. |
| 2026-07-27 | **Repo layered library / composition / application**; a spacecraft is a config file (`config/vehicles/*.toml`) resolved by name through a registry, not a code path. | Answers "different flight computer + N sensors + PID vs ML control" by composition. Deployment (RT/memory/core) and host topology stay separate so one vehicle definition deploys to different boards. Registry indirection is also the hook for remotely-installed components. |
| 2026-07-27 | **Health is telemetry, not printf** (`protos/health.proto`), tagged with config checksum. | Recorder, FDIR, regression comparison, and the ML corpus all need the loop's own vitals as data; a window whose parameters can't be reconstructed is not evidence. |
| 2026-07-27 | **Requirements carry an explicit verification method**; test-verified ones are linked to tests by marker and enforced in CI. Analysis/inspection/demonstration are named as such. | Turns measurements already made (cyclictest, BER, bus latency) into an auditable spec. Calling a 10-minute load campaign a unit test would be worse than naming the method. |
| 2026-07-27 | **Basilisk clock-sync module NOT used**; the HIL feed paces itself with the same absolute-deadline pattern as the flight loops. | The bridge must hook every step anyway (read state, publish, ingest torque); one pacing idiom repo-wide. Lost time is dropped, never replayed — a faster-than-real burst would present the controller with dynamics it will never see. |
| 2026-07-24 | **ADCS language: Python first, C++ only when measurement demands it.** Port trigger is a number, not a feeling: worst-case loop lateness under adversarial load exceeding ~5% of the period, or a rate target beyond Python's floor (~1 kHz / sub-100 µs). | 100–200 Hz has ms-scale budget; disciplined Python (prealloc, gc frozen, mlockall, FIFO) holds ~100–300 µs jitter on the RT kernel. The bus-topic seam makes the later port invisible to the rest of the system; hard-RT tier belongs to the STM32 judge anyway. Jitter harness runs from day one so the before/after is free. |
| 2026-07-27 | **Architecture v2: one installable package `flatsat/`** — `flight/`, `ground/`, and `hal` dissolve into core / hardware / control / comms / mode / telemetry / apps / msgs / sim. `cpp/` and `cdh/` (F´) created on first consumer, never before. Full layout: `docs/ARCHITECTURE.md`. | The package IS the Tier-1 versionable artifact; where code runs is deployment's decision, not the tree's. Self-describing imports. |
| 2026-07-27 | **Versioning is four tiers**: (0) board image via A/B rootfs; (1) FSW release, one artifact; (2) components installed via registry; (3) config. Safe mode's survival law ships in Tier 1, never Tier 2. | Hot-swap mechanism is uniform; authority is gated (shadow → measured → authorized). C1 is one traversal of Tier 2. Mixed-language releases favor a container as the eventual release format. |
| 2026-07-27 | **Actuator layer mirrors the sensor layer**: `ActuatorDriver` contract; generic actuator daemon owns stale-command zeroing + mounting projection. Controller publishes body-frame torque; no central mixer until joint allocation is actually needed. | The wheel was configured but nothing consumed its commands flight-side; the TorqueSink protection moves into the flight path where it belongs. |
| 2026-07-27 | **Physical truth splits device-intrinsic vs integration.** `config/devices/*.toml` = datasheet envelopes + per-serial measured calibration; vehicle file = `[body]` mass/inertia + per-device `mounting` (later: `[power]`, bus-topology fields). A view exists only when a consumer exists. | Designed vs discovered knowledge kept separate; flight uses nominal ⊕ calibration. Five views named (mechanical, electrical, data, thermal, compute); three deferred with named future consumers. |
| 2026-07-27 | **Basilisk shows up as drivers** (`basilisk_imu`, `basilisk_reaction_wheel`); `sim_gyro` deleted; the bridge slims to physics + truth topics and builds its plant FROM the vehicle's physical model. | Kills the stop-the-daemon HIL footgun; flight-clock timestamps, health continuity, and staleness handling stay in the daemon machinery (§10's time rule actually honored). Sim/flight mismatch impossible by construction; deliberate `truth_overrides` later = robustness campaigns. |
| 2026-07-27 | **FDIR lives at `control/health/`** — same sense-decide-act shape as attitude control, plant = the system. **ML gets no silo**: runtime in `core/inference.py`; models register into existing registries (controller / detector / modem). | One pattern everywhere: contract → named implementations → thin app. ML is never architecturally special, which is what makes the promotion gate enforceable. |
| 2026-07-27 | **Tests colocate**: one `<name>_test.py` beside every module, 1:1. | pytest `python_files` + ruff pattern updates; tests ship in the Tier-1 artifact → an installed release can self-test on target. |

### Current state (2026-07-27, post-design session)

**Live right now.** Jetson on the RT kernel (`5.15.148-rt-tegra`);
`flatsat.target`'s RUNNING services still hold the pre-migration code
images (v1 layout, sim_gyro). After migration commits 1–3 the units are
regenerated for `flatsat.apps.*` — now four services (imu0 basilisk_imu
daemon, thermal_tj daemon, wheel0 actuator daemon, ADCS loop) — and need
`sudo ./tools/install-units.sh && sudo systemctl restart flatsat.target`
(Trevor) to pick up the new layout. Expected bench state with no Mac:
imu0 STALE-flagged at cadence, commands self-flag, wheel zeroed — that
is correct quiet-state behavior, not a bug. Mac (`~/venvs/flatsat-ground`,
Basilisk 2.11 + Vizard) needs `git pull` before the next HIL run.

**Where to start reading:** `README.md` (the end state), then
`docs/ARCHITECTURE.md` (target layout, contracts, config views, versioning
tiers), then `config/vehicles/flatsat_v1.toml` (what the spacecraft IS),
then `requirements/` + `tools/traceability.py` (what it must do and how
each claim is verified).

**Everyday commands.**
- tests: `~/venvs/flatsat-ml/bin/python -m pytest -q`
- traceability: `~/venvs/flatsat-ml/bin/python tools/traceability.py`
- after editing a vehicle/deployment file:
  `~/venvs/flatsat-ml/bin/python tools/gen-units.py` then
  `sudo ./tools/install-units.sh && sudo systemctl restart flatsat.target`
- after editing a `.proto`: `./tools/gen-protos.sh` (bindings are committed)
- HIL: Mac `python -m flatsat.sim.basilisk_hil --closed-loop [--viz]` —
  after migration commit 3, no services need stopping (Basilisk is a
  driver swap, not a topic squatter).

**Gotchas learned the hard way (all encoded in code/tests now).**
- sysfs EAGAIN evades `except OSError` through Python's text and buffered
  IO layers — use raw `os.read`.
- Zero-IF: never put signal energy at the LO frequency (BER 1.8e-1 → 0).
- An actuator must zero when commands stop, or it keeps flying a dead
  controller's last order.
- Vizard is the CLIENT'S peer: the sim dials out, and Basilisk's default
  `0.0.0.0` is undialable on macOS — pass `--viz-address localhost`.
- Tests must use test-only topics; a production key silently reads live
  daemon or HIL traffic.
- Report VERIFIED scheduling state, not what the process requested.

**Known gaps, deliberately open.** No telemetry recorder (health messages
flow; nothing archives them). No estimator (the ~2 mrad/s detumble floor is
gyro noise the controller chases). No mode manager, no fault injection.
Config has no proto schema, so parameters cannot be uplinked yet. No
packaged Tier-1 artifact (deployment is a git checkout). Second Pluto
still boxed.

### Next up

1. ~~Migration commit 1 — restructure~~ **DONE 2026-07-27** (see §0 Done).
   Remaining hands-on: `sudo ./tools/install-units.sh && sudo systemctl
   restart flatsat.target` (Trevor) and Mac `git pull`.
2. ~~Migration commit 2 — actuator layer~~ **DONE 2026-07-27** (see §0
   Done).
3. ~~Migration commit 3 — Basilisk as drivers~~ **DONE 2026-07-27** (see
   §0 Done). Cross-machine HIL re-validation with the new driver chain
   still pending (needs the Mac).
4. ~~Telemetry recorder~~ **DONE 2026-07-27** (see §0 Done).
5. **A1 core isolation** (sudo + reboot session, N3-style runbook):
   isolcpus/nohz_full/IRQ affinity/idle-state cap on RT_CORE from the host
   profile, then re-run the cyclictest gate and the loop's own report
   against the recorded baselines (69 µs cyclictest; loop FIFO-under-load
   MAX 124–137 µs).
6. **Unbox ground Pluto** (Trevor, ~10 min): record serial into the §2 role
   table; resolve the two-Pluto identity collision (both default to
   192.168.2.1) before any two-radio work.
7. **P3**: BPSK/framing over the loopback (zero-IF lesson applies),
   graduating proven PHY into `flatsat/comms/`. (~~Mode manager~~ **DONE
   2026-07-27**, see §0 Done — the `sys/mode` service with broadcast+ack
   is live in code; needs the unit reinstall to run as a service.)
8. ~~Scenario harness + mission profiles~~ **DONE 2026-07-28** (see §0
   Done). Natural extensions when wanted: fault-injection phases (kill
   truth mid-mission, assert flag-and-forward + safing), criteria read
   from recorded telemetry, and running a mission profile against real
   Basilisk HIL via the Mac.

Operational notes for working sessions: Claude Code runs as `trevor` directly
on the Jetson but has **no passwordless sudo** — privileged steps (apt install,
nvpmodel, kernel work) are run by Trevor. Workflow convention: scripts get
written to the repo for inspection first, run only after go-ahead; anything
that transmits requires explicit per-instance approval.

---

## 1. Purpose and philosophy

This project builds a distributed flat-sat testbed to learn, by construction,
what modern satellite software development actually requires. The reference
point is Muon Space's stack (MuonOS): a Linux-based, telemetry-first,
data-centric architecture where the same software runs against simulation and
hardware, and where software delivery to orbit is safe and routine. The
satellite may never fly; the knowledge is the product. Two tracks run in
parallel — Track A (avionics, FDIR, space-MLOps) and Track B (adaptive ML
radio) — converging at milestone C1: redeploying an ML model over the RF link
with rollback.

Three design principles govern everything below. First, **seams over
features**: the architecture's value is in its boundaries (HAL seam, link seam,
F´/bus bridge), because seams are where simulation, fault injection, and future
replacement happen. Second, **the quiet state is the default**: after anything
unexpected, the system rests in its safest configuration and only deliberate
human action makes it interesting again. Third, **trust is earned by
measurement**: no ML component gets authority over the system until it has run
in shadow mode against ground truth and posted numbers.

An important honest framing on the MuonOS question: public evidence indicates
Muon built its flight software entirely in-house — their MuOS is described as
data-centric, IP-based middleware unifying space, ground, and cloud, and
nothing in their materials or hiring suggests F´ or cFS heritage. This project
nevertheless uses F´ deliberately (§5): as a reference implementation of the
canonical C&DH service inventory, to be progressively understood and, where
instructive, reimplemented bus-natively. The endpoint — a data-centric stack
where framework services have been replaced by owned equivalents — is the "my
own MuonOS" goal; F´ is scaffolding, not destination.

## 2. Hardware baseline

The flight computer is an NVIDIA Jetson Orin Nano Super Dev Kit (8 GB, 6×
Cortex-A78AE, Ampere GPU), booted from the JP6.2.1 SD image with rootfs
migrated to a Fanxiang S690Q 500 GB NVMe (Gen4 drive negotiating to the Orin's
Gen3 — ~3 GB/s, entirely sufficient; the SD card is retained as a golden
recovery image). The radio segment is two ADALM-Pluto SDRs — one flat-sat, one
ground station — with serial numbers recorded against roles:

| Role | Serial | Host | Firmware |
|------|--------|------|----------|
| Flat-sat radio | `104473b04a06000602001c00dd1f84cfaa` | Jetson | v0.37 (Rev.C, reports ad9364) |
| Ground radio | *(record when unboxed)* | Mac (future; Jetson until ground segment exists) | TBD |

Firmware pin: v0.39 + ad9364 unlock *before comparative baseline measurements*
(deferred for now — see decision log). RF passives on hand: 3× 10 dB SMA pads
(30 dB total), short SMA M-M cables, F-F barrels, 50 Ω terminators. Cabled
only; nothing transmits over the air. **Never TX→RX without pads inline.**

Sensors (IMU, magnetometer, thermal, GNSS, camera) attach via I2C/USB per the
BOM v6 selections. The independent hardware judge — STM32 NUCLEO-H723ZG plus
eFuse power switching (~$300) — is deferred until the physical-fault milestone;
until then its role is documented as an explicit gap. The development and
ground-segment host is a Mac (no x86 Linux box on hand), which runs Basilisk,
the GNU Radio ground modem, the F´ GDS, mission control, and ML training;
on-device flashing/migration procedures are used in place of SDK Manager.

Compute rationale (from the original hardware decision): the Jetson was chosen
over the AMD Kria KV260 because the work's center of gravity moved from
flat-sat realism toward learned/ML-native radio, where the KV260's DPU
(CNN-only, frozen at Vitis AI 3.5, no transformers) was the weakest flank and
top project risk. Native PyTorch/TensorRT, higher SDR-hosting throughput, an
installable PREEMPT_RT kernel, native IMX219 camera support, and lower cost
($249) won. What was forfeited — FPGA fabric and an on-die RTOS island — is
either not needed by the current thesis (FPGA) or relocated to the STM32 judge
(hard-real-time island, cleaner fault separation). Each Pluto is USB-2-limited
to ~4–8 MS/s sustained, which is ideal: narrowband, D2D-relevant waveforms
(GSM ~200 kHz, NB-IoT 180 kHz) fit comfortably and match the interference/D2D
thesis.

## 3. Version matrix and lifecycle plan

All pins verified July 2026 unless amended (see decision log). The JetPack
choice cascades into everything: it fixes Ubuntu, kernel, Python, CUDA, and
therefore every wheel above.

| Layer | Pin | Rationale / notes |
|-------|-----|-------------------|
| BSP | JetPack 6.2.1 (L4T r36.4.x; device reports r36.4.7) | Ubuntu 22.04, kernel 5.15, CUDA 12.6, TensorRT 10.3; the mature path. JP7.2 deliberately deferred |
| RT kernel | `nvidia-l4t-rt-kernel` + headers + oot-modules + display, via NVIDIA's OTA apt repo | No custom kernel build. "Developer Preview" quality; generic kernel stays in extlinux.conf as boot fallback, always |
| Python (onboard) | 3.10 (fixed by 22.04) | All onboard wheels must be cp310 |
| PyTorch | torch 2.8.0 / torchvision 0.23.0 from the jp6/cu126 index at pypi.jetson-ai-lab.io | Only these wheels match JP6.2; some want numpy<2 |
| Inference | onnxruntime first; TensorRT 10.3 when latency demands it | FDIR at ~1 Hz does not need TensorRT |
| Middleware | Zenoh (rmw_zenoh available if ROS 2 interop wanted) | Humble EOL May 2027 is a migration driver |
| GNU Radio | **AMENDED 2026-07-23: apt 3.10.1.1, self-managed, no conda.** (v1.0 pinned radioconda 3.10.11+ — overridden by no-conda decision) | Open risk: Mac ground modem must version-match before cross-machine BER baselines; revisit before B2 |
| libiio | 0.2x line (device has 0.23) | In-tree gr-iio targets 0.x; the libiio 1.x API rewrite pairs only with newest source builds — don't mix |
| Pluto firmware | **AMENDED 2026-07-23: v0.37 for now; v0.39 before comparative measurements** | Flash both units to the same version before any baseline |
| Flight core | F´ v4.2.0 via fprime-bootstrap (pins matching fprime-tools/fpp) | v4 is a breaking change from v3 — pre-late-2025 tutorials silently mislead. Trust only versioned official docs |
| Simulator | Basilisk 2.11.x on the Mac | Never on the Jetson; bridged over Ethernet (§10) |
| Ground MCS | Yamcs 5.x (or OpenC3 COSMOS) | Native CCSDS handling pairs with F´ v4 framing |

Lifecycle: three clocks expire in 2027 (Ubuntu 22.04, ROS 2 Humble, NVIDIA's
attention shifting to JP7). The plan treats this as a feature: a named 2027
milestone performs the fleet migration to JP7.2/24.04 with A/B rollback and no
loss of the FDIR corpus pipeline — a realistic space-MLOps exercise.

## 4. Architecture overview

Two segments, two buses, never merged. The flat-sat runs an onboard Zenoh bus;
the ground segment runs its own bus plus mission database. Exactly two things
connect them: the RF link (Pluto↔Pluto, existing only during scheduled contact
windows) and the lab-Ethernet sim seam (Basilisk feeding fake sensors).
Everything else in the system is a process on one bus or the other.

**Layering.** L0: hardware, or Basilisk standing in for it. L1: HAL daemons —
one process per device owning driver-level access (I2C/SPI/IIO), publishing
typed topics; no application ever touches a hardware register. The
fault-injection shim lives here. L2: transport — Zenoh for the general plane;
the 100–200 Hz ADCS path either stays within a single pinned process or uses
shared-memory transport, never forced through the same machinery as image
blobs. L3: services — C&DH spine (F´, §5), mode manager (§7), FDIR (§8),
telemetry recording. L4: applications — ADCS, payload, anomaly scorer, link
agent. L5: link layer — CCSDS framing over GNU Radio to the Pluto, gated by the
contact scheduler (§6). Ground mirrors this: modem, link-service mirror, ground
bus, Yamcs, pass scheduler, Basilisk, ML training, CI.

**HAL daemon contract.** Every sensor message carries: sample timestamp,
publish timestamp, a validity word (range check, comm/CRC status, freshness),
and a monotonic sequence number. Daemons flag and forward — they never silently
repair data, because FDIR cannot detect what the HAL papers over. The topic
name and message schema are the entire interface: a Basilisk-fed daemon
publishing the same schema is indistinguishable downstream, which is the whole
simulation strategy. The injection shim interposes on any topic on command:
corrupt, bias, delay, drop, freeze (replay-last), or kill — each campaign
scripted through the normal command path so ground truth is logged
automatically.

**Middleware discipline.** Zenoh peer mode onboard (brokerless — there is no
broker process to crash or pin; note DDS is likewise brokerless, a common
misconception). Latched/queryable state topics (`sys/mode`) so respawned
processes learn current state immediately. Shared-memory allocations tuned down
per-node from Zenoh's large defaults — a dozen daemons plus PyTorch on 8 GB
requires explicit per-service memory budgets, which is also authentic
flight-software hygiene. Big payloads (images, IQ) travel by shared memory or
buffer-handle reference with best-effort QoS, so a fat tensor can never delay a
control message.

**Real-time strategy.** Determinism comes from the OS, not from special code:
PREEMPT_RT kernel (OTA debs), the ADCS process under SCHED_FIFO (priority >90)
pinned to a dedicated core with isolcpus/nohz_full/rcu_nocbs and IRQ affinity
steering interrupts away. The known threat on Orin is not CPU contention but
memory-bandwidth contention from the GPU: the N3 acceptance test therefore runs
cyclictest under combined CUDA + memory-bandwidth load, targeting <~100 µs
worst-case. Documented community results (~50–120 µs quiet, 300–500 µs spikes
under unpinned interrupt load) say this is achievable but only with IRQ
affinity done properly. Fallback if the gate fails: relax the ADCS loop rate
(LEO attitude dynamics tolerate 10–50 Hz easily) and assign hard-real-time
duties to the STM32 later. Orchestration is Docker/Podman supervised by systemd
(Restart=, WatchdogSec=, cgroup pinning) — deliberately not k3s, whose
control-plane overhead and multi-node purpose don't fit a single flight
computer; Kubernetes belongs in the ground segment if anywhere.

## 5. C&DH layer: F´ v4.2

One F´ deployment process owns the C&DH spine and radio path. The service
inventory it provides, and what each means here: command dispatch (opcode
routing, dictionary validation, ack semantics); the command sequencer
(uploaded, time-tagged sequences — the heart of overpass operations);
channelized telemetry with v4.2's named streams — REALTIME vs RECORDED with
per-group on-change/rate-limited/heartbeat policies, which is the live-pass vs
store-and-forward split as configuration; events/EVRs (ordered,
severity-filtered — every FDIR decision becomes ground-visible); the parameter
database (all tunables — FDIR thresholds, mode tables, ADCS gains — settable by
command, never reflash); file uplink/downlink/management (the C1 deployment
path); data products (the onboard anomaly corpus: recorded, cataloged,
priority-dumped during contacts); CCSDS framing with prioritized com queues
(command acks never wait behind payload data); health pinging with watchdog
hook; rate groups (the deterministic timing skeleton); and buffer management
(static memory discipline). Two things F´ deliberately lacks — a system mode
manager and real FDIR — are built custom (§7, §8), which is correct: they are
the research.

**The bridge pattern.** A single F´ component bridges to the Zenoh plane: bus
topics in → telemetry channels and events; F´ commands in → bus publications
out. HAL daemons, ADCS, and ML services never link against F´ — they stay
bus-native, restartable, Python-friendly. Placement rule: anything that must
survive contact-window ops (commands, sequences, telemetry, files) lives
F´-side; anything experimental or GPU-adjacent lives bus-side. Over time, F´
services may be reimplemented bus-natively one at a time behind the bridge —
the MuonOS trajectory of §1.

**On-ramp:** F´ builds natively on macOS (develop + GDS on the Mac; the Orin
self-hosts its own builds). Sequence: reference deployment with GDS → strip to
a minimal FlatSatCdh topology (dispatcher, sequencer, telemetry, events,
params, health, rate groups) → one trivial custom component to learn the
FPP→autocode→wire loop → swap the com driver from TCP to the CCSDS/Pluto path.

## 6. Link layer and the two-bus rule

A space link is intermittent, asymmetric (kbps up, Mbps down), high-BER, and
mostly absent — so the middleware is never tunneled over it; discovery chatter
and reliable-QoS retransmission would melt down, and transparency would erase
exactly the operational structure this project exists to learn. Instead, the
onboard link service subscribes to telemetry topics, frames into CCSDS, stores
between passes, and transmits during windows; the ground mirror deframes and
republishes onto the ground bus. To every other process the link is invisible;
underneath it is an explicitly managed, scheduled, store-and-forward pipe. The
correctness test: everything keeps working with the link down, just with stale
ground data. The contact scheduler enforces windows even though both Plutos sit
on one desk — simulated pass geometry (eventually driven by Basilisk orbit
propagation) gates the radio, telemetry accumulates between passes and dumps on
contact, commands queue ground-side. Track B plugs in underneath: the classical
GNU Radio modem is the baseline PHY (BER-vs-SNR curves recorded under the
locked environment), the learned receiver later swaps in under identical
framing, and the adaptive link agent — subscribing to link telemetry,
publishing modcod recommendations the link service may honor — is disabled by
Safe mode by definition.

## 7. Mode management

Four flat states: Init/Boot → Nominal ⇄ (Safe → Recovery). The transition
asymmetry is the philosophy: toward safety is automatic and
software-triggerable from anywhere (FDIR trip, watchdog, failed init checks);
away from safety is ground-command-only. Recovery is distinct from Nominal so
checkout happens payload-off, one subsystem at a time; Recovery can trip back
to Safe. Orthogonal conditions (in-contact, sim-fed) are flags, never modes —
modes multiply combinatorially and tax every app.

A mode is a contract each app implements locally: Nominal = payload + ML
active, full telemetry policy, adaptive link permitted; Safe = load shed, ADCS
in survival law, low-rate housekeeping heartbeat, most-robust modcod locked.
Distribution is hybrid broadcast-plus-ack: mode published latched on `sys/mode`
with monotonic sequence and reason; every registered app must ack the sequence
within a timeout; a missing ack is itself a fault escalating to supervisor
kill. Per-app mode responses live in the parameter database as a mode table
(app × mode → config), tunable by command and sweepable by fault campaigns. The
mode manager is small, boring, and dependency-minimal — no imports from the
experimental world — because Safe's entry path must not depend on anything that
can be the reason for entering it. Below it sit the containment shells:
FDIR→mode manager (software), health/watchdog (process), STM32 power-cycle to a
Safe-booting slot (hardware, once installed). Anti-flap: minimum dwell times,
persistent transition counters with escalation on repeat entry, every
transition logged as event + data product with triggering telemetry snapshot
(free corpus labels). Boot policy: any unexpected reset (reset-reason flag in
persistent storage) boots into Safe; only a clean prior shutdown allows
Init→Nominal.

## 8. FDIR — basic tier

Scope decision: the flat-sat builds a basic FDIR first; the SBIR three-tier
stack (AUKF, transformer, LLM reasoning) comes later on the same seams. The
pipeline is two parallel detectors feeding an arbiter feeding a response
ladder.

Level 0 is the HAL validity word (§4). Level 1 is the limit checker: one
service, declarative YAML/param rules of four types — static bounds,
rate-of-change, staleness, cross-channel consistency — each with an N-of-M
persistence requirement, emitting fault flags naming rule, channel, value.
Deliberately dumb; its dumbness is its trustworthiness. The arbiter maps flags
to suspects via a topology table (channel → {device, daemon, shared bus}),
performs common-cause grouping (all I2C channels faulting = the bus, not three
devices), fuses detectors, counts strikes, and rate-limits. The response
ladder: Tier 1 restart the implicated daemon; Tier 2 reconfigure around it
(mark invalid, switch redundant source, GPIO power-cycle); Tier 3 request Safe
from the mode manager — FDIR is a client of mode authority, never the owner.
Responses are table-driven; every action emits an event with snapshot.

Level 2, shadow-mode ML: per-channel rolling z-scores and EWMA drift on a
windowed feature vector first; then at most a small autoencoder (reconstruction
error as score) via ONNX/onnxruntime. It publishes `fdir/anomaly` (score,
contributing channels, model + threshold version) which the arbiter logs but
cannot act on. Promotion to response authority happens only after weeks of
shadow operation measured against the limit checker and injection ground truth
— the advisory→measured→authorized gate is the single most transferable lesson
in the subsystem.

Starter fault set, one per detector class: frozen sensor (replay-last; caught
by staleness/variance, invisible to bounds), thermal runaway (injected ramp;
rate rule), daemon death (process kill; heartbeat, auto-fixed by Tier 1 — the
first closed-loop demo). Standing metrics from day one: detection latency
(injection→flag) and time-to-recovery (flag→nominal), per fault class.

## 9. ML and data flywheel

The loop: telemetry recorder + injection campaigns → labeled anomaly corpus
(onboard as data products, dumped by priority, archived ground-side) → training
(PyTorch on Mac/cloud) → model registry (versioned artifacts: weights, feature
spec, thresholds, training-set hash) → uplink via CCSDS file transfer → A/B
model slots onboard (activate-new, rollback-on-regression) → inference
publishing scored messages that are themselves telemetry → back into the
corpus. Two properties make it honest: predictions are recorded beside the
inputs that produced them (self-growing training set as an architectural side
effect), and deployment goes through the real link machinery with rollback — C1
is exactly one traversal of this loop's right-hand side. Every model output
message carries its model version; every response ties back to the rule or
model that triggered it.

## 10. Simulation

Basilisk 2.11.x runs on the Mac, never the Jetson (RAM, wheel support, and —
decisively — the sim computer should not be the flight computer). The seam is
the HAL: Basilisk-fed daemon processes publish identical schemas on the same
topics, over lab Ethernet. Time is the tricky part and gets decided early: the
flat-sat runs on wall-clock; Basilisk runs real-time-paced so the flight
software never knows sim time exists; sample timestamps come from the
publishing daemon in flight-clock terms. (Faster-than-real-time campaigns are a
later, deliberate extension — they require the flight software to accept an
external time source, which touches everything.) Basilisk's orbit propagation
eventually drives the contact scheduler's pass geometry, closing the loop
between simulated orbits and simulated comms windows.

## 11. Testing, CI, and code quality

Three test tiers. Unit: F´'s autogenerated component harnesses plus pytest for
bus services. Integration/HIL: the F´ GDS Python API drives scripted
command/telemetry regressions against the live flat-sat from the Mac — commands
in, expected events and channels out — runnable on every merge. Campaign:
scripted fault-injection sweeps (fault class × mode × load level) producing
corpus data and regression-checking the standing FDIR metrics; RF-path
regressions re-run the classical BER-vs-SNR baseline under the locked
environment whenever the modem changes. Latency gates (cyclictest under load)
run as scheduled jobs so RT regressions are caught when introduced, not when
noticed.

**Code quality gates (live as of 2026-07-23).** Every Python function must
have typed inputs and outputs and a docstring — enforced twice: ruff with the
full `ANN` (flake8-annotations) and `D` (pydocstyle, Google convention) rule
sets, and mypy with `disallow_untyped_defs`/`disallow_incomplete_defs`.
Enforcement points: pre-commit git hooks (install once per clone:
`pre-commit install`) and the `quality` GitHub Actions workflow re-running the
same hooks on push/PR, so a locally bypassed commit still fails remotely.
Config lives in `pyproject.toml` + `.pre-commit-config.yaml`; tooling installs
via pipx in `tools/jetson-setup.sh`. Known relaxations: `test_*.py` files skip
module/function docstring rules (types still enforced); leading-underscore
(private) modules relax docstring rules per Python convention.

## 12. Milestones

| # | Milestone | Exit criterion |
|---|-----------|----------------|
| N1–N3 | Jetson bring-up | SSH + NVMe root ✅; GR installed ✅ (apt 3.10.1.1, amended); PyTorch cu126 + CUDA gate ✅; RT kernel live with stock fallback ✅, cyclictest 69 µs worst-case under CUDA + memory load ✅ — **complete 2026-07-24** |
| P1–P3 | Pluto bring-up | Both enumerate (identity collision resolved); common firmware on both (v0.39 before baselines — amended) + ad9364 unlock; cabled A→B link with recorded baseline. *P1 partial: flat-sat unit enumerated, reports ad9364* |
| M1 | HAL + injection | All sensors behind daemons with validity words; shim can freeze/bias/kill any topic by command; Basilisk feeding fakes end-to-end |
| M2 | Minimal C&DH | F´ FlatSatCdh topology; time-tagged sequence executes; realtime + recorded telemetry streams; params tunable from GDS |
| M3 | Contact emulation | Radio exists only during scheduled windows; telemetry stores and dumps on contact; commands queue ground-side; system healthy through link outages |
| M4 | Closed-loop FDIR | Limit table + arbiter + Tier-1/2/3 ladder; three starter faults detected and recovered automatically; metrics dashboarded |
| M5 | Shadow ML | Statistical scorer live in shadow; precision/recall vs ground truth measured over ≥2 weeks of campaigns |
| M6 | Mode discipline | Safe-by-default boot; broadcast+ack transitions; flap protection demonstrated under repeated injection |
| C1 | Space-MLOps demo | New model trained from corpus, uplinked over RF via file transfer during a contact window, activated in B slot, regression-checked, rolled back on command |
| M7 (2027) | Fleet migration | JP7.2/Ubuntu 24.04 migration with A/B rollback, zero corpus-pipeline loss |

### Track A activities (avionics / FDIR / space-MLOps)

| ID | Activity | Pass criterion | Artifact |
|----|----------|----------------|----------|
| A1 | Consolidated RT avionics: mock 100 Hz–1 kHz ADCS loop pinned to isolated core under load | Holds period with < ~100 µs jitter under CPU/GPU stress | Cyclictest/jitter plot; consolidated-vs-federated note |
| A2 | Flight software: build F´ (and cFS) natively on the Jetson | FSW runs, emits telemetry, accepts commands | Running FSW; telemetry log |
| A3 | Networking: CSP/libcsp over virtual CAN (vcan) | `csp_ping` round-trip; telemetry over CSP | CSP-over-vcan demo |
| A4 | Basilisk HIL bridge: simSynch tick → ZMQ/JSON → FSW | Sim→FSW→actuator loop closes at 1 Hz, bounded jitter | Closed HIL loop; latency plot |
| A5 | Fault injection + resilience: kernel fault-injection, `strace -e inject`, A/B rootfs + overlayroot | Faults injected and logged; corrupt-rootfs run self-recovers | Labeled anomaly dataset + fault cookbook |
| A6 | Physical layer (adds STM32 judge + eFuse + PSU + CAN) | Judge cuts/restores the Jetson's rail cleanly; real CAN telemetry | Power-fault campaign; hardware watchdog demo |
| A7 | Onboard edge ML: real-time FDIR anomaly detector | Flags injected fault faster than a ground round-trip | Onboard-intelligence demo + writeup |

A1–A5, A7 need only the Jetson. A6 is the moment to buy the STM32/eFuse/PSU
cluster (~$300).

### Track B activities (adaptive ML radio)

| ID | Activity | Pass criterion | Artifact |
|----|----------|----------------|----------|
| B1 | Receive a real signal: NOAA APT / cubesat frame via gr-satellites (Pluto RX + antenna) | A decoded image/frame from space | Weather image / telemetry from orbit (postable) |
| B2 | Classical modem block by block; sweep SNR via attenuators over the real link | Clean decode; BER-vs-SNR curve | The baseline BER curve |
| B3 | Framing: CCSDS/AX.25 + Reed-Solomon via gr-satellites over the two-radio link | End-to-end framed packet decode both directions | Framed-link decode + BLER |
| B4 | ⭐ Learned receiver under interference: software jammer, sweep SIR; neural receiver vs matched-filter/LDPC baseline; train on jammers A,B / test on unseen C | Neural beats classical on BER-vs-SIR; graceful degradation on unseen interference | The money chart (NSF evidence) |
| B5 | Real-time modulation classifier (RadioML CNN) live on Jetson against received IQ | Live classification at usable rate | Real-time RF-ML classifier demo |
| B6 | Adaptive link: agent picks modulation/coding/power against a swept channel | Higher throughput than fixed baseline over the sweep | Adaptive-comms result |

B1 needs a Pluto + antenna. B2–B6 use both Plutos + passives + the Jetson.
Pretrain/derisk B4–B5 on public datasets (RadioML 2018.01A, MIT RF Challenge,
NVIDIA Sionna) before touching RF.

**Artifact trail (build-in-public):** satellite decode → BER baseline → framed
link → learned-vs-classical-under-interference chart → real-time classifier →
adaptive link · and in parallel · cyclictest/RT plot → running flight software
→ closed HIL loop → labeled fault dataset → reprogram-over-RF. Post each with
one honest paragraph of what it taught.

## 13. Open risks and decisions

RT-kernel maturity on Orin remains the top technical risk (Developer Preview
quality; mitigated by the fallback chain: relaxed loop rates → STM32 ownership
of hard-RT). The STM32 judge's absence until its milestone leaves the "Jetson
supervising itself" gap open — documented, not solved. The Zenoh-vs-ROS 2
question stays open until M1 forces it (Zenoh favored; rmw_zenoh preserves the
ROS option). Basilisk time management (§10) needs deciding at M1, not
discovering at M3. The F´-replacement trajectory (§5) is a standing judgment
call to revisit per-service — the criterion is always whether reimplementing
teaches more than it costs. **Added 2026-07-23:** GNU Radio version skew
(Jetson apt 3.10.1.1 vs whatever the Mac ground modem runs) must be resolved
before any cross-machine BER baseline — either pin the Mac to 3.10.1.1 or
upgrade both ends together.

## 14. Repository layout (target — architecture v2, migration in progress)

```
flatsat/                       # repo root
├── PLAN.md                    # this document — architecture + status + decisions
├── README.md                  # the end state, for humans arriving cold
├── docs/ARCHITECTURE.md       # package layout, contracts, config views, versioning tiers
├── flatsat/                   # THE package — the Tier-1 versionable artifact
│   ├── core/                  #   framework: bus, config, rt, health, registry (+inference later)
│   ├── hardware/              #   sensor.py + actuator.py contracts, drivers/, models/
│   ├── control/attitude/      #   controller contract + controllers/, estimators/
│   ├── control/health/        #   (future, M4) FDIR: rules, arbiter, responses, detectors/
│   ├── comms/                 #   (future, P3) modem contract, phy/, framing/, link
│   ├── mode/                  #   (future) system state machine — imports core only
│   ├── telemetry/             #   (future) recorder
│   ├── apps/                  #   thin generic executables: daemons + control loop
│   ├── msgs/                  #   generated protobuf bindings (committed)
│   └── sim/                   #   Basilisk bridge — the universe-fake, never on flight computer
├── config/                    # data, not code
│   ├── vehicles/*.toml        #   WHAT a spacecraft IS: composition + [body] + mounting (+[power] later)
│   └── devices/*.toml         #   device-intrinsic: datasheet envelopes + per-serial calibration
├── deployment.toml            # RT policy / core role / memory, per component
├── protos/                    # wire contracts: hal, adcs, mode, health (source of truth)
├── requirements/*.toml        # requirements + verification method + evidence
├── radio/                     # experiments bench — proven code GRADUATES into flatsat/comms/
├── tools/                     # gen-protos, gen-units, install-units, traceability,
│                              #   bus_bench, host-profiles/, n3-rt-kernel-runbook.md
├── cdh/                       # (future, M2) F´ deployment — own build system, bus bridge
└── cpp/                       # (future) created by the first measured port trigger, not before
```

Tests colocate with the code they test: one `<name>_test.py` beside every
module, 1:1 — no separate tests/ tree.
