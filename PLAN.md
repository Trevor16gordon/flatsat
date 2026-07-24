# Flat-Sat Software Architecture & Development Plan

v1.1 — July 2026 · Trevor · single merged plan (architecture + working status)

**Status: Jetson bring-up COMPLETE (N1–N3).** N3 PREEMPT_RT live, gate passed
untuned (69 µs worst-case under load) · P1 partially complete (flat-sat Pluto
verified TX+RX+data through pad loopback; firmware update deferred) · repo +
quality gates live · next: ground Pluto unboxing / M1 seam work / P3.

---

## 0. Working status and decision log

### Done (as of 2026-07-24)

- **M1 foundation layer (2026-07-24).** `protos/` established as the single
  source of truth for inter-service interfaces (decision: protobuf schemas;
  Python codegen + mypy stubs committed via `tools/gen-protos.sh`; generated
  files gate-exempt, consumers fully checked via stubs; C++ codegen deferred
  to first consumer ~M2; pydantic deferred to config edges). `hal.proto`:
  Header (§4 contract: timestamps, monotonic seq, validity word — semantics
  sharpened after review: acquisition facts only, operational normality is
  FDIR L1's job) + ImuSample + TemperatureSample. `mode.proto`: SystemMode +
  latched ModeState. Zenoh 1.9.0 validated on-device: pub/sub with protobuf
  payloads AND the §7 latched-mode query pattern (late joiner learns mode
  immediately) — first running evidence for the §13 Zenoh-vs-ROS2 call.
  `SensorDaemon` base implements contract mechanics once (header stamping,
  seq, validity propagation, drift-free absolute-deadline cadence); adding a
  sensor = SensorConfig + a read() that must not raise. **First real HAL
  daemon live: `flight/hal/jetson_thermal.py`** — tj-thermal at ~48 °C over
  the bus, sample→publish ~400 µs; power-gated cv* zones publish COMM-flagged
  on cadence (flag-and-forward proven against real EAGAIN). War story
  recorded in code: sysfs EAGAIN evades OSError via Python text IO
  (TypeError) and buffered binary IO (silent None) — raw os.read is the
  honest path. 8 integration/unit tests green on-device.

- **N3 COMPLETE (2026-07-24) — PREEMPT_RT kernel live, gate passed.**
  Installed via NVIDIA's rt-kernel OTA repo (`5.15.148-rt-tegra`, build
  36.4.0-20241014 against BSP r36.4.7). Boot-path discovery confirmed on
  hardware: bootloader reads the SD APP partition's extlinux.conf + kernel
  (with `root=/dev/nvme0n1p1`), NOT the NVMe's — apt installs required
  manual sync to SD (runbook stages 3–4). Postinst trap confirmed and fixed:
  its generated entry wrote `root=/dev/mmcblk0p1` (would have booted the
  stale SD rootfs). Fallback preserved: stock `primary` entry + its own
  initrd untouched on SD; RT entry has separate Image.real-time +
  initrd.real-time; rescue = serial console boot menu or SD-card edit on
  the Mac. Verified under RT: `/sys/kernel/realtime`=1, CONFIG_PREEMPT_RT=y,
  nvgpu loaded, zero failed units, full N2b CUDA gate re-passed.
  **Acceptance gate: cyclictest 10 min, SMP, prio 95, under sustained CUDA
  4096² matmul + stress-ng (4 cpu + 2 vm×1GB): worst-case 69 µs (per-core
  max 69/39/45/56/48/48, avg 5–10 µs) — beats the <~100 µs target with NO
  isolation tuning.** This is the A1 "before" baseline; isolcpus/nohz_full/
  IRQ affinity remain A1 work. Install captured idempotently in
  jetson-setup.sh (boot sync deliberately manual, script warns + points at
  `tools/n3-rt-kernel-runbook.md`).

- **N1 complete.** JetPack 6.2.1 flashed to microSD (board firmware was already
  36.4.3 — no JP5.1.3 detour needed). Headless oem-config over USB-C serial,
  WiFi via `nmcli` (installer WiFi tool fails — skip via Ethernet-DHCP-timeout
  trick), `avahi-daemon` → `ssh trevor@jetson.local`, jtop. Hostname: `jetson`.
- **Storage.** 500 GB NVMe (Fanxiang S690Q) in M.2; rootfs migrated on-device
  (GPT + ext4 → rsync → `root=/dev/nvme0n1p1` in extlinux.conf). SD retained as
  rescue boot (flip `root=` back to `mmcblk0p1` to recover).
- **Reproducibility.** `tools/jetson-setup.sh` — idempotent bring-up (state
  checks skip completed work); dpkg/pip/kernel manifests dumped to
  `tools/setup-manifests/` and committed. Full reset ≈ 1 hr reflash.
- **N2, radio half.** GNU Radio 3.10.1.1 + libiio 0.23 + libad9361 via apt;
  gr-iio `fmcomms2` Pluto blocks verified importable.
- **Repo + GitHub.** `Trevor16gordon/flatsat` (private), gh CLI authenticated,
  git+gh install captured in setup script.
- **Quality gates.** ruff (ANN: all function inputs/outputs typed; D:
  docstrings mandatory) + mypy (`disallow_untyped_defs`) + pre-commit hooks +
  CI re-run. Untyped or undocumented Python cannot be committed.
- **P1 partial.** Flat-sat Pluto enumerates over USB (`usb:1.4.5`) and IP
  (`192.168.2.1` / `pluto.local`); fw v0.37; driver already reports
  `ad9361-phy,model: ad9364` (extended-range unlock effectively present).
  Physical state: TX→30 dB pads→RX looped on the flat-sat unit, on the Jetson.
- **RX smoke test PASSED (2026-07-23).** `radio/pluto_smoke_test.py` (RX-only,
  never transmits) run with Trevor's go-ahead after the constructor was
  rewritten to the verified in-tree gr-iio 3.10 API (introspected on-device:
  `fmcomms2_source_fc32(uri, ch_en, buffer_size)` + setters; no bandwidth
  setter — RF filter follows sample rate). Result at 915 MHz / 2.084 MSa/s /
  40 dB gain, URI auto-detected `usb:1.4.5`: 262,144 IQ samples captured;
  mean power −65.7 dBFS, peak |amp| 0.002 (no saturation), DC offset ~0
  (rfdc/bbdc working), dominant FFT bin at DC (residual LO leakage — normal
  for zero-IF with no signal present). Clean noise floor, no dominant tone
  ⇒ expected result with TX silent. Receive path confirmed alive. Benign
  first-run `vmcircbuf` factory warnings from GNU Radio; ignorable.
- **TX loopback tone test PASSED (2026-07-23)** — first RF transmission of
  the project, run with Trevor's per-instance go-ahead, 30 dB pads confirmed
  inline. `radio/pluto_tx_loopback_test.py` (gated on `--transmit`; TX
  attenuation forced to 89.75 dB on every exit path): cos tone at LO+100 kHz,
  amp 0.5, 20 dB TX atten, 30 dB RX gain, 915 MHz, 2.084 MSa/s. Result:
  dominant bin +100.00 kHz (offset error +1 Hz), tone 65.9 dB above median
  floor, mean power −52.3 dBFS (vs −65.7 RX-only baseline), peak |amp| 0.0044
  — no saturation. TX and RX paths both verified end-to-end through the pads.
  Note: received level ran well below the survey link budget (≈−49 dBm
  predicted at RX) — enormous margins either way; calibrate absolute levels
  only if/when a baseline needs them. Benign TX underrun chars (`UUU`) at
  flowgraph teardown.
- **Data loopback PASSED (2026-07-23) — first real data over RF.**
  `radio/pluto_data_loopback_test.py` (GMSK via stock gmsk_mod/gmsk_demod,
  framed 64-bit access code + 64-byte payload, 260.5 kbaud at LO+250 kHz):
  113/113 frames bit-perfect, payload BER 0.00 over 57,856 bits, message
  recovered verbatim. Run 1 failed (BER 1.8e-1) with the signal centered at
  baseband DC — on a shared-LO zero-IF loopback the RX LO leakage +
  AD936x DC-correction loops sit mid-signal. Lesson recorded for all future
  waveforms on this hardware: **keep signal energy off DC** (digital offset
  tune: rotate up on TX, xlating-LPF down on RX; envelope unchanged).
  Diagnosis method that worked: identical chain back-to-back in software
  (BER 0) exonerated the modem before touching RF again.
- **SNR sweep PASSED (2026-07-23) — first BER-vs-SNR waterfall.**
  `radio/pluto_snr_sweep_test.py`: one TX capture (same GMSK link), then
  calibrated AWGN added offline in 15 stages, each re-demodulated. Real
  capture: clean ≥30 dB; BER 5.2e-5 at 20 dB; 2.4e-3 at 15 dB; 1.9e-2 at
  12 dB; frame sync collapses below ~5 dB; dead at 4 dB. Synthetic (pure
  software) capture ran ~2 dB better at the same SNRs — the measured
  implementation penalty of real hardware (phase noise, residual DC, capture
  noise floor). Capture archived at
  `~/flatsat-captures/loopback_gmsk_915MHz_20260723.npz` — re-sweep any
  ladder offline via `--load`, no RF. This is the B2 methodology in miniature
  (there: real attenuator sweeps + two radios; the offline-AWGN trick stays
  useful for controlled comparisons).
- **N2b complete (2026-07-23).** PyTorch 2.8.0 (CUDA 12.6 build) +
  torchvision 0.23.0 in `~/venvs/flatsat-ml` (`--system-site-packages`),
  wheels from `pypi.jetson-ai-lab.io/jp6/cu126` (the `.dev` mirror is dead)
  downloaded `--no-deps` and installed from local files — pypi.org ships a
  CPU-only torch under the identical version string, so index mixing is
  never allowed to resolve torch. numpy held at 1.21.5 (<2) for the system
  gnuradio bindings. Acceptance gate `tools/verify_torch_cuda.py` PASSED:
  CUDA build, Orin GPU (compute 8.7), GPU matmul matches CPU, gnuradio
  3.10.1.1 imports alongside torch in one process (B5 prerequisite).
  Implemented idempotently in `jetson-setup.sh`; venv captured in manifests.
- **Repo restructured + plans merged** (this document) — pushed to GitHub.

### Decision log

| Date | Decision | Rationale / consequence |
|------|----------|-------------------------|
| 2026-07-23 | **No conda.** GNU Radio self-managed; stay on apt 3.10.1.1 for now (deviates from the radioconda 3.10.11+ pin in v1.0 §3). | Simpler env story (system python + venvs only). Open risk: Mac ground modem must version-match before any cross-machine BER baseline; revisit before B2. |
| 2026-07-23 | **Pluto firmware stays v0.37** for now; v0.39 upgrade deferred to just before comparative baselines. | Unit is modern + already reports ad9364; flashing is the riskiest Pluto op — defer until it buys something. P1 exit amended accordingly. |
| 2026-07-23 | **Serial→role assignment.** `104473b04a06000602001c00dd1f84cfaa` = **flat-sat radio** (on the Jetson). Second unit = **ground radio** (eventually on the Mac). | Recorded per §2; ground segment on Mac is future work — both radios live on the Jetson until the ground segment stands up. |
| 2026-07-23 | **Repo layout by architecture segment** (`flight/`, `ground/`, `radio/`, `tools/`); plans merged into this single PLAN.md. | Mirrors the two-segment architecture from day one. |
| 2026-07-24 | **ADCS language: Python first, C++ only when measurement demands it.** Port trigger is a number, not a feeling: worst-case loop lateness under adversarial load exceeding ~5% of the period, or a rate target beyond Python's floor (~1 kHz / sub-100 µs). | 100–200 Hz has ms-scale budget; disciplined Python (prealloc, gc frozen, mlockall, FIFO) holds ~100–300 µs jitter on the RT kernel. The bus-topic seam makes the later port invisible to the rest of the system; hard-RT tier belongs to the STM32 judge anyway. Jitter harness runs from day one so the before/after is free. |

### Next up

1. Unbox ground Pluto (Trevor, ~10 min): record serial into §2 role table;
   confront the two-Pluto identity collision (both default to
   usb 192.168.2.1) before any two-radio work.
2. M1 seam work (no hardware needed): Zenoh evaluation (pub/sub + latched
   `sys/mode` smoke test) and the HAL daemon message contract as typed code
   (sample/publish timestamps, validity word, sequence number) + one
   fake-sensor daemon.
3. P3 proper: BPSK/framing over the loopback, then two-radio link once the
   ground Pluto is enumerated and firmware aligned (v0.39 both — decision
   log). Zero-IF lesson applies: keep signal energy off DC.
4. A1 (later): core isolation (isolcpus/nohz_full/IRQ affinity) + re-run
   the cyclictest gate against the 69 µs untuned baseline.

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

## 14. Repository layout

```
flatsat/
├── PLAN.md                  # this document — architecture + status + decisions
├── README.md
├── pyproject.toml           # ruff + mypy quality contract
├── .pre-commit-config.yaml  # commit-time enforcement
├── .github/workflows/       # CI re-running the same hooks
├── tools/                   # bring-up + host tooling
│   ├── jetson-setup.sh      #   idempotent Jetson bring-up
│   └── setup-manifests/     #   versioned known-good system snapshots
├── radio/                   # shared PHY: flowgraphs, smoke tests, modem work
├── flight/                  # onboard segment: HAL daemons, services, link svc
└── ground/                  # ground segment (Mac): modem, GDS, mission ctl
```
