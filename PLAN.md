# Project Plan — Flat-Sat + Adaptive ML Radio

Working plan capturing the hardware decisions and two-track structure settled in
planning. Two independent but hardware-sharing tracks run off one compute node
and two radios.

**Status — updated 2026-07-23**

## Done

- **N1 nearly complete.** JetPack 6.2.1 flashed to microSD (board firmware was
  already 36.4.3, so no JetPack 5.1.3 firmware detour was needed). Headless
  oem-config over USB-C serial (`screen /dev/cu.usbmodem* 115200`), WiFi via
  `nmcli` post-install (installer's WiFi tool failed — skip it via the
  Ethernet-DHCP-timeout trick), `avahi-daemon` → `ssh trevor@jetson.local`, jtop
  installed. Hostname: `jetson`.
- **Storage upgrade complete.** 500 GB NVMe (Fanxiang S690Q) installed in the M.2
  slot; rootfs migrated on-device (parted GPT + single ext4 partition → rsync of
  `/` → `root=/dev/nvme0n1p1` in `/boot/extlinux/extlinux.conf`). `/` now on NVMe;
  SD card retained as rescue boot (flip `root=` back to `mmcblk0p1` to recover).
- **Reproducibility.** `jetson-setup.sh` — idempotent bring-up script (apt
  updates, WiFi via env vars, jtop + poetry pinned, pipx, dpkg/pip/kernel
  manifest dumps to `setup-manifests/`). Full reset path: reflash SD ≈ 1 hr,
  mostly unattended. Convention: system Python untouched; project venvs;
  TensorRT-needing envs use `--system-site-packages`.
- **N2, radio half.** GNU Radio 3.10 + libiio/libad9361 installed via the script.
- **RF chain assembled.** 3× 10 dB SMA pads on hand (not the 30+10 originally
  listed). First test: single-Pluto loopback (TX → 3 pads → RX on one unit), then
  graduate to the two-radio A→B link. **Never TX→RX without pads.**

## Next session (Claude Code)

- Finish git: `~/flatsat` repo, Jetson commits directly over SSH key; script +
  manifests + this plan go in.
- **P1** — plug in Pluto #1, `iio_info -s` enumerates it; firmware update.
- **P2** — `fw_setenv` compatible `ad9364` unlock on both units.
- **P3** — loopback tone/BPSK in GNU Radio, then the two-radio cabled link.
- **N2b** — PyTorch from NVIDIA's JP6/CUDA-12.6 Jetson wheel index (not PyPI
  torch), TensorRT via `nvidia-jetpack`; pass: `torch.cuda.is_available()` →
  `True`. Add to script.
- **N3** — PREEMPT_RT kernel packages, `isolcpus` in `extlinux.conf` (keep stock
  kernel entry as fallback), `cyclictest` gate.

## Decisions & hardware on hand

**Compute** — NVIDIA Jetson Orin Nano Super Developer Kit (chosen over the AMD
Kria KV260). Rationale: the work's center of gravity moved from flat-sat realism
toward learned/ML-native radio, where the KV260's DPU (CNN-only, frozen at Vitis
AI 3.5, no transformers) was the weakest flank and the top project risk. The
Jetson runs native PyTorch/TensorRT, hosts SDRs at higher throughput, has an
installable PREEMPT_RT kernel, native IMX219 camera support, and is cheaper
($249). What was forfeited — the FPGA fabric and an on-die RTOS island — is either
not needed by the current thesis (FPGA) or relocated to the independent STM32
judge (hard-real-time island, cleaner fault separation).

**Radios** — 2× borrowed ADALM-Pluto SDR. Two radios (not one) means a genuine
bidirectional link — one as "ground," one as "satellite" — through a cabled,
attenuated path, rather than self-loopback. Each Pluto is USB-2-limited to
~4–8 MS/s sustained, which is ideal: the narrowband, D2D-relevant waveforms
(GSM ~200 kHz, NB-IoT 180 kHz) fit comfortably and match the interference/D2D
thesis.

**RF passives** — borrowed: 3× 10 dB SMA pads (30 dB total), short SMA M-M
cables, F-F barrels, 50 Ω terminators. (Cabled only; nothing transmits over the
air.)

**Storage** — 64 GB microSD (boot/rescue) + 500 GB NVMe (root, installed).
Rootfs migrated to NVMe 2026-07-23; SD kept as rescue boot.

**Flat-sat hardware, added when Track A reaches physical faults (~later):**
STM32H7 Nucleo-H723ZG judge, TI TPS259824OEVM eFuse (LCL), Korad KA3005P PSU, CAN
transceivers + USB-CAN adapters, Qwiic sensor chain.

## The two tracks

- **Track A — Flat-Sat / avionics / FDIR / space-MLOps.** Runs on the Jetson
  alone at first; adds the STM32/power cluster later.
- **Track B — Adaptive ML radio.** Runs on the Jetson + two Plutos + RF passives.

They converge at one milestone (reprogram/redeploy an ML model over the RF link),
which is why both live in one plan.

## Software bring-up milestones (do first — gates everything)

### Jetson Orin Nano

- **N1 — First light.** Flash JetPack to microSD; headless setup over USB-C; SSH
  in over Ethernet. Pass: `ssh` shell, `nvidia-smi`/jtop shows the GPU.
- **N2 — Toolchain.** Install GNU Radio + PyTorch + TensorRT; confirm GPU
  inference runs. Pass: a trivial CUDA/PyTorch tensor op on GPU;
  `gnuradio-companion` launches.
- **N3 — Real-time kernel.** Install the JetPack PREEMPT_RT kernel; `isolcpus` +
  SCHED_FIFO; run `cyclictest` under load. Pass: `uname -a` shows PREEMPT_RT;
  worst-case latency < ~100 µs on an isolated core.

### Pluto (×2)

- **P1 — Recognized.** Both Plutos enumerate on the host (USB network devices,
  distinct IPs); update firmware. Pass: `iio_info` sees each; both visible
  simultaneously.
- **P2 — Unlocked.** Apply `fw_setenv compatible ad9364` on each (70 MHz–6 GHz,
  56 MHz BW). Pass: each reports the extended tuning range.
- **P3 — Two-radio cabled link.** Pluto A TX → 30 dB pad → cable → 10 dB → Pluto B
  RX; a tone/BPSK from A is visible on B in GNU Radio. Pass: clean received
  constellation/spectrum at a controlled power level.

> Do the risky first-boot/firmware steps at home before travel — a QSPI-firmware
> update may be needed before JetPack boots, and that's better hit with a full
> setup than in a hotel.

## Track A — Flat-Sat / Avionics / FDIR / Space-MLOps

Objective: a consolidated, new-space-style avionics node on the Jetson (RT Linux
+ flight software + edge ML), an independent hardware watchdog, and — the research
payload — a fault-injection harness that produces a labeled anomaly corpus and
demonstrates recover/reprogram-over-the-link. Explores the "space-MLOps" product
hypothesis hands-on. Scope note (July 2026): start with a basic FDIR
detection/response pipeline, not the full three-tier SBIR stack.

| ID | Activity | Pass criterion | Artifact |
|----|----------|----------------|----------|
| A1 | Consolidated RT avionics: pin a mock 100 Hz–1 kHz ADCS loop to an isolated core under load | Loop holds period with < ~100 µs jitter under CPU/GPU stress | Cyclictest/jitter plot; "consolidated vs federated" note |
| A2 | Flight software: build F´ (and cFS) natively on the Jetson from the reference deployments | FSW runs, emits telemetry, accepts commands | Running FSW; telemetry log |
| A3 | Networking: CSP/libcsp over virtual CAN (vcan) — no hardware yet | `csp_ping` round-trip; telemetry over CSP | CSP-over-vcan demo |
| A4 | Basilisk HIL bridge: simSynch wall-clock tick → ZMQ/JSON → FSW with simulated sensors/actuators | Sim→FSW→actuator loop closes at 1 Hz, bounded jitter | Closed HIL loop; latency plot |
| A5 | Fault injection + resilience: kernel fault-injection, `strace -e inject`, A/B rootfs + overlayroot | Deliberate faults injected and logged; corrupt-rootfs run self-recovers | Labeled anomaly dataset + "fault cookbook" |
| A6 | Physical layer (adds STM32 judge + eFuse + PSU + CAN): power-path LCL, real CAN bus, sensor hub | Judge cuts/restores the Jetson's rail cleanly; real CAN telemetry | Power-fault campaign; hardware watchdog demo |
| A7 | Onboard edge ML: run an anomaly detector on the Jetson making a real-time FDIR call | Detector flags an injected fault faster than a ground round-trip | Onboard-intelligence demo + decision writeup |

Note: A1–A5 and A7 need only the Jetson. A6 is the moment to buy the
STM32/eFuse/PSU cluster (~$300).

## Track B — Adaptive ML Radio

Objective: build the classical comms chain hands-on, then replace rigid blocks
with learned ones, with the flagship result being a learned receiver that beats
classical under interference — the intuition-builder, the NSF-pitch evidence, and
the headline artifact. Two Plutos make it a real link.

| ID | Activity | Pass criterion | Artifact |
|----|----------|----------------|----------|
| B1 | Receive a real signal: decode a live satellite downlink (NOAA APT image or cubesat frame via gr-satellites) using a Pluto + antenna | A decoded image/frame from space | Weather image / telemetry from orbit (postable) |
| B2 | Classical modem, block by block: bits→FEC→symbols→pulse→link (Pluto A→pad→Pluto B)→sync→demod→decode; sweep SNR via attenuators | Clean decode; BER-vs-SNR curve measured over the real link | The baseline BER curve |
| B3 | Framing: CCSDS/AX.25 + Reed-Solomon via gr-satellites over the two-radio link | End-to-end framed packet decode both directions | Framed-link decode + BLER |
| B4 | ⭐ Learned receiver under interference: inject a software jammer/co-channel signal, sweep SIR; train a neural receiver on the Jetson; compare to matched-filter/LDPC baseline; then train on jammers A,B / test on unseen C | Neural beats classical on BER-vs-SIR; graceful degradation on unseen interference | The money chart (NSF evidence + credibility post + deck slide) |
| B5 | Real-time modulation classifier on the edge GPU: train a CNN (RadioML), run live on the Jetson against received IQ | Live modulation classification at usable rate | Real-time RF-ML classifier demo |
| B6 | Adaptive link: agent observes channel/SIR and picks modulation/coding/power against a swept channel (variable attenuator) | Higher throughput than a fixed baseline over the sweep | Adaptive-comms result |

Note: B1 needs a Pluto (RX) + antenna. B2–B6 use both Plutos + RF passives + the
Jetson. Pretrain/derisk B4–B5 with public datasets (RadioML 2018.01A, MIT RF
Challenge, NVIDIA Sionna) before touching RF.

## Convergence, sequencing, and artifacts

**The bridge milestone (both tracks):**

- **C1 — Reprogram/redeploy over the link.** Push an ML model or config update to
  the Jetson "satellite" over the Pluto RF link (CCSDS TC + a CFDP-style
  transfer), with A/B rollback. Pass: a model update lands and runs over RF; a bad
  update rolls back. Artifact: "I updated my flat-sat's ML model over the radio,
  with rollback" — the space-MLOps hypothesis, demonstrated. Needs Track A (A5 A/B
  boot) + Track B (B3 link).

**Hardware timeline:** Jetson + 2 Plutos + RF passives cover bring-up, all of
Track B, and Track A through A5/A7. Only A6 (and the full C1 fidelity) needs the
STM32/eFuse/PSU cluster — buy it when Track A reaches physical power faults.

**Artifact trail (build-in-public):** satellite decode → BER baseline → framed
link → learned-vs-classical-under-interference chart → real-time classifier →
adaptive link · and in parallel · cyclictest/RT plot → running flight software →
closed HIL loop → labeled fault dataset → reprogram-over-RF. Post each with one
honest paragraph of what it taught you — the demos build the intuition, the posts
generate the inbound conversations.
