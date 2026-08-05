# Demo cheatsheet — commands only

Narrative, talk tracks, and break-glass table: `docs/demo-runbook-2026-08-06.md`.

Six terminals. A, C, F on the Mac at `~/flatsat`; B, D, E are ssh
sessions on the Jetson (`ssh trevor@100.65.0.120`), logged in once.
Plus the browser (live console) and Vizard.

## 0 — Pre-demo reset (terminal D, once)

Clear stale test artifacts from the uplink store (old ml_policy@v1/v2
from July testing would clutter the ACTIVE line on stage) and confirm
the services:
```bash
rm -rf ~/flatsat-uplink/staging/ml_policy@* ~/flatsat-uplink/slots/ml_policy* && sudo systemctl restart flatsat-uplink && systemctl is-active flatsat-router flatsat-uplink flatsat-adcs
```

## Setup (once)

**A (Mac — bridge):**
```bash
cd ~/flatsat
```

**C (Mac — ground commands):**
```bash
cd ~/flatsat && export FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447
```

**B (Jetson — events feed).** The adcs status wall drowns the story;
this keeps the one-line rates and every fdir/mode/uplink verdict:
```bash
journalctl -u flatsat-adcs -u flatsat-uplink -u flatsat-fdir -u flatsat-mode -f | grep --line-buffered -E "\[fdir\]|\[mode\]|\[uplink\]|\[adcs\] \|omega\||controller:"
```

**D (Jetson — sudo):** just be logged in.

**E (Jetson — flight side of the space link).** Ground tools talk to
the `ground/` namespace; the link pair is the ONLY bridge to the
flight bus — allowlisted, framed (CCSDS), queued, gated by a 10 s
contact window every 30 s, and carried over REAL RF: 915 MHz GMSK
between the two Plutos through the 30 dB pads. Log in and run:
```bash
cd ~/flatsat && ~/venvs/flatsat-ml/bin/python -m flatsat.apps.link_service --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb
```

**F (Mac — ground station, second Mac terminal).** Owns the ground
radio (`usb:0.3.5`, the Pluto on the Mac):
```bash
cd ~/flatsat && FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447 PYTHONPATH=$PWD ~/venvs/gr-ground/bin/python -m flatsat.apps.link_service --ground --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb
```

FALLBACK (no radios, same story minus RF): skip E and F above, run the
loopback harness on the Jetson instead — every other command is
identical:
```bash
cd ~/flatsat && ~/venvs/flatsat-ml/bin/python tools/bench_link.py --vehicle config/vehicles/flatsat_v1_mldemo.txtpb
```
(If `flatsat-link.service` ever gets installed by a units reinstall,
turn it off: `sudo systemctl disable --now flatsat-link`.)

## 1 — Release, detumble on fallback PD

D:
```bash
sudo systemctl restart flatsat.target
```
A (immediately after — clock sync):
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.05,-0.04,0.03 --viz
```
**Then Vizard** (the bridge BINDS the socket; Vizard connects to it, so
the sim must already be running): Direct Communication →
`tcp://localhost:5556` → Start Visualization.

B shows: `controller: ml_policy … NO ACTIVE VERSION — fallback PD`, |omega| 70 → <5 mrad/s.
Live console: drag `downlink/imu.omega_mrad_s` onto a plot — the
detumble curve draws itself, one point per downlinked sample.

## 2 — Blackout → safing (WATCH B — it prints ONCE, ~10 s after the kill)

A: **Ctrl-C**.
B shows: `[fdir] tripped: imu_stale — requesting SAFE` then
`[mode] SYSTEM_MODE_NOMINAL -> SYSTEM_MODE_SAFE seq=N (fdir: imu_stale)`.

Then restart the bridge — A:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.05,-0.04,0.03 --viz
```
(Vehicle STAYS SAFE — no self-recovery. A second blackout while SAFE
prints nothing new: safing is edge-triggered.)

## 3 — Uplink + the SAFE refusal

C:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground send ml_detumble 2026-08-05a build/ml_detumble/2026-08-05a.json --kind model
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground activate ml_detumble 2026-08-05a --ground
```
The commands QUEUE at the ground station; terminal E shows the pass
open (`contact OPEN` → `delivered to FLIGHT`, up to 30 s wait — that
wait IS the space-link story). Then B shows `[uplink] staged …` and
`[uplink] REFUSED: activation refused in SAFE: survival law is not swappable`.

## 4 — Recover, deliberately

C:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request RECOVERY --ground --reason "bridge restored" --topic-prefix ground
```
wait for B to show `[mode] SYSTEM_MODE_SAFE -> SYSTEM_MODE_RECOVERY` (next pass), then:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request NOMINAL --ground --reason "checkout complete" --topic-prefix ground
```
Commands cross at contact windows — B shows each `[mode] … -> …`
transition as it lands.

## 5 — Activate for real, adopt by restart

C:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground activate ml_detumble 2026-08-05a --ground
```
(wait for E's next pass + B's `[uplink] activated …`) then D:
```bash
sudo systemctl restart flatsat-adcs
```
B shows: `[uplink] activated ml_detumble@2026-08-05a`, then
`controller: ml_policy … version=2026-08-05a hidden=32 … sha256=56974ca8ec34`.

## 6 — Fresh tumble, network flying

D:
```bash
sudo systemctl restart flatsat.target
```
A (immediately after):
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.06,0.05,-0.04 --viz
```
Slot pointer survives the restart — B still shows `version=2026-08-05a`.
Settles ~0.5 mrad/s (PD floor was 2–4).

## 7 — Rollback

C:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground rollback ml_detumble
```
(wait for the pass) then D:
```bash
sudo systemctl restart flatsat-adcs
```
B: back to `NO ACTIVE VERSION — fallback PD`.
(If rollback says `REFUSED: no previous version`, the fallback line
after restart is the same story — only one version was ever active.)

## Live web console (mission control's screen)

Viewer dev server (if not already running):
```bash
npm --prefix ~/flatsat/web run dev
```
Bridge on the Mac (third Mac terminal):
```bash
cd ~/flatsat && FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447 ~/venvs/flatsat-ground/bin/python tools/ground_bridge.py
```
Then open:
```
http://localhost:5173/?api=http://localhost:8600
```
Everything it shows CROSSED THE RADIO: the `● LIVE · downlink Ns ago`
badge is the age of the newest downlinked sample (green = pass just
landed, amber = between passes, red = link quiet); contact passes are
timeline spans; imu/wheel/mode/fdir/uplink are draggable channels;
transitions and staging land in the events lane. Updates every 3 s;
docked plots survive. Terminal alternative: `tools/ground_console.py`.

Demo moments to point at: `omega_mrad_s` falling during detumble;
`staged:` then `ACTIVE: ml_detumble@2026-08-05a` appearing in events
after step 5 — each having crossed the radio on a pass.

**COMMANDING panel** (bottom of the live view): every C-terminal verb
is also a button — mode requests (reason field + RECOVERY / NOMINAL /
SAFE), artifact upload / activate (sha-confirm dialog) / rollback, and
a ground-side "clear view" for demo resets. Buttons POST to the ground
station and cross at the next pass, identical to the CLI; every click
lands in the events lane as a mission-control audit entry. The CLI
commands in §3-§7 remain valid — use whichever reads better on stage.

## Sim segment (viewer)

Load `~/hil-trace/campaign_pd_vs_ml.json` → timeline spans → drag
`sim/ic180mrad/pd.omega_mrad_s` + `sim/ic180mrad/ml.omega_mrad_s` onto
one plot. Then load yesterday's nadir export for the orbit view.

## Mode / state checks any time

C (direct query — bench convenience, say so if narrating):
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request --status
```
