# Demo cheatsheet — commands only

Narrative, talk tracks, break-glass table: `docs/demo-runbook-2026-08-06.md`.

**Three terminals + the browser + Vizard.**
- **A** — Mac, `~/flatsat`: the Basilisk bridge (the universe)
- **B** — Jetson ssh: the flight journal (the spacecraft's own voice)
- **C** — Jetson ssh: sudo actions (restarts)
- **Browser** — mission control: telemetry, passes, commanding, audit
- **Vizard** — the 3D vehicle

Everything else (link services, ground bridge, viewer server) runs
backgrounded from the setup block and stays out of sight. Commanding
happens in the browser; the CLI equivalents live in the appendix as
fallbacks.

## 0 — Pre-demo reset (terminal C, once)

Clear stale test artifacts and confirm services:
```bash
rm -rf ~/flatsat-uplink/staging/ml_policy@* ~/flatsat-uplink/slots/ml_policy* && sudo systemctl restart flatsat-uplink && systemctl is-active flatsat-router flatsat-uplink flatsat-adcs
```

## Setup (once)

**Terminal C (Jetson)** — start the flight side of the space link,
backgrounded. IDEMPOTENT: kills any prior instance first — duplicate
link services double-deliver every sample and scribble the plots:
```bash
pkill -f "apps[.]link_service"; sleep 1; cd ~/flatsat && setsid ~/venvs/flatsat-ml/bin/python -m flatsat.apps.link_service --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb > /tmp/link_flight.log 2>&1 < /dev/null &
```

**Any Mac terminal** — ground station stack, backgrounded (also
idempotent — kills prior instances first):
```bash
pkill -f "link_service --ground"; pkill -f "ground_bridge"; sleep 1; cd ~/flatsat && export FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447 && PYTHONPATH=$PWD nohup ~/venvs/gr-ground/bin/python -m flatsat.apps.link_service --ground --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb > /tmp/link_ground.log 2>&1 & nohup ~/venvs/flatsat-ground/bin/python tools/ground_bridge.py > /tmp/ground_bridge.log 2>&1 & (curl -s -o /dev/null http://localhost:5173 || nohup npm --prefix web run dev > /tmp/viewer.log 2>&1 &) ; sleep 3 && echo "ground station up"
```

**Browser** — mission control:
```
http://localhost:5173/?api=http://localhost:8600
```
LIVE badge = age of newest downlinked sample (green: pass just landed,
amber: between passes, red: link quiet). COMMANDING panel at the
bottom: every command queues at the ground station and crosses at the
next pass; every click is audited in the events lane.

**Terminal B (Jetson)** — flight journal:
```bash
journalctl -u flatsat-adcs -u flatsat-uplink -u flatsat-fdir -u flatsat-mode -f | grep --line-buffered -E "\[fdir\]|\[mode\]|\[uplink\]|\[adcs\] \|omega\||controller:"
```

## 1 — Release, detumble on fallback PD

A — bridge FIRST (truth is flowing before the stack wakes, so FDIR
never sees a stale window and the vehicle comes up NOMINAL; the
detumble demo vehicle has no onboard-orbit dependency, so this order
is safe here — nadir-pointing runs pair the restarts the other way):
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.05,-0.04,0.03 --viz
```
C — then release the vehicle:
```bash
sudo systemctl restart flatsat.target
```
**Then Vizard** (the bridge binds the socket; Vizard is the client):
Direct Communication → `tcp://localhost:5556` → Start Visualization.

B: `controller: ml_policy … NO ACTIVE VERSION — fallback PD`.
Browser: drag `downlink/imu.omega_mrad_s` onto a plot — the detumble
curve draws itself from samples that each rode a pass.

## 2 — Blackout → safing (watch B: prints ONCE, ~10 s after the kill)

A: **Ctrl-C**.
B: `[fdir] tripped: imu_stale — requesting SAFE` →
`[mode] … -> SYSTEM_MODE_SAFE`. Browser: mode chip flips to SAFE at
the next pass.

Restart the bridge — A (same command as step 1). Vehicle STAYS SAFE:
no self-recovery.

## 3 — Upload + the SAFE refusal (browser)

COMMANDING → DEPLOY: **upload…** → pick
`build/ml_detumble/2026-08-05a.json`, version tag `2026-08-05a`.
Wait a pass: B shows `[uplink] staged ml_detumble@2026-08-05a`.

Then select it in the dropdown → **activate** → confirm.
B shows: `[uplink] REFUSED: activation refused in SAFE: survival law
is not swappable`. (The refusal is the system declining power — say so.)

## 4 — Recover, deliberately (browser)

MODE row: reason `bridge restored` → **→ RECOVERY**. Wait for the pass
(B: `SAFE -> RECOVERY`). Then reason `checkout complete` → **→ NOMINAL**.

## 5 — Activate for real (browser), adopt by restart (C)

DEPLOY: select `ml_detumble@2026-08-05a` → **activate** → confirm.
Wait for `[uplink] activated …` in B, then C:
```bash
sudo systemctl restart flatsat-adcs
```
B: `controller: ml_policy … version=2026-08-05a hidden=32 …
sha256=56974ca8ec34`. Browser events: `ACTIVE: ml_detumble@2026-08-05a`.

## 6 — Fresh tumble, network flying

C:
```bash
sudo systemctl restart flatsat.target
```
A (immediately after):
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.06,0.05,-0.04 --viz
```
Slot pointer survives — B still shows `version=2026-08-05a`. The
browser's omega plot now draws the NETWORK's detumble (settles ~0.5
mrad/s; PD floor was 2–4).

## 7 — Rollback (browser)

DEPLOY → **rollback** → wait for the pass → C:
```bash
sudo systemctl restart flatsat-adcs
```
B: back to `NO ACTIVE VERSION — fallback PD`.
(`REFUSED: no previous version` is honest bookkeeping if only one
version was ever active — the fallback line after restart is the same
story.)

## Between takes

Browser CONSOLE row → **clear view**: fresh ground console (ground-side
only; next pass repaints current state). For a full vehicle reset:
step 1's restart pair.

## Sim segment (viewer, second tab)

Plain `http://localhost:5173` → **load run…** →
`~/hil-trace/campaign_pd_vs_ml.json`: six releases, pd/ml spans, drag
`sim/ic180mrad/pd.omega_mrad_s` + `…/ml.omega_mrad_s` onto one plot.
Then load the recorded nadir export for the orbit view.

## Appendix — CLI fallbacks (terminal, same effect as the buttons)

```bash
cd ~/flatsat && export FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground send ml_detumble 2026-08-05a build/ml_detumble/2026-08-05a.json --kind model
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground activate ml_detumble 2026-08-05a --ground
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground rollback ml_detumble
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request RECOVERY --ground --reason "bridge restored" --topic-prefix ground
```
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request --status
```

LINK FALLBACK (no radios — same story minus RF): kill both link
services, then on the Jetson:
```bash
cd ~/flatsat && ~/venvs/flatsat-ml/bin/python tools/bench_link.py --vehicle config/vehicles/flatsat_v1_mldemo.txtpb
```
(If `flatsat-link.service` ever gets installed by a units reinstall:
`sudo systemctl disable --now flatsat-link`.)
