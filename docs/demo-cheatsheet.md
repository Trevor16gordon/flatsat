# Demo cheatsheet — detumble, point at Earth, dump the data

The arc: a tumbling satellite arrests itself, swings its instrument
face onto the Earth and tracks it; a sensor dies and the vehicle safes
itself until ground commands recovery over the radio; then the whole
flight is pulled down and scrubbed in the viewer. (No weight uplink,
no activation/rollback — that machinery exists and is one sentence,
not a segment.)

**Three terminals + two browser tabs + Vizard.**
- **A** — Mac, `~/flatsat`: the Basilisk bridge (the universe)
- **B** — Jetson ssh: the flight journal
- **C** — Jetson ssh: sudo actions
- **Browser tab 1** — live mission control (downlink over the radio)
- **Browser tab 2** — the archive viewer (data dumps)
- **Vizard** — the 3D vehicle

Vehicle: `flatsat_v1` — nadir pointing on the star tracker, momentum
dump on the rods (wheels never ratchet between takes). The link
services still read their comms config from the RF vehicle file; the
topic names are identical.

## 0 — One-time vehicle restore + reset (terminal C)

```bash
cd ~/flatsat && git checkout units/generated && ~/venvs/flatsat-ml/bin/python tools/gen-units.py && sudo bash tools/install-units.sh && sudo systemctl restart flatsat.target && journalctl -u flatsat-adcs -n 3 | grep controller
```
Expect `controller: nadir_point …`.

## Setup (once)

**Terminal C (Jetson)** — flight side of the space link, backgrounded
(idempotent — kills any prior instance):
```bash
pkill -f "apps[.]link_service"; sleep 1; cd ~/flatsat && setsid ~/venvs/flatsat-ml/bin/python -m flatsat.apps.link_service --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb > /tmp/link_flight.log 2>&1 < /dev/null &
```

**Any Mac terminal** — ground station stack, backgrounded (idempotent):
```bash
pkill -f "link_service --ground"; pkill -f "ground_bridge"; sleep 1; cd ~/flatsat && export FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447 && PYTHONPATH=$PWD nohup ~/venvs/gr-ground/bin/python -m flatsat.apps.link_service --ground --vehicle config/vehicles/flatsat_v1_mldemo_rf.txtpb > /tmp/link_ground.log 2>&1 & nohup ~/venvs/flatsat-ground/bin/python tools/ground_bridge.py > /tmp/ground_bridge.log 2>&1 & (curl -s -o /dev/null http://localhost:5173 || nohup npm --prefix web run dev > /tmp/viewer.log 2>&1 &) ; sleep 3 && echo "ground station up"
```

**Browser tab 1** — live mission control:
```
http://localhost:5173/?api=http://localhost:8600
```
LIVE badge = age of the newest downlinked sample. Passes tile the
timeline. COMMANDING panel at the bottom (mode requests cross the
radio at the next pass; CONSOLE → clear view resets the ground display
between takes, nothing reaches the vehicle).

**Terminal B (Jetson)** — flight journal:
```bash
journalctl -u flatsat-adcs -u flatsat-fdir -u flatsat-mode -f | grep --line-buffered -E "\[fdir\]|\[mode\]|\[adcs\] \|omega\||controller:"
```

## Web app restart (any time)

Page won't load → viewer server:
```bash
cd ~/flatsat && npm --prefix web run dev
```
Page loads but LIVE badge red / stale → bridge:
```bash
pkill -f "ground_bridge"; sleep 1; cd ~/flatsat && FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447 nohup ~/venvs/flatsat-ground/bin/python tools/ground_bridge.py > /tmp/ground_bridge.log 2>&1 &
```
Reload the tab after either.

## 1 — Release: tumble → detumble → Earth-pointing

A — bridge first:
```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.05,-0.04,0.03 --viz
```
C — release the vehicle IMMEDIATELY after (within ~10 s: FDIR's stale
window, and the epoch skew stays negligible for nadir):
```bash
sudo systemctl restart flatsat.target
```
**Then Vizard**: Direct Communication → `tcp://localhost:5556` →
Start Visualization.

Watch, in order:
- B: `|omega|` falls 70 → <5 mrad/s (~3 min), then `off-target XX deg`
  appears and falls — the vehicle is swinging +z onto the Earth.
  Settles ~1–2° (orbit-rate tracking + sensor noise: that's physics).
- Vizard: tumble dies, instrument face pins to Earth and TRACKS it.
- Tab 1: drag `downlink/imu.omega_mrad_s` onto a plot — the curve
  draws itself from samples that each rode a pass.

## 2 — Sensor failure: safing without resetting the world

C — kill the IMU (the universe keeps running; nothing resets):
```bash
sudo systemctl stop flatsat-imu0
```
B: `[fdir] tripped: imu_silent — requesting SAFE` →
`[mode] … -> SYSTEM_MODE_SAFE`. Tab 1: mode chip goes SAFE at the next
pass. Pointing drifts slowly (loop holds zero torque on stale input).

C — "repair" the sensor:
```bash
sudo systemctl start flatsat-imu0
```
Vehicle STAYS SAFE — recovery is a human decision. Tab 1, MODE row:
reason `sensor restored` → **→ RECOVERY**, wait a pass, then
**→ NOMINAL**. Watch B: pointing re-acquires, off-target falls back
to ~1–2°.

## 3 — The data dump

Mac (any terminal) — pull the run and export it:
```bash
rm -rf ~/hil-trace/run-latest && RESTART=$(ssh trevor@100.65.0.120 "systemctl show flatsat-recorder.service -p ActiveEnterTimestamp --value") && ssh trevor@100.65.0.120 "cd ~/flatsat-telemetry && find . -type f -newermt \"$RESTART\" -print" | rsync -av --files-from=- trevor@100.65.0.120:~/flatsat-telemetry/ ~/hil-trace/run-latest/ && ~/venvs/flatsat-ground/bin/python -m flatsat.telemetry.export ~/hil-trace/run-latest -o ~/hil-trace/run-latest/mission.json
```
**Browser tab 2** — plain `http://localhost:5173` → **load run…** →
`~/hil-trace/run-latest/mission.json`. Scrub: the run header (vehicle,
code sha, host), full channel list, the orbit panel with the +z arrow
pinned to Earth, gyro/wheel plots, the SAFE window visible in
`sys/mode`.

Also worth loading: yesterday's eclipse-crossing nadir run for the
star-tracker-through-shadow story.

## Between takes

- Fresh tumble = the FULL pair in step 1 (bridge first, then target) —
  the target restart zeroes wheel momentum and starts a new recording.
- Tab 1 CONSOLE → **clear view** for a clean ground console.
- Vehicle stuck SAFE? Tab 1: → RECOVERY, → NOMINAL (ground authority).

## If something breaks

| Symptom | First move |
|---|---|
| LIVE badge red, stays red | link died: rerun BOTH setup blocks (idempotent) |
| Page won't load | viewer server: web app restart section |
| Vizard blank | it connects to the bridge — start the bridge first, then Start Visualization |
| No `off-target` in B | estimator has no attitude: check `systemctl is-active flatsat-st0 flatsat-css0` |
| Wheels near ±100 rad/s | you skipped the target restart — do the step-1 pair |
| Mode refused | check current mode in tab 1; away-from-safety needs RECOVERY first |
