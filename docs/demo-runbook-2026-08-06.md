# Demo runbook — 2026-08-06

Two threads, one storyline: **operations** (a real flight computer,
commanded by real ground software, deploying a learned controller with
authority gates) and **engineering** (the evidence that controller
earned its slot before it flew).

The live vehicle flies `flatsat_v1_mldemo` — the `ml_policy` strategy
with the classical PD as its written-in fallback. No mid-demo unit
swaps; the uplink changes what is in authority, restart-shaped, exactly
as designed.

---

## 0. Tonight's prep (needs Trevor's sudo, ~5 min)

On the Jetson:

```bash
ssh trevor@100.65.0.120
cd ~/flatsat && git pull
# demo units: same fleet, adcs flies flatsat_v1_mldemo
~/venvs/flatsat-ml/bin/python tools/gen-units.py \
  --vehicle "$PWD/config/vehicles/flatsat_v1_mldemo.txtpb" \
  --out "$PWD/units/generated"
sudo bash tools/install-units.sh          # installs generated + flatsat-uplink + router
sudo systemctl enable --now flatsat-uplink
sudo systemctl restart flatsat.target
systemctl is-active flatsat-adcs flatsat-uplink flatsat-router
journalctl -u flatsat-adcs -n 5 | grep ml_policy   # expect: NO ACTIVE VERSION — fallback PD
```

> The demo vehicle now declares a comms block, so a units regen also
> emits `flatsat-link.service` — do NOT enable it; `tools/bench_link.py`
> runs both link ends itself (in-process loopback has no peer).
>
> `units/generated` is now dirty in the Jetson checkout (demo vehicle).
> That is intentional for tomorrow; **do not commit it**. Restore after
> the demo (§6).

On the Mac (already done tonight, listed for completeness):

- artifact: `build/ml_detumble/2026-08-05a.json` (gated; sha256 `56974ca8ec34…`)
- campaign blob: `~/hil-trace/campaign_pd_vs_ml.json` (loads in the viewer)
- every ground command below assumes:

```bash
export FLATSAT_ZENOH_CONNECT=tcp/100.65.0.120:7447
```

---

## 1. Vizard connection (before anything moves)

1. Open Vizard → **Direct Communication**
2. Socket:

```
tcp://localhost:5556
```

3. **Start Visualization** — it waits for the bridge.

---

## 1b. The space link (two-bus rule, PLAN §6)

All ground commanding and the artifact uplink cross a real
store-and-forward link: ground tools publish into the `ground/`
namespace, `tools/bench_link.py` (running on the Jetson) carries the
directional allowlists across a loopback channel in CCSDS frames
during a 10 s contact window every 30 s. Commands WAIT for the pass —
narrate it, don't apologize for it. Commands: docs/demo-cheatsheet.md.

## 2. Thread 1 — live HIL (the spine)

### 2a. Release and detumble on the FALLBACK (classical PD)

```bash
ssh trevor@100.65.0.120 "sudo systemctl restart flatsat.target"
```

then immediately (epoch sync):

```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.05,-0.04,0.03 --viz
```

Watch (second terminal):

```bash
ssh -t trevor@100.65.0.120 "journalctl -u flatsat-adcs -u flatsat-uplink -u flatsat-fdir -u flatsat-mode -f"
```

Talk track: journal says `ml_policy … NO ACTIVE VERSION — fallback PD` —
the learned slot is EMPTY and the vehicle flies the classical law.
|omega| falls from ~70 to <5 mrad/s in a few minutes, visible in Vizard.

### 2b. Fault: blackout, safing, ground-commanded recovery

Ctrl-C the bridge (this IS the blackout — truth stops). Within ~10 s
FDIR trips `imu_stale`, the vehicle safes; journal shows the safing.
Restart the bridge (same command as 2a — flight stack keeps running,
no epoch problem: also restart flatsat.target if you want clean clocks),
then show that recovery does NOT happen by itself:

```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request --status
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request RECOVERY --ground --reason "demo: bridge restored" --topic-prefix ground
~/venvs/flatsat-ground/bin/python -m flatsat.apps.mode_request NOMINAL --ground --reason "checkout complete" --topic-prefix ground
```

Each request queues at the ground station and crosses at the next
contact window (≤30 s) — watch the transition land in the journal.

Talk track: after anything unexpected the system rests in its safest
configuration; only deliberate human action makes it interesting again.

### 2c. OPTIONAL beat: activation refused while SAFE

If you want the gate on stage: while still SAFE (before the RECOVERY
command above), run the activate from 2d — the uplink journal answers
`REFUSED: activation refused in SAFE: survival law is not swappable`.

### 2d. Uplink, activate, fly the learned controller

```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground send ml_detumble 2026-08-05a build/ml_detumble/2026-08-05a.json --kind model
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground activate ml_detumble 2026-08-05a --ground
ssh trevor@100.65.0.120 "sudo systemctl restart flatsat-adcs"
```

(Each crosses at the next pass — terminal E narrates the window.)

Journal now reads:
`controller: ml_policy artifact='ml_detumble' version=2026-08-05a hidden=32 trained_on=fastloop-48ic+… sha256=56974ca8ec34…`

Re-release a tumble to let the NETWORK detumble it live: Ctrl-C the
bridge, restart with a fresh spin (re-request NOMINAL after FDIR safes,
or move quickly inside the stale window):

```bash
~/venvs/flatsat-ground/bin/python -m flatsat.sim.basilisk_hil --orbit config/orbits/starlink_leo.txtpb --omega0 0.06,0.05,-0.04 --viz
```

### 2e. Rollback, live

```bash
~/venvs/flatsat-ground/bin/python -m flatsat.apps.uplink_send --topic-prefix ground rollback ml_detumble
ssh trevor@100.65.0.120 "sudo systemctl restart flatsat-adcs"
```

Rollback needs no authority and works in any mode — toward-safety
actions are never gated. Journal shows the previous state back in
authority.

---

## 3. Thread 2 — the evidence segment (sim, ~5 min)

While the vehicle quietly holds behind you:

1. Mission viewer (`npm --prefix web run dev`, or the running instance)
   → **load run…** → `~/hil-trace/campaign_pd_vs_ml.json`
2. Timeline lane: six releases 30→180 mrad/s, `pd-detumble` and
   `ml-detumble` per release, all judged against the same 5 mrad/s
   floor. Point at the 180 case: HOTTER than anything in training.
3. Drag `sim/ic180mrad/pd.omega_mrad_s` and `…/ml.omega_mrad_s` onto
   one plot — same release, same physics, two laws overlaid.
4. Numbers if asked: ML settles ~0.5 mrad/s on every condition; PD
   spreads 0.3–2.4 (its derivative term chases gyro noise; the clone
   averaged that away across 48 rollouts).
5. Close with the recorded nadir run (`load run…` → yesterday's export):
   orbit view, +z pinned to Earth, `st0` channels live through eclipse.

Talk track: this campaign ran BEFORE the artifact was ever uplinked —
faster than real time, same registry composition, same device models,
same physics the flight computer closes its loop against. Components
earn authority through measurement; deployment is the LAST step.

---

## 4. RF stretch goal (go/no-go at morning rehearsal)

Default demo path sends the artifact through the bench link — real
framing, queues, and contact windows over a loopback channel (rehearsed
end to end).
The RF hop (Pluto loopback through the 30 dB pads, as in the proven
file-uplink) is garnish — attempt ONLY if:

- [ ] morning dress rehearsal of §2 was clean end to end
- [ ] Pluto cabling verified (TX→pads→RX)
- [ ] Trevor gives per-instance RF transmit approval (house rule)

Otherwise say honestly: "the link layer is PHY-swappable; this artifact
rode the network path today, and it has ridden the radio before."

---

## 5. If something breaks

| Symptom | First move |
|---|---|
| Bridge can't connect | `ssh … systemctl is-active flatsat-router`; `nc -vz 100.65.0.120 7447` |
| Channels missing in telemetry | `systemctl --failed 'flatsat-*'` — with the router there is no roulette |
| `activation … REFUSED` | `mode_request --status` — you are SAFE; recover first (that's the design talking) |
| ML flies strangely | `uplink_send rollback ml_detumble` + restart flatsat-adcs — **the rollback IS the demo** |
| Vizard blank | Start Visualization BEFORE the bridge; bridge prints `[viz] client connected` |

---

## 6. After the demo — restore the flight vehicle

```bash
ssh trevor@100.65.0.120
cd ~/flatsat && git checkout units/generated
~/venvs/flatsat-ml/bin/python tools/gen-units.py     # flatsat_v1 default
sudo bash tools/install-units.sh && sudo systemctl restart flatsat.target
```
