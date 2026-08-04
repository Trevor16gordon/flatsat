# Repository architecture

How the code is organized and why. The rule everything follows: **libraries
know nothing about deployment, applications know nothing about devices or
control math, and composition happens in configuration.**

> Status: this describes the **target** architecture settled in the
> 2026-07-27 design session. Code migration is in progress — see PLAN.md §0
> for what has landed.

## The package

`flatsat/` is one installable Python package — the **Tier-1 versionable
artifact**. Everything that ships to a flight computer lives inside it;
everything outside it is data, contracts, or tooling.

```
flatsat/
├── core/                  # framework: bus, config, rt, health, registry
│   └── inference.py       #   (future) ONNX runtime, A/B model slots
├── hardware/
│   ├── sensor.py          # SensorDriver contract + daemon mechanics
│   ├── actuator.py        # ActuatorDriver contract + stale-zeroing + mounting projection
│   ├── drivers/           # jetson_thermal, sim_reaction_wheel, basilisk_imu, basilisk_reaction_wheel
│   └── models/            # noise/bias physics shared by fake devices and the sim feed
├── control/
│   ├── attitude/
│   │   ├── controller.py  # AttitudeController contract (+ reference sources, until one is time-varying)
│   │   ├── controllers/   # rate_damping, pid, (ml_policy later)
│   │   └── estimators/    # passthrough today; EKF later
│   └── health/            # (future, M4) FDIR: rules, arbiter, responses, detectors/
├── comms/                 # (future, when P3 graduates) modem contract, phy/, framing/, link
├── mode/                  # (future) system state machine — imports core ONLY
├── telemetry/             # (future) recorder
├── apps/                  # thin generic executables: sensor_daemon, actuator_daemon, control_loop
├── msgs/                  # generated protobuf bindings (committed, gate-exempt)
└── sim/                   # Basilisk bridge — the universe-fake; never on the flight computer
```

Directories marked *(future)* are names reserved by design and created only
when their first real code lands — never as empty scaffolding.

**Import rule:** every domain may import `core`; `core` imports no domain.
Dependencies only point up the layering: a driver never imports an
application; an application never imports a concrete driver — it resolves
one by name through `core/registry.py` (lazy `module:Class` indirection, so
a board without CUDA can run a thermal daemon, and so remotely-installed
components can register new names).

Outside the package: `config/` (data — textproto instances of the proto
schemas), `requirements/` (verified spec), `deployment.toml` +
`tools/host-profiles/` (where and how things run), `radio/` (experiments
bench), and — on first consumer, not before — `cdh/` (the F´ deployment,
own build system) and `cpp/` (when a measured trigger demands a port).
Proto schemas live INSIDE the package, colocated with their owners: wire
contracts in `flatsat/msgs/*.proto`, config schemas beside the code they
configure. One `tools/gen-protos.sh` run generates every binding next to
its proto; the same files feed C++ when that consumer arrives.

## The contracts

One pattern everywhere: **contract → named implementations → thin generic
app.** The registry mediates.

**`SensorDriver`** (`hardware/sensor.py`) — owns one device, answers
`read() -> (message, validity flags)`, never raises: an acquisition failure
is a flag on a publishable message (flag and forward). Knows nothing about
the bus or cadence — `apps/sensor_daemon.py` supplies that.

**`ActuatorDriver`** (`hardware/actuator.py`) — the write-side twin:
`apply(command) -> flags` (never raises) plus `state() -> message` (a wheel
has real state: speed, momentum, saturation). `apps/actuator_daemon.py`
supplies the bus, cadence, and two protections every actuator inherits:
**stale-command zeroing** (an actuator must zero when commands stop) and the
**mounting projection** (see below).

**`AttitudeController`** (`control/attitude/controller.py`) —
`update(state, reference, dt) -> torque`, in the **body frame**. It takes a
reference, so detumble / pointing / trajectory are objective swaps, not
different loops. Pure: numbers in, numbers out, no bus, no clock; stateful
laws expose `reset()`. Reference-source code lives beside the contract until
the first time-varying objective (e.g. ground-target pointing) earns it a
module of its own.

**`StateEstimator`** (`control/attitude/estimators/`) — measurements in,
state estimate out; sits between sensor topics and the controller. Starts as
`passthrough` (today's behavior made explicit); a real filter is a config
swap.

**FDIR** (`control/health/`, future) — the same sense-decide-act shape with
the *system* as the plant: the limit checker estimates health, the arbiter
decides, the responses actuate (restart daemon / reconfigure / request Safe
— FDIR is a client of mode authority, never the owner).

**`Modem`** (`comms/modem.py`, future) — payload bytes ⇄ RF. Stock GNU
Radio blocks, a hand-built PHY, and a learned receiver are peer
implementations under identical framing — which is what makes the
learned-vs-classical comparison honest.

**ML gets no silo.** Runtime machinery (ONNX sessions, A/B slots, version
stamping) is framework → `core/inference.py`. The models themselves register
into existing registries: an ML controller is a controller, an anomaly
scorer is a detector, a learned receiver is a modem — same limits, same
validity rules, same promotion gate as anything hand-written.

## Configuration: schemas are protos, files are textproto

Config schemas are proto messages COLOCATED with their owners
(`flatsat/vehicle.proto`, `flatsat/hardware/devices.proto`, per-driver
options in `flatsat/hardware/drivers/driver_options.proto`, ...); config
files under `config/` are textproto instances bound to their schema by a
`# proto-file:` header. One edit point: the proto defines the field, the
editor validates and command-clicks the config against it, the generated
stubs type every Python access, and C++ generates from the same file.
Parsing is STRICT — a misspelled key fails startup, never defaults
silently. Implementations are selected by filling exactly one oneof
block; the field name IS the registry key, so the selector and its typed
options cannot disagree. Committed bindings are drift-guarded in CI.

Two kinds of physical truth, split by where the knowledge comes from:

**`config/devices/*.txtpb` — device-intrinsic** (true of the unit no matter
which spacecraft it's bolted to). A *datasheet* section — torque/momentum
envelopes, max update rate, noise characteristics, temp limits, power draw
(designed knowledge) — and a *calibration* section — measured deviations for
this serial number: scale factors, biases, actual-vs-nominal alignment
(discovered knowledge, updated by calibration campaigns, never by editing
the design).

**`config/vehicles/*.txtpb` — integration** (true of this build of this
spacecraft). Composition (which drivers, strategies, topics, rates), plus
the physical model: a `[body]` section (mass, inertia tensor) and a
`mounting` entry per device (position + orientation in the body frame).

Five system views exist conceptually — mechanical, electrical, data,
thermal, compute — and **a view gets a home only when something consumes
it**. Mechanical is consumed now (mounting projection, sim plant). Compute
already exists (`deployment.toml` + host profiles). Electrical (`[power]`:
sources, lines, per-line budgets) arrives with mode-manager load shedding /
A6; data topology (`bus = "i2c0"` per device) arrives with FDIR's
common-cause table; thermal stays per-device datasheet limits.

**The command chain** ties it together: controller outputs body-frame
torque → each actuator daemon projects it through *its own* mounting
geometry (no central mixer until joint allocation — saturation
redistribution, null space — is actually needed) → the driver applies its
internal calibration → hardware. Sensors run the mirror chain inward. Flight
software always uses **nominal ⊕ calibration**.

## Simulation: two different fakes

- **Device fakes** are ordinary drivers (`sim_*` prefix) running on the
  flight computer — open-loop synthetic signals, no external dependencies.
- **The universe fake** is `sim/basilisk_hil.py`: Basilisk physics on the
  ground machine, closed-loop — it consumes actuator commands, integrates
  dynamics, and feeds back the result. Never runs on the flight computer.

Basilisk shows up to the flight software **as drivers** (`basilisk_imu`,
`basilisk_reaction_wheel`): the ordinary daemons run them, so flight-clock
timestamps, sequence numbers, staleness flags, and health telemetry all stay
intact during HIL — no stopping services, no topic squatting. The bridge
builds its plant *from the vehicle file's* `[body]` + mounting and applies
the shared `hardware/models/` corruption, so sim and flight cannot disagree
about the spacecraft by accident. Deliberate disagreement (a `truth_overrides`
block: "actual inertia 8% higher than believed") is the future
robustness-campaign knob.

## Versioning: four tiers

| Tier | What | Mechanism |
|---|---|---|
| 0 | Board image (OS, RT kernel) | A/B rootfs + overlayroot (A5, M7) |
| 1 | FSW release: this package, one artifact | Deployed/rolled back as a unit; version in telemetry |
| 2 | Installed components | Registry registration; B-slot, activate by command, rollback on regression (the C1 path) |
| 3 | Configuration | File (later: parameter database); checksum echoed in telemetry |

The mechanism is uniform; the **authority is gated**: Safe mode's survival
law ships in Tier 1 and is never selected from Tier 2. New components run in
shadow, post numbers, then get promoted — trust is earned by measurement.

## Requirements and verification

`requirements/*.toml` declares what the system must do — stable ID,
rationale, **verification method**, evidence. Tests cite requirements with
`@pytest.mark.verifies("ID")`; `tools/traceability.py --strict` fails CI on
an unverified test-verified requirement or an unknown ID. Methods are named
honestly: `test`, `analysis` (measurement campaigns), `inspection`,
`demonstration`.

## Testing

Tests colocate: **one `<name>_test.py` beside every module**, 1:1. Contract
tests parameterize over every registered implementation, so a new controller
or driver inherits them automatically. Integration tests live beside the app
they exercise; the HIL contract test lives in `sim/`. Tests ship inside the
Tier-1 artifact — an installed release can self-test on the flight computer.

Tests must use test-only topic names: a test subscribing to a production key
silently reads a live daemon's — or a running HIL sim's — traffic.

## Languages

Python first; C++ only when measurement demands it (the port trigger is a
number — see the decision log). The bus + protos are the language-neutral
contract, so a C++ control loop is just another executable publishing the
same messages, selected by deployment, invisible to the rest of the system.
F´ (the C&DH spine, M2) lives in `cdh/` with its own build system, bridged
to the bus by a single component.

## Mission logging: spans, provenance, and the blob

A recording answers two questions the flat telemetry stream could not:
**what was the spacecraft doing**, and **what produced this data**.

**Structure rides the bus.** `flatsat/telemetry/mission_log.py` publishes
`Span` and `Annotation` messages on `mission/**`. The recorder captures them
like any other traffic, so the archive is self-describing — no sidecar, no
index, and slicing a run by sub-mission works from the files alone. A span
is an *interval* and nests; an annotation is a *point event*. Commands are
annotations rather than span boundaries, because a span is defined by the
state it represents: a command that changes mode closes a span, but because
it changed the state, not because it was a command.

Span start and end are separate records rather than one carrying a duration.
A run that dies mid-phase leaves an unclosed span, and that is the finding.

**Provenance is per file, not per run.** `SessionHeader` is written as the
first record of *every* archive file, because files rotate and the oldest
are pruned — metadata held once at the start is the first thing lost. It
carries the run id, source kind, vehicle path and digest *as flown*, git sha
and dirtiness, plant, host, and both clocks. SIM, HIL and FLIGHT publish
identical topics by design; that is exactly why a recording must record
which one it was.

**Sub-missions are reusable.** A `PhaseConfig` standing alone in
`config/submissions/*.txtpb` is a saved building block. A mission phase
naming `use:` takes it as a base and overrides only what differs — plain
proto3 merge, so an unset scalar leaves the base alone. One definition of
"what detumble means" reaches every mission that uses it.

**The blob.** `python -m flatsat.telemetry.export <run-dir>` turns an
archive into one JSON document: run identity, span tree, annotations,
per-topic statistics, and decimated series. Decoding needs a topic-to-type
map (`TOPIC_TYPES`) because protobuf is not self-describing on the wire —
that map is the thing a ground system would call a mission database, and
generating it from the `.proto` descriptors is the obvious next step.
Unmapped topics still parse as `HeaderEnvelope` and contribute timing and
validity, so nothing vanishes silently.

Record a run and export it:

```bash
python -m flatsat.sim.run_mission config/missions/composed_demo.txtpb --record
python -m flatsat.telemetry.export ~/flatsat-missions/<run-id> -o blob.json
```

## The simulated universe: orbit, field, and plant parity

Two plants stand behind the same sim contract. `LocalPlant` is rigid-body
dynamics plus the analytic environment in `flatsat/sim/orbit.py`; it runs
anywhere, including the flight computer and CI. `BasiliskPlant` is real
Basilisk and runs only where Basilisk is installed — today the ground
machine, not the Jetson.

**The parity requirement.** Both plants publish `TruthState`, and flight
software cannot tell them apart. That is the point of the contract, and it
carries an obligation: *a mission file must produce the same physics under
either plant*. Where they differ, the difference must be fidelity — better
gravity, better field — never presence or absence of a phenomenon. A
mission that silently loses its orbit when you ask for the higher-fidelity
plant is worse than one that never had an orbit, because the sim now
disagrees with itself and the disagreement is invisible.

> **GAP CLOSED (2026-08-03).** `BasiliskPlant` now flies the same universe.
> It takes the same `sigma0`, `orbit_elements` and epoch as `LocalPlant`; a
> `gravBodyFactory` Earth carries `orbit.py`'s mu, radius and J2 (fed in as
> the normalized C20 coefficient, built in Python — the pip wheel ships no
> gravity data files); the hub's initial position and velocity come from
> `propagate_eci(elements, 0.0)`, so both plants start from the identical
> state. The `TruthState` environment fields go through one shared
> `fill_environment` helper: position is each plant's OWN integrated state,
> but the field, sun and eclipse evaluated there are always `orbit.py`'s
> models. The remaining plant difference is therefore integration fidelity,
> as required — Basilisk carries J2's short-period oscillations (roughly
> ±5 km at 525 km) that the analytic propagator deliberately averages out,
> so positions agree to ~0.3% over an orbit, exactly rather than
> approximately at epoch. Swapping the dipole for Basilisk's own magnetic
> models would be a deliberate future fidelity upgrade, taken only with a
> parity check in hand.

### What `orbit.py` models

Two-body Kepler with **J2 nodal regression**, and a **tilted dipole**
geomagnetic field. Both choices are load-bearing:

- J2 is not a refinement. It is the entire reason sun-synchronous orbits
  exist, so omitting it deletes the property an SSO mission is simulating.
- The dipole gets field magnitude right to roughly 10-20%, and gets the
  field's *rotation along the orbit* right. That rotation is the only thing
  B-dot detumble depends on — a controller that works against a perfect
  dipole but not the real lumpy field has been tuned to the model.

Absent: drag, third bodies, higher harmonics, IGRF detail, penumbra. The
inertial frame is not J2000 — no precession or nutation. Self-consistent
for simulation; unsuitable for pointing a real antenna.

Tests anchor to values that can be looked up rather than to the code: ISS
period, geostationary sidereal day, the 97.4-degree SSO inclination, one
node revolution per year, 31 uT at the magnetic equator and twice that over
the poles, 20-45% eclipse fraction.

### Orbits are configuration

`config/orbits/*.txtpb` holds saved environments the way
`config/submissions/` holds saved phases. **An unset `inclination_deg`
means "sun-synchronous at this altitude"** — the mission states the
property it wants and the number follows, so changing altitude cannot
silently break sun-synchronicity.

### A magnetorquer has no maximum torque

Recorded here because getting it wrong is easy and the resulting sim lies
in a flattering direction. A magnetorquer is specified by its **magnetic
dipole moment** (A·m²). Torque is `m x B`, so:

- available torque varies continuously around the orbit;
- there is never any torque about the field line, so magnetic control is
  underactuated at every instant;
- detumble works only because B rotates as the vehicle orbits.

Writing `max_torque_n_m` into a magnetorquer device file would grant
authority that does not exist and make detumble look better than reality.
The device parameter is `max_dipole_a_m2`; the plant computes the torque
from the local field.

**Built (2026-08-03).** `MagnetorquerDevice` carries `max_dipole_a_m2` and
nothing else (a test pins the field list so a torque figure cannot arrive
quietly). The chain: `basilisk_magnetometer` corrupts the truth field
through `config/devices/mag0.txtpb`; the `bdot` strategy computes
`m = -k·dB/dt` from a low-passed finite difference and publishes a
`DipoleCommand` (A·m², never N·m); the ordinary actuator daemon projects
it through each rod's mounting (drivers declare their command kind); the
`basilisk_magnetorquer` driver clips to the device envelope and feeds the
applied dipole to whichever plant is running, which computes `m x B` from
ITS OWN local field via one shared function. Measured, not assumed: for
the rideshare deployment tumble, B-dot removes the perpendicular rate
component (0.120 → 0.003 rad/s in ~35 s with the scenario-scale rod)
while the along-B component — 0.033 rad/s for that release attitude —
survives, exactly as `m x B` demands. The mission bound sits above that
floor on purpose.

**The combined system (2026-08-03).** Wheels and rods now fly together:
the `momentum_dump` strategy delegates its torque law to the SAME
rate-damping controller a wheels-only vehicle uses (adding rods cannot
change how the wheels fly), and commands dipole `m = k·(h x B)/|B|²`
from the wheels' own published momentum — the external torque `-k·h_perp`
that lets the wheels hand a deployment tumble's momentum to the field.
The dump goes quiet, never the torque, when the field or any wheel's
state is stale. The control loop carries the second output on
`dipole_output_topic` and fails loudly at startup if a dual-output
strategy has nowhere to send it; mission phases can bound the body-frame
wheel momentum read from the wheels' state topics. `flatsat_v1` now
carries the full complement — three wheels, three real-scale rods, a
magnetometer — so a HIL run detumbles on wheels and then drains them
magnetically over the following tens of minutes, at the tempo the
1.2 A·m² rods honestly have. Still deliberately absent: a joint
allocator (saturation redistribution, null space) — the two commands
remain independent body-frame vectors until a vehicle needs more.

**Attitude knowledge and sun pointing (2026-08-04).** The vehicle now
KNOWS where it points, not just how fast it spins. A coarse sun sensor
(`css0`, 0.02 rad cone noise, honest darkness in eclipse — `sun_visible`
false is a measurement, STALE is the fault) joins the magnetometer, and
the `triad` estimator turns the two measured directions plus onboard
models — deliberately the SAME pure-python `sim/orbit.py` the plants fly,
which is what makes the estimate checkable against truth — into a full
MRP attitude. Sun is the primary vector (its model error is tiny next to
a dipole field's); in eclipse or on a degenerate pair `attitude_valid`
goes false and rates keep passing through. The flagship `sun_point`
strategy aims a body axis at the MEASURED sun (`k_align · a×s` — no
attitude estimate in the loop, so pointing survives everything but
darkness), keeps the detumble PD for rate damping, and reuses the
momentum-dump dipole law verbatim. In eclipse the alignment term pauses
and damping holds. The fastloop tier closes the loop against plant
truth: TRIAD tracks within the sensor floor, acquisition from a tumble
settles under 5°, and shadow produces NaN attitude — never an invented
one.

### Staged build-out

1. Orbit, epoch, geomagnetic field — **done** for both plants (see the closed gap above).
2. Magnetorquer device, driver, B-dot controller — **done** (see above).
3. Coarse sun sensor + magnetometer + TRIAD, sun-pointing strategy — **done** (see above). A star tracker (with sun/Earth exclusion angles) remains the fine-knowledge upgrade when degrees stop being enough.
4. Module layer: `config/modules/*.txtpb` as procurement truth, body mass and inertia **derived** by parallel axis theorem rather than hand-typed, with an optional measured override that warns on disagreement.
5. Power draw per device, summed into a per-mode budget.

Run a mission with an orbit:

```bash
python -m flatsat.sim.run_mission config/missions/deploy_detumble_sso.txtpb --record
```
