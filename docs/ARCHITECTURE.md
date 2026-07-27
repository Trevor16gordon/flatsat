# Repository architecture

How the code is organized and why. The rule everything follows: **libraries
know nothing about deployment, applications know nothing about devices or
control math, and composition happens in configuration.**

## The four layers

```
protos/            CONTRACTS       wire format between every process (source of truth)
flight/core        FRAMEWORK       bus, config loading, real-time hygiene
flight/hal         DEVICES         SensorDriver contract + one module per device
flight/adcs        STRATEGIES      AttitudeController contract + one module per law
flight/apps        APPLICATIONS    generic executables that compose the above
config/vehicles    COMPOSITION     which drivers + which strategy = this spacecraft
flight/deployment  DEPLOYMENT      RT priority, core role, memory budget
tools/host-profiles  TOPOLOGY      which physical core a role maps to on this board
```

A dependency may only point *up* that list. A driver never imports an
application; an application never imports a concrete driver (it resolves one
through `flight/registry.py`); nothing in `flight/` reads a host profile.

## The two contracts that matter

**`SensorDriver`** (`flight/hal/driver.py`) — owns access to one device and
answers `read() -> (message, validity flags)`. It knows nothing about the
bus, cadence, or consumers; `flight/apps/sensor_daemon.py` supplies all of
that. This is why a simulated device and a real one are interchangeable, and
why `read()` must never raise (an acquisition failure is a flag on a
publishable message — flag and forward).

**`AttitudeController`** (`flight/adcs/controller.py`) —
`update(state, reference, dt) -> torque`. Every strategy answers this: PD,
PID, LQR, an ML policy. Two decisions make it durable:

- It takes a **reference**, not just a measurement. Detumble is "track zero
  rate"; pointing and trajectory following are different `ReferenceSource`
  implementations (`flight/adcs/guidance.py`) feeding the same controller
  interface — not different control loops.
- It is as **pure** as the strategy allows: numbers in, numbers out, no bus,
  no clock. Stateful strategies keep state internal and expose `reset()`.
  That is what makes a law unit-testable, Monte-Carlo-able, and replayable
  against recorded telemetry with none of the flight plumbing.

Both are looked up **by name** in `flight/registry.py`, with per-entry lazy
imports so a board without CUDA can still run a thermal daemon.

## Composition: what a different spacecraft looks like

`config/vehicles/flatsat_v1.toml` declares the sensor complement and how the
vehicle is flown. Every variation you would expect is an edit to that file:

| Want | Change |
|---|---|
| More or fewer sensors | add/remove `[[sensors]]` entries |
| Simulated instead of real | `driver = "sim_gyro"` → `driver = "imu_bno055"` |
| PID instead of PD | `strategy = "pid"` plus its gains |
| ML policy instead of PID | `strategy = "ml_policy"` plus a model path |
| Pointing instead of detumble | `objective` + `objective_options` |
| A different flight computer | a different host profile; vehicle unchanged |
| Harder real-time for a sensor | a `[sensors.<name>]` block in `deployment.toml` |

Deployment is deliberately *not* in the vehicle file: the same spacecraft
definition must be deployable to different computers, so RT policy and
memory budgets live in `flight/deployment.toml` (portable across boards) and
physical core numbers live in `tools/host-profiles/*.env` (per board).
`tools/gen-units.py` joins all three into systemd units.

## Applications are generic

Two executables serve the whole vehicle:

- `flight/apps/sensor_daemon.py --sensor imu0` runs **any** driver.
- `flight/apps/control_loop.py` runs **any** controller against **any**
  reference source.

Adding a sensor or a control law adds a library module and a config entry —
never a new copy of a cadence loop. Both apps do real-time hygiene the same
way (absolute-deadline cadence, GC quiesced, verified scheduling reported)
and instrument themselves with the same percentile report shape as
`cyclictest` and `tools/bus_bench.py`.

## Builds and remote-installable components

Two distinct delivery paths, and the layering above is what keeps them apart:

**Pre-flight build.** The contract bindings and unit files are *generated
artifacts committed to the repo* (`flight/msgs/*_pb2.py` via
`tools/gen-protos.sh`, `flight/units/generated/*` via `tools/gen-units.py`),
so a flight computer needs no protoc and no code generation at deploy time.
The pre-flight build step is: regenerate, run the gates and the test suite,
commit. Future work is to package that into a versioned, immutable artifact
(wheel or container) rather than a git checkout, so deployments are
identified by version and can be rolled back as a unit.

**Remote install and reconfiguration.** Three levels, cheapest first:

1. **Reconfigure** — change gains, rates, thresholds, or the objective by
   replacing a config file. No code moves; the loop echoes the file's
   checksum into telemetry so any recorded run traces to its exact
   parameters. This is the level a commandable parameter database plugs
   into: swap the file read in `flight/core/config.py` for a bus query and
   nothing downstream changes.
2. **Re-compose** — change *which* strategy or driver runs by editing the
   vehicle file and restarting the affected unit. Still no new code.
3. **Install new components** — upload a package that registers additional
   entries in the driver/controller/guidance registries at import time. The
   registry's lazy `module:Class` indirection is what makes this possible
   without touching flight code; an uploaded ML policy is a controller like
   any other, selected by name, running under the same limits and validity
   rules as a hand-written law.

Level 3 is the milestone-C1 path (train on the ground, uplink over RF,
activate in a B slot, roll back on regression) and needs the file-transfer,
A/B slot, and signature machinery that does not exist yet. The seams are in
place so that work does not require restructuring.

## Requirements and verification

`requirements/*.toml` declares what the system must do, each entry carrying a
stable ID, a rationale, a **verification method**, and the evidence that
discharges it. Tests cite requirements with `@pytest.mark.verifies("ID")`,
and `tools/traceability.py` cross-references the two, failing CI (`--strict`)
on an unverified test-verified requirement or a test citing an unknown ID.

Methods are distinguished honestly: `test` (an automated assertion),
`analysis` (a measurement campaign — the cyclictest and bus-latency numbers),
`inspection` (a property visible in code or config), `demonstration` (an
operator-witnessed run). Calling a 10-minute load campaign a unit test would
be worse than naming the method.

## Testing

- **Pure/unit** — control strategies and sensor models, no bus or hardware.
  Includes contract tests parameterized over *every registered strategy*, so
  a new controller inherits them automatically.
- **Integration** — composed daemons publishing on a real bus, including
  real hardware faults (a power-gated thermal zone that EAGAINs proves
  flag-and-forward against something real).
- **Hardware-in-the-loop** — `ground/basilisk_hil.py` runs a simulated
  spacecraft on the ground host and closes the loop through the flight
  computer over the network.

Tests must use test-only topic names. A test that subscribes to a production
key will silently read a live daemon's — or a running HIL sim's — traffic.
