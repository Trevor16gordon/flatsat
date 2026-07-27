# Requirements

Each `*.toml` here declares requirements for one subsystem. A requirement is
a claim the system must satisfy, with an identifier, a rationale, and —
crucially — a stated **method of verification** and the **evidence** that
discharges it.

The point is not paperwork. It is that "cyclictest stays under 100 µs under
load" and "a driver read failure never stops the cadence" are things this
project has already *proven experimentally*; writing them as identified
requirements with linked evidence turns a narrative into a specification
that someone else can audit, and turns a passing test suite into a
verification report.

## Entry format

```toml
[[requirement]]
id = "FSW-HAL-001"          # stable, never reused, never renumbered
statement = "..."           # what must be true, testable as written
rationale = "..."           # why it exists; a requirement without a why gets deleted later
verification = "test"       # test | analysis | inspection | demonstration
evidence = "..."            # where the proof lives (measurement, doc, run)
status = "verified"         # draft | allocated | verified | waived
```

`verification = "test"` means an automated test asserts it. Link them by
marking the test:

```python
@pytest.mark.verifies("FSW-HAL-001")
def test_read_failure_does_not_stop_cadence() -> None:
    ...
```

Other methods are for claims a unit test cannot make:
- **analysis** — a measurement campaign or derivation (jitter percentiles
  under load, a link budget).
- **inspection** — reading the code or configuration (a safety interlock
  exists in a specific place).
- **demonstration** — an operator-witnessed run (the HIL detumble).

## Checking traceability

```bash
~/venvs/flatsat-ml/bin/python tools/traceability.py          # matrix
~/venvs/flatsat-ml/bin/python tools/traceability.py --strict # CI gate
```

`--strict` fails when a `verification = "test"` requirement has no test
marked against it, or when a test cites an unknown requirement ID. Both are
real defects: the first is an unverified claim, the second is a dangling
reference that hides one.

## Conventions

- IDs are `FSW-<AREA>-<NNN>`; areas mirror the architecture (`RT`, `HAL`,
  `ADCS`, `BUS`, `RADIO`, `SIM`, `CFG`).
- Statements say what the system does, not how it is coded.
- A requirement whose evidence disappears (test deleted, measurement
  invalidated) goes back to `status = "allocated"` — it does not silently
  stay `verified`.
