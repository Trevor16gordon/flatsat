# flatsat

Distributed flat-sat testbed + adaptive ML radio — one Jetson Orin Nano flight
computer, two ADALM-Pluto SDRs, two tracks converging at "redeploy an ML model
over the RF link with rollback." Full architecture, status, and decision log:
[`PLAN.md`](PLAN.md).

## Layout

| Path | What |
|------|------|
| `PLAN.md` | The plan: architecture (v1.1), milestones, working status, decision log |
| `tools/jetson-setup.sh` | Idempotent Jetson bring-up (JetPack 6.2.1); re-runs skip completed work |
| `tools/setup-manifests/` | Timestamped dpkg/pip/kernel snapshots of known-good states |
| `radio/` | Shared PHY: Pluto smoke tests, flowgraphs, modem work (see its README for RF safety) |
| `flight/` | Onboard segment: HAL daemons, services, link (from M1) |
| `ground/` | Ground segment on the Mac: modem, MCS, Basilisk (post-P3) |

## Setup

```bash
tools/jetson-setup.sh                       # bring up / verify a Jetson
pre-commit install                          # once per clone — enables quality gates
```

Env vars for the setup script: `WIFI_SSID`/`WIFI_PASS`, `GIT_USER_NAME`/
`GIT_USER_EMAIL`, `FORCE_UPGRADE=1`. One-time manual step: `gh auth login`
(GitHub.com → HTTPS → web browser).

## Code quality — non-negotiable

Every Python function must have **typed inputs and outputs** and a
**docstring**. Enforced twice (ruff `ANN`+`D`, mypy `disallow_untyped_defs`)
at two points (pre-commit hook, CI). A violating commit fails locally; a
`--no-verify` bypass fails in the `quality` workflow. Config:
[`pyproject.toml`](pyproject.toml),
[`.pre-commit-config.yaml`](.pre-commit-config.yaml).

## Conventions

- System Python untouched; project venvs (TensorRT-needing envs use
  `--system-site-packages`). No conda.
- GNU Radio via apt (3.10.1.1) for now — see PLAN.md decision log.
- `extlinux.conf` edits always keep the stock kernel entry as fallback.
- **RF: never TX→RX without the 30 dB of pads inline. Cabled only.**

## Host

Jetson Orin Nano Super Dev Kit, hostname `jetson`, JetPack 6.2.1 (L4T r36.4.7).
Root on 500 GB NVMe; 64 GB microSD retained as rescue boot (flip `root=` back
to `mmcblk0p1` to recover).
