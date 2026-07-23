# flatsat

Bring-up and reproducibility for the **Flat-Sat + Adaptive ML Radio** project — one
Jetson Orin Nano compute node driving two ADALM-Pluto SDRs, split into two
hardware-sharing tracks. See [`PLAN.md`](PLAN.md) for the full plan.

## Contents

| Path | What |
|------|------|
| `jetson-setup.sh` | Idempotent bring-up script for the Jetson (JetPack 6.2.1). Re-runnable; skips anything already installed. |
| `setup-manifests/` | Timestamped `dpkg` / `pip` / kernel snapshots — the versioned record of a known-good system. |
| `PLAN.md` | Project plan: hardware decisions, bring-up milestones (N/P), Track A (flat-sat/FDIR), Track B (adaptive ML radio), convergence. |

## Usage

```bash
./jetson-setup.sh                                   # all validated stages
WIFI_SSID="MyNet" WIFI_PASS="secret" ./jetson-setup.sh   # also join WiFi
GIT_USER_NAME="Trevor Gordon" GIT_USER_EMAIL="you@example.com" ./jetson-setup.sh  # set git identity
FORCE_UPGRADE=1 ./jetson-setup.sh                   # force apt upgrade
```

Installs `git` + the GitHub CLI (`gh`, from GitHub's official apt repo). `gh` auth
is a one-time interactive step the script can't do for you:
`gh auth login` → GitHub.com → HTTPS → *Authenticate Git: Yes* → web browser.

The script is idempotent: each stage checks state first (`dpkg -s`, `pip show`,
`nvpmodel -q`, active WiFi) and skips work already done, so re-running is fast and
only fills in gaps. `apt upgrade` is throttled to once/day via a local stamp.

## Conventions

- **System Python is untouched.** Project code lives in venvs.
- Envs that need the TensorRT bindings (apt-only, system site-packages) use
  `python3 -m venv --system-site-packages .venv`; pure-Python projects get
  normal isolated envs.
- When editing `/boot/extlinux/extlinux.conf` (N3), always keep the stock kernel
  entry as a fallback.
- **RF: never TX→RX without the 30 dB of SMA pads inline. Cabled only — nothing
  transmits over the air.**

## Host

- Jetson Orin Nano Super Dev Kit, hostname `jetson`, JetPack 6.2.1.
- Root on 500 GB NVMe (`/dev/nvme0n1p1`); 64 GB microSD retained as rescue boot
  (flip `root=` back to `mmcblk0p1` in `extlinux.conf` to recover).
