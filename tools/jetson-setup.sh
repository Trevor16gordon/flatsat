#!/usr/bin/env bash
# jetson-setup.sh — reproducible bring-up for Jetson Orin Nano (JetPack 6.2.1)
#
# Assumes: fresh SD-card flash of JP6.2.1, first-boot oem-config completed
# (user created, EULA accepted), running as that user.
#
# Usage:
#   ./jetson-setup.sh                          # run all validated stages
#   WIFI_SSID="MyNet" WIFI_PASS="secret" ./jetson-setup.sh   # also join WiFi
#   FORCE_UPGRADE=1 ./jetson-setup.sh          # force apt upgrade even if run recently
#
# Idempotent: safe to re-run. Every stage checks state first and skips work
# that is already done, so re-running is fast and only fills in what's missing.

set -euo pipefail

log()      { echo -e "\n=== $* ==="; }
skip()     { echo "  [skip] $*"; }
do_()      { echo "  [do]   $*"; }
have_cmd() { command -v "$1" >/dev/null 2>&1; }

# Directory this script lives in (the repo) — manifests land here so they're
# captured in version control automatically.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# dpkg-guarded apt install: only touches apt for packages not already present.
APT_UPDATED=0
apt_update_once() {
  if [[ "$APT_UPDATED" -eq 0 ]]; then
    do_ "apt-get update"
    sudo apt-get update
    APT_UPDATED=1
  fi
}
apt_install() {
  local missing=()
  local p
  for p in "$@"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
  done
  if [[ ${#missing[@]} -eq 0 ]]; then
    skip "apt: already installed: $*"
  else
    apt_update_once
    do_ "apt-get install: ${missing[*]}"
    sudo apt-get install -y "${missing[@]}"
  fi
}

# ---------------------------------------------------------------- N1: base --

if [[ -n "${WIFI_SSID:-}" ]]; then
  log "N1: WiFi ($WIFI_SSID)"
  if nmcli -t -f NAME connection show --active 2>/dev/null | grep -Fxq "$WIFI_SSID"; then
    skip "WiFi already connected to $WIFI_SSID"
  else
    do_ "connect WiFi $WIFI_SSID"
    # delete any stale profile with the same name, then connect
    sudo nmcli connection delete "$WIFI_SSID" 2>/dev/null || true
    sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS"
  fi
fi

log "N1: verify internet"
ping -c 2 -W 5 google.com

log "N1: system updates"
# apt upgrade hits the network; gate behind a daily stamp so re-runs are fast.
# Override with FORCE_UPGRADE=1.
UPGRADE_STAMP="$SCRIPT_DIR/.last-apt-upgrade"
if [[ "${FORCE_UPGRADE:-0}" != "1" ]] \
   && [[ -f "$UPGRADE_STAMP" ]] \
   && [[ -z "$(find "$UPGRADE_STAMP" -mtime +1 2>/dev/null)" ]]; then
  skip "apt upgrade (ran within last day; FORCE_UPGRADE=1 to override)"
else
  apt_update_once
  do_ "apt-get upgrade"
  sudo apt-get upgrade -y
  touch "$UPGRADE_STAMP"
fi

log "N1: monitoring (jtop)"
apt_install python3-pip
JTOP_VERSION="${JTOP_VERSION:-4.3.1}"
if pip3 show jetson-stats 2>/dev/null | grep -q "^Version: ${JTOP_VERSION}$"; then
  skip "jetson-stats==${JTOP_VERSION} already installed"
else
  do_ "pip install jetson-stats==${JTOP_VERSION}"
  sudo pip3 install "jetson-stats==${JTOP_VERSION}"
fi

log "N1: python tooling (pipx + poetry, system python untouched)"
apt_install pipx
pipx ensurepath >/dev/null
POETRY_VERSION="${POETRY_VERSION:-1.8.3}"
if pipx list 2>/dev/null | grep -q "poetry ${POETRY_VERSION}"; then
  skip "poetry==${POETRY_VERSION} already installed"
else
  do_ "pipx install poetry==${POETRY_VERSION}"
  pipx install "poetry==${POETRY_VERSION}" || pipx upgrade poetry
fi
# Jetson wrinkle: TensorRT python bindings are apt-only (system site-packages).
# Projects needing them: python3 -m venv --system-site-packages .venv
# and point poetry at it; pure-python projects get normal isolated envs.

log "N1: code quality tooling (ruff + pre-commit via pipx)"
# Quality contract (pyproject.toml + .pre-commit-config.yaml): every python
# function must have typed inputs/outputs and a docstring — enforced by
# ruff (ANN, D) + mypy at commit time, re-checked in CI.
for tool in ruff pre-commit; do
  if pipx list 2>/dev/null | grep -q "package $tool "; then
    skip "$tool already installed"
  else
    do_ "pipx install $tool"
    pipx install "$tool"
  fi
done
# install the git hook if this checkout carries the config (idempotent)
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$REPO_ROOT/.pre-commit-config.yaml" ]] && [[ -d "$REPO_ROOT/.git" ]]; then
  do_ "pre-commit install (git hook)"
  (cd "$REPO_ROOT" && "$HOME/.local/bin/pre-commit" install)
fi

log "N1: git + GitHub CLI"
apt_install git
# git identity — only set if provided via env AND not already configured.
#   GIT_USER_NAME="Trevor Gordon" GIT_USER_EMAIL="you@example.com" ./jetson-setup.sh
if [[ -n "${GIT_USER_NAME:-}" ]] && [[ -z "$(git config --global user.name || true)" ]]; then
  do_ "git config --global user.name"
  git config --global user.name "$GIT_USER_NAME"
else
  skip "git user.name (already set, or GIT_USER_NAME unset)"
fi
if [[ -n "${GIT_USER_EMAIL:-}" ]] && [[ -z "$(git config --global user.email || true)" ]]; then
  do_ "git config --global user.email"
  git config --global user.email "$GIT_USER_EMAIL"
else
  skip "git user.email (already set, or GIT_USER_EMAIL unset)"
fi
# GitHub CLI from the official apt repo (apt's own gh is old); skip if present.
if have_cmd gh; then
  skip "gh already installed ($(gh --version | head -1))"
else
  do_ "install gh from GitHub's official apt repo"
  have_cmd wget || apt_install wget
  sudo mkdir -p -m 755 /etc/apt/keyrings
  wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null
  sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y gh
fi
# One-time, interactive (browser device-code — not scriptable):
#   gh auth login  ->  GitHub.com  ->  HTTPS  ->  Authenticate Git: Yes  ->  web browser
echo "  note: run 'gh auth login' once to authenticate GitHub (GitHub.com / HTTPS / web browser)"

log "N1: power mode -> MAXN SUPER"
# mode 2 = MAXN SUPER on JP6.2.x Orin Nano; verify with: sudo nvpmodel -q
if sudo nvpmodel -q 2>/dev/null | grep -qE 'NV Power Mode:\s*MAXN'; then
  skip "already in MAXN SUPER (mode 2)"
else
  do_ "nvpmodel -m 2"
  sudo nvpmodel -m 2 || echo "WARN: nvpmodel mode 2 failed; check 'sudo nvpmodel -q'"
fi

# ------------------------------------------------------- N2: radio toolchain --

log "N2: mDNS (ssh trevor@jetson.local)"
apt_install avahi-daemon

log "N2: GNU Radio + IIO/Pluto stack"
# gr-iio ships the fmcomms2 source/sink blocks — the native Pluto interface.
apt_install gnuradio gnuradio-dev libiio-utils libiio0 libad9361-0 libad9361-dev

# ------------------------------------------------- N2b: PyTorch (jp6/cu126) --

log "N2b: PyTorch venv (jp6/cu126 Jetson wheels)"
# Wheels MUST come from the Jetson index — pypi.org's torch 2.8.0 aarch64
# wheel is CPU-only under the IDENTICAL version string, so it is downloaded
# with --no-deps from the Jetson index and installed from the local file;
# dependencies then resolve from pypi.org but can never displace torch itself.
# Venv is --system-site-packages: TensorRT python bindings are apt-only, and
# Track B (B5 live classifier) wants gnuradio + torch in one process.
# numpy<2 pinned first: torch/vision would otherwise pull numpy 2.x into the
# venv, breaking the system gnuradio bindings (built against numpy 1.x).
ML_VENV="${FLATSAT_ML_VENV:-$HOME/venvs/flatsat-ml}"
TORCH_VERSION="${TORCH_VERSION:-2.8.0}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.23.0}"
JETSON_WHEEL_INDEX="${JETSON_WHEEL_INDEX:-https://pypi.jetson-ai-lab.io/jp6/cu126}"
WHEEL_CACHE="$HOME/.cache/flatsat/jetson-wheels"
apt_install python3-venv
if [[ -f "$ML_VENV/bin/python" ]]; then
  skip "venv exists: $ML_VENV"
else
  do_ "python3 -m venv --system-site-packages $ML_VENV"
  mkdir -p "$(dirname "$ML_VENV")"
  python3 -m venv --system-site-packages "$ML_VENV"
fi
if "$ML_VENV/bin/pip" show torch 2>/dev/null | grep -q "^Version: ${TORCH_VERSION}$"; then
  skip "torch==${TORCH_VERSION} already in venv"
else
  do_ "download torch/torchvision from the Jetson index (no deps)"
  mkdir -p "$WHEEL_CACHE"
  "$ML_VENV/bin/pip" download --no-deps -d "$WHEEL_CACHE" \
    --index-url "$JETSON_WHEEL_INDEX" \
    "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}"
  do_ "pip install numpy<2 + local Jetson wheels"
  "$ML_VENV/bin/pip" install "numpy<2"
  # Flight-software runtime deps. tomli only because the onboard interpreter
  # is 3.10; 3.11+ has tomllib in the stdlib (flatsat/core/config.py falls back).
  "$ML_VENV/bin/pip" install eclipse-zenoh protobuf tomli pytest
  "$ML_VENV/bin/pip" install \
    "$WHEEL_CACHE"/torch-"${TORCH_VERSION}"-*.whl \
    "$WHEEL_CACHE"/torchvision-"${TORCHVISION_VERSION}"-*.whl
fi
do_ "N2b gate: verify_torch_cuda.py (CUDA build, GPU compute, gnuradio coexists)"
"$ML_VENV/bin/python" "$SCRIPT_DIR/verify_torch_cuda.py"
# TensorRT: python3-libnvinfer already ships with JP6.2.1 (apt) — reachable
# from this venv via system site-packages; onnxruntime comes later per §3.

# ------------------------------------------------------- N4: zenoh router --

log "N4: zenoh router (multicast-free bus; ARCHITECTURE.md 'The bus off the LAN')"
# Why: zenoh multicast scouting is per-process dice under VPNs/Tailscale
# (one wheel daemon reachable, its twin not — seen in the field). Every
# flatsat daemon switches to CLIENT mode against this router the moment
# /etc/flatsat/bus.env exists; absent file = LAN multicast, unchanged.
ZENOH_LIST="/etc/apt/sources.list.d/zenoh.list"
if [[ -f "$ZENOH_LIST" ]]; then
  skip "zenoh apt repo already configured"
else
  do_ "add eclipse zenoh apt repo"
  echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
    | sudo tee "$ZENOH_LIST" >/dev/null
  sudo apt-get update
  APT_UPDATED=1
fi
apt_install zenoh
# Protocol-parity note: zenohd and the venv's eclipse-zenoh wheel must
# share a protocol major. Print both so a mismatch is visible at setup.
PY_ZENOH_V="$("$ML_VENV/bin/pip" show eclipse-zenoh 2>/dev/null | awk '/^Version:/{print $2}' || true)"
echo "  note: python eclipse-zenoh=${PY_ZENOH_V:-unknown}; $(zenohd --version 2>/dev/null | head -1 || echo 'zenohd version unknown')"
if [[ -f /etc/flatsat/bus.env ]]; then
  skip "/etc/flatsat/bus.env exists"
else
  do_ "write /etc/flatsat/bus.env (daemons become clients of the local router)"
  sudo mkdir -p /etc/flatsat
  echo "FLATSAT_ZENOH_CONNECT=tcp/127.0.0.1:7447" | sudo tee /etc/flatsat/bus.env >/dev/null
fi
do_ "install units (generated + hand-maintained flatsat-router) and enable the router"
sudo bash "$SCRIPT_DIR/install-units.sh"
sudo systemctl enable --now flatsat-router

# --------------------------------------------------- N3: PREEMPT_RT kernel --

log "N3: PREEMPT_RT kernel (NVIDIA OTA debs) + RT test tools"
# Boot-path caveat (this box): the bootloader reads extlinux.conf and the
# kernel image from the SD card's APP partition, NOT from the NVMe rootfs —
# apt installs land on the NVMe and are invisible to boot until synced to
# the SD. That sync (fallback-preserving extlinux edit, root=/dev/nvme0n1p1
# fix — the postinst template wrongly writes root=/dev/mmcblk0p1) is a
# DELIBERATELY MANUAL step: tools/n3-rt-kernel-runbook.md stages 3-4.
# Core-isolation tuning (isolcpus/nohz_full/IRQ affinity) is A1 work.
RT_REPO_LIST="/etc/apt/sources.list.d/nvidia-l4t-rt-kernel.list"
if [[ -f "$RT_REPO_LIST" ]]; then
  skip "rt-kernel apt repo already configured"
else
  do_ "add NVIDIA rt-kernel apt repo (r36.4)"
  echo "deb https://repo.download.nvidia.com/jetson/rt-kernel r36.4 main" \
    | sudo tee "$RT_REPO_LIST" >/dev/null
  sudo apt-get update
  APT_UPDATED=1
fi
apt_install nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers \
  nvidia-l4t-rt-kernel-oot-modules nvidia-l4t-display-rt-kernel
apt_install rt-tests stress-ng
if [[ "$(uname -r)" == *-rt-* ]] && [[ "$(cat /sys/kernel/realtime 2>/dev/null)" == "1" ]]; then
  skip "running the PREEMPT_RT kernel ($(uname -r))"
else
  echo "  note: RT kernel installed but NOT running. Boot-path sync + reboot"
  echo "        are manual: tools/n3-rt-kernel-runbook.md stages 3-4."
fi

# --------------------------------------------------------------- manifests --

log "manifest: dump versioned record of a known-good system"
MANIFEST_DIR="$SCRIPT_DIR/setup-manifests"
mkdir -p "$MANIFEST_DIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
dpkg -l                     > "$MANIFEST_DIR/dpkg-$STAMP.txt"
pip3 list 2>/dev/null       > "$MANIFEST_DIR/pip-$STAMP.txt"
if [[ -f "$ML_VENV/bin/pip" ]]; then
  "$ML_VENV/bin/pip" list 2>/dev/null > "$MANIFEST_DIR/pip-ml-venv-$STAMP.txt"
fi
uname -a                    > "$MANIFEST_DIR/kernel-$STAMP.txt"
cat /etc/nv_tegra_release  >> "$MANIFEST_DIR/kernel-$STAMP.txt" 2>/dev/null || true
echo "  manifest written to $MANIFEST_DIR (commit these to the repo)"

log "Done. Reboot recommended if kernel/power changed: sudo reboot"
