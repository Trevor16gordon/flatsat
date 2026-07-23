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

# ---------------------------------------------- N2b: PyTorch/TensorRT (pending) --
# Added after we validate on-device:
#   - PyTorch from NVIDIA's JP6/CUDA-12.6 Jetson wheel index (NOT PyPI torch)
#   - TensorRT via nvidia-jetpack meta-package
#   - pass check: torch.cuda.is_available() == True

# ------------------------------------------- N3: PREEMPT_RT kernel (pending) --
# Added after N2 validates:
#   - nvidia-l4t-rt-kernel + headers + oot-modules
#   - isolcpus/nohz_full in /boot/extlinux/extlinux.conf (keep stock kernel entry!)
#   - rt-tests, stress-ng; pass check: cyclictest worst-case < ~100 us

# --------------------------------------------------------------- manifests --

log "manifest: dump versioned record of a known-good system"
MANIFEST_DIR="$SCRIPT_DIR/setup-manifests"
mkdir -p "$MANIFEST_DIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
dpkg -l                     > "$MANIFEST_DIR/dpkg-$STAMP.txt"
pip3 list 2>/dev/null       > "$MANIFEST_DIR/pip-$STAMP.txt"
uname -a                    > "$MANIFEST_DIR/kernel-$STAMP.txt"
cat /etc/nv_tegra_release  >> "$MANIFEST_DIR/kernel-$STAMP.txt" 2>/dev/null || true
echo "  manifest written to $MANIFEST_DIR (commit these to the repo)"

log "Done. Reboot recommended if kernel/power changed: sudo reboot"
