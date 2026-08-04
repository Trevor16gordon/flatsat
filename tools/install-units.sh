#!/usr/bin/env bash
# Install the generated flatsat systemd units. RUN WITH SUDO.
#
# Regenerate first if the table/profile changed:
#   ~/venvs/flatsat-ml/bin/python tools/gen-units.py
#
# Day-to-day code changes need NO reinstall — units run whatever is in the
# repo; `sudo systemctl restart flatsat-<name>` picks up new code.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNITS="$REPO/units/generated"

if [[ ! -d "$UNITS" ]]; then
  echo "no generated units at $UNITS — run tools/gen-units.py first" >&2
  exit 1
fi

# Remove installed units this vehicle no longer declares — a renamed or
# deleted sensor must not leave a running process for hardware the
# spacecraft does not have. Stop and disable before unlinking.
# HAND-MAINTAINED units (units/*.service, e.g. flatsat-router) are part
# of the declared set too — the sweep must not eat them.
for installed in /etc/systemd/system/flatsat-*.service; do
  [[ -e "$installed" ]] || continue
  name="$(basename "$installed")"
  if [[ ! -f "$UNITS/$name" && ! -f "$REPO/units/$name" ]]; then
    echo "removing stale unit $name"
    systemctl stop "$name" 2>/dev/null || true
    systemctl disable "$name" 2>/dev/null || true
    rm -f "$installed"
  fi
done

install -m 644 "$UNITS"/flatsat-*.service "$UNITS"/flatsat.target /etc/systemd/system/
# Hand-maintained units live one level up from the generated ones.
if compgen -G "$REPO/units/flatsat-*.service" >/dev/null; then
  install -m 644 "$REPO"/units/flatsat-*.service /etc/systemd/system/
fi
systemctl daemon-reload

echo "Installed $(ls "$UNITS" | wc -l) unit files."
echo "  start everything : sudo systemctl start flatsat.target"
echo "  stop everything  : sudo systemctl stop flatsat.target"
echo "  status           : systemctl list-units 'flatsat-*'"
echo "  loop jitter logs : journalctl -u flatsat-adcs -f"
