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
UNITS="$REPO/flight/units/generated"

if [[ ! -d "$UNITS" ]]; then
  echo "no generated units at $UNITS — run tools/gen-units.py first" >&2
  exit 1
fi

# Remove installed units this vehicle no longer declares — a renamed or
# deleted sensor must not leave a running process for hardware the
# spacecraft does not have. Stop and disable before unlinking.
for installed in /etc/systemd/system/flatsat-*.service; do
  [[ -e "$installed" ]] || continue
  name="$(basename "$installed")"
  if [[ ! -f "$UNITS/$name" ]]; then
    echo "removing stale unit $name"
    systemctl stop "$name" 2>/dev/null || true
    systemctl disable "$name" 2>/dev/null || true
    rm -f "$installed"
  fi
done

install -m 644 "$UNITS"/flatsat-*.service "$UNITS"/flatsat.target /etc/systemd/system/
systemctl daemon-reload

echo "Installed $(ls "$UNITS" | wc -l) unit files."
echo "  start everything : sudo systemctl start flatsat.target"
echo "  stop everything  : sudo systemctl stop flatsat.target"
echo "  status           : systemctl list-units 'flatsat-*'"
echo "  loop jitter logs : journalctl -u flatsat-adcs -f"
