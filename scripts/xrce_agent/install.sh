#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
agent="$(command -v MicroXRCEAgent)"
user="$(id -un)"
if [[ "$user" == root || "$root" == *['"\%$']* || "$agent" == *['"\%$']* ]]; then
  echo 'Run as the Jetson login user from a path without special systemd characters.' >&2
  exit 1
fi
/usr/bin/python3 -c 'import yaml'
unit="$(mktemp)"
trap 'rm -f "$unit"' EXIT
cat > "$unit" <<UNIT
[Unit]
Description=PX4 Micro XRCE-DDS Agent

[Service]
User=$user
SupplementaryGroups=dialout
ExecStart=/usr/bin/python3 "$root/scripts/xrce_agent/run.py" "$root/src/pl_bringup/config/real.yaml" "$agent"
Restart=always
RestartSec=2
KillSignal=SIGTERM

[Install]
WantedBy=multi-user.target
UNIT
sudo install -m 0644 "$unit" /etc/systemd/system/precision-xrce-agent.service
sudo systemctl daemon-reload
sudo systemctl enable precision-xrce-agent.service
sudo systemctl restart precision-xrce-agent.service
