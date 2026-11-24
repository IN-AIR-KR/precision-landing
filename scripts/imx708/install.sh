#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
cd "$root/.deps/imx708"
if [[ "$(id -u)" != 0 ]]; then
  echo "Run: sudo bash $PWD/install.sh"
  exit 1
fi
expected=5.15.185-tegra
[[ "$(uname -r)" == "$expected" ]] || { echo "Kernel changed; rebuild the driver."; exit 1; }
[[ "$(modinfo -F vermagic ./nv_imx708.ko)" == "$expected "* ]]
backup="/boot/imx708-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup"
cp -a /boot/extlinux/extlinux.conf "$backup/extlinux.conf"
if [[ -f /boot/imx708-cam1.dtb ]]; then cp -a /boot/imx708-cam1.dtb "$backup/imx708-cam1.dtb"; fi
module="/lib/modules/$expected/updates/drivers/media/i2c/nv_imx708.ko"
if [[ -f "$module" ]]; then cp -a "$module" "$backup/nv_imx708.ko"; fi
install -D -m 0644 nv_imx708.ko "$module"
install -m 0644 imx708-cam1.dtb /boot/imx708-cam1.dtb
depmod -a "$expected"
python3 - <<'PY'
from pathlib import Path
import re
path=Path('/boot/extlinux/extlinux.conf')
text=path.read_text()
text=re.sub(r'(?m)^DEFAULT\s+\S+.*$', 'DEFAULT imx708', text)
if not re.search(r'(?m)^DEFAULT ', text):
    text='DEFAULT imx708\n'+text
if not re.search(r'(?m)^LABEL imx708$', text):
    text+=Path('boot-entry.txt').read_text()
path.write_text(text)
PY
echo "Installed IMX708 for CAM1. Backup: $backup"
echo "Power off, connect Camera Module 3 to CAM1, then power on."
echo "To restore boot selection: sudo cp $backup/extlinux.conf /boot/extlinux/extlinux.conf"
