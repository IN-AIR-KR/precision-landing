#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
work="$root/.deps/imx708"
revision=ed0b66fddcce9a941175892d70ffa3c8e784db3a
[[ "$(uname -r)" == 5.15.185-tegra ]] || { echo 'This build targets L4T 36.5 / 5.15.185-tegra.'; exit 1; }
mkdir -p "$work"
if [[ ! -d "$work/toolkit/.git" ]]; then
  git init "$work/toolkit"
  git -C "$work/toolkit" remote add origin https://github.com/rm0nroe/nvidia-jetson-toolkit.git
  git -C "$work/toolkit" fetch --depth 1 origin "$revision"
  git -C "$work/toolkit" checkout --detach "$revision"
fi
[[ "$(git -C "$work/toolkit" rev-parse HEAD)" == "$revision" ]]
if ! git -C "$work/toolkit" apply --reverse --check "$root/scripts/imx708/mode-1536x864.patch" 2>/dev/null; then
  git -C "$work/toolkit" apply "$root/scripts/imx708/mode-1536x864.patch"
fi
make -C "$work/toolkit/jp62-imx708-rpi-v3/driver" modules
cp "$work/toolkit/jp62-imx708-rpi-v3/driver/src/nv_imx708.ko" "$work/nv_imx708.ko"
dtc -I dts -O dtb -o "$work/imx708.dtbo" "$root/scripts/imx708/imx708-cam1.dts"
fdtoverlay -i /boot/kernel_tegra234-p3768-0000+p3767-0005-nv-super-uartpio.dtb -o "$work/imx708-cam1.dtb" /boot/jetson-io-hdr40-user-custom.dtbo "$work/imx708.dtbo"
python3 - "$work" <<'PY'
from pathlib import Path
import sys
config=Path('/boot/extlinux/extlinux.conf').read_text()
block=config.split('LABEL JetsonIO\n',1)[1]
append=next(line.strip() for line in block.splitlines() if line.strip().startswith('APPEND '))
Path(sys.argv[1], 'boot-entry.txt').write_text(f"""
LABEL imx708
      MENU LABEL IMX708 Camera Module 3 on CAM1
      LINUX /boot/Image
      FDT /boot/imx708-cam1.dtb
      INITRD /boot/initrd
      {append}
""")
PY
echo 'Built IMX708 module and CAM1 DTB. Install with sudo bash scripts/imx708/install.sh'
