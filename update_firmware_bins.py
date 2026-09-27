'''
Written by yufeng.wu0902@gmail.com

Copies the latest PlatformIO builds into q8bot/firmware/, where the desktop
app's Flash Firmware window looks for them. Expects q8botone-firmware
checked out next to this repo (the q8botone workspace layout). Run after
building:

    cd ../q8botone-firmware/robot && pio run
    cd ../q8botone-firmware/controller && pio run
    python update_firmware_bins.py

Refuses to copy a file whose embedded FIRMWARE_ID doesn't match where it's
going, so a mixed-up build directory fails here rather than at flash time.
'''

import shutil
import sys
import time
from pathlib import Path

from q8bot.firmware_flasher import FIRMWARE_DIR, bin_path, read_bin_target

FIRMWARE_REPO = Path(__file__).resolve().parent.parent / 'q8botone-firmware'
BUILDS = {
    'robot': FIRMWARE_REPO / 'robot' / '.pio' / 'build' / 'robot_permanent' / 'firmware.bin',
    'controller': FIRMWARE_REPO / 'controller' / '.pio' / 'build' / 'controller_permanent' / 'firmware.bin',
}


def main():
    FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
    failed = False
    for target, source in BUILDS.items():
        if not source.exists():
            print(f"{target:<10} no build at {source} - run `pio run` in its project first")
            failed = True
            continue
        found = read_bin_target(source)
        if found != target:
            print(f"{target:<10} REFUSED: {source.name} identifies as {found!r}, not {target!r}")
            failed = True
            continue
        shutil.copy2(source, bin_path(target))
        built = time.strftime('%Y-%m-%d %H:%M', time.localtime(source.stat().st_mtime))
        print(f"{target:<10} copied ({source.stat().st_size:,} bytes, built {built})")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
