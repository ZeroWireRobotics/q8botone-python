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
Warns (but still copies) when a build isn't a tagged release - between tags,
uncommitted changes, or unversioned - since whatever is copied here ships
with the app.
'''

import re
import shutil
import sys
import time
from pathlib import Path

import check_firmware_bins
from q8bot.firmware_flasher import FIRMWARE_DIR, bin_path, read_bin_identity

FIRMWARE_REPO = Path(__file__).resolve().parent.parent / 'q8botone-firmware'
BUILDS = {
    'robot': FIRMWARE_REPO / 'robot' / '.pio' / 'build' / 'robot_permanent' / 'firmware.bin',
    'controller': FIRMWARE_REPO / 'controller' / '.pio' / 'build' / 'controller_permanent' / 'firmware.bin',
}
# The rest of a complete flash, for a blank board (see firmware_flasher's
# SUPPORT_IMAGES): bootloader and partition table from the builds - the same
# for both boards, which is checked - and boot_app0 from the Arduino core.
SUPPORT_SOURCES = {
    'bootloader.bin': [BUILDS[t].with_name('bootloader.bin') for t in BUILDS],
    'partitions.bin': [BUILDS[t].with_name('partitions.bin') for t in BUILDS],
    'boot_app0.bin': [Path.home() / '.platformio' / 'packages' / 'framework-arduinoespressif32'
                      / 'tools' / 'partitions' / 'boot_app0.bin'],
}
# What a build of exactly a vX.Y.Z tag with no local changes reports.
RELEASE_VERSION = re.compile(r'^\d+\.\d+\.\d+$')


def main():
    FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
    failed = False
    for target, source in BUILDS.items():
        if not source.exists():
            print(f"{target:<10} no build at {source} - run `pio run` in its project first")
            failed = True
            continue
        identity = read_bin_identity(source)
        found = identity.target if identity else None
        if found != target:
            print(f"{target:<10} REFUSED: {source.name} identifies as {found!r}, not {target!r}")
            failed = True
            continue
        shutil.copy2(source, bin_path(target))
        built = time.strftime('%Y-%m-%d %H:%M', time.localtime(source.stat().st_mtime))
        version = identity.version or 'unversioned'
        print(f"{target:<10} copied {version} ({source.stat().st_size:,} bytes, built {built})")
        if not RELEASE_VERSION.match(version):
            print(f"{'':<10} WARNING: not a tagged release build - tag and rebuild before shipping")
    for name, sources in SUPPORT_SOURCES.items():
        missing = [s for s in sources if not s.exists()]
        if missing:
            print(f"{name:<15} not found at {missing[0]}")
            failed = True
            continue
        if len({s.read_bytes() for s in sources}) > 1:
            print(f"{name:<15} REFUSED: robot's and controller's differ - one flash set can't serve both")
            failed = True
            continue
        shutil.copy2(sources[0], FIRMWARE_DIR / name)
        print(f"{name:<15} copied ({sources[0].stat().st_size:,} bytes)")
    # The same check every package build runs - better to hear it now.
    print()
    print('\n'.join(check_firmware_bins.check()[1]))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
