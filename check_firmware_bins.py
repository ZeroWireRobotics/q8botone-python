'''
Written by yufeng.wu0902@gmail.com

Checks that the firmware bundled in q8bot/firmware/ is the latest firmware
release: both .bin files must report exactly the version of the newest vX.Y.Z
tag in q8botone-firmware (expected next to this repo - the workspace layout).
A between-tag, dirty, or unversioned build fails too. Runs on every package
build (see build_backend.py), and by hand:

    python check_firmware_bins.py             # exit 1 if the bundled firmware is stale
    python check_firmware_bins.py --staged    # same, for the bins staged in git
                                              # (the pre-commit hook runs this)

Passes with a "skipped" note when there's no q8botone-firmware to compare
against - e.g. a pip install straight from GitHub, where it couldn't be
fixed anyway. Tags are read from the local clone, so fetch first if the
firmware was released from another machine.

Standard library only: it runs inside pip's isolated build environment,
where q8bot's own dependencies (pyserial, for firmware_flasher) aren't
installed.
'''

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent
FIRMWARE_DIR = REPO / 'q8bot' / 'firmware'
FIRMWARE_REPO = REPO.parent / 'q8botone-firmware'
TARGETS = ('robot', 'controller')


def bundled_version(target, staged=False):
    '''
    The version inside q8bot/firmware/<target>.bin (same marker
    firmware_flasher reads) - the working-tree file, or the staged one.
    '''
    path = FIRMWARE_DIR / f'{target}.bin'
    if staged:
        result = subprocess.run(['git', 'show', f':{path.relative_to(REPO).as_posix()}'],
                                cwd=REPO, capture_output=True)
        if result.returncode != 0:
            return 'missing'
        data = result.stdout
    elif path.exists():
        data = path.read_bytes()
    else:
        return 'missing'
    match = re.search(rb'Q8BOT_FW\|' + target.encode() + rb'\|([A-Za-z0-9._+-]+)', data)
    return match.group(1).decode() if match else 'unversioned'


def _git(*args):
    return subprocess.check_output(('git', '-C', str(FIRMWARE_REPO)) + args,
                                   stderr=subprocess.DEVNULL, text=True).strip()


def latest_release():
    '''The newest vX.Y.Z tag in q8botone-firmware, or None if there isn't one to read.'''
    try:
        tags = _git('tag', '--list', 'v[0-9]*', '--sort=-v:refname').split()
    except (OSError, subprocess.CalledProcessError):
        return None
    return next((tag for tag in tags if re.fullmatch(r'v\d+\.\d+\.\d+', tag)), None)


def check(staged=False):
    '''
    staged: check the bins staged in git instead of the working-tree files.

    Returns:
        (ok, lines) - ok is False only when the bundled firmware is provably
        not the latest release; lines explain the result either way.
    '''
    if not FIRMWARE_REPO.is_dir():
        return True, [f'firmware check skipped: no {FIRMWARE_REPO.name} next to this repo']
    tag = latest_release()
    if tag is None:
        return True, [f'firmware check skipped: no vX.Y.Z tag in {FIRMWARE_REPO.name}']

    latest = tag[1:]
    stale = [f'{target}.bin is {version}, not the latest firmware release {latest}'
             for target in TARGETS if (version := bundled_version(target, staged)) != latest]
    if stale:
        return False, stale + [
            f'Fix: build both firmwares at {tag} with no local changes (their build log '
            f'must say Q8BOT_FW_VERSION = {latest}), then run update_firmware_bins.py.']

    lines = [f'bundled firmware {latest} is the latest release ({tag})']
    try:
        unreleased = int(_git('rev-list', '--count', f'{tag}..HEAD'))
    except (OSError, subprocess.CalledProcessError, ValueError):
        unreleased = 0
    if unreleased:
        lines.append(f'note: {FIRMWARE_REPO.name} has {unreleased} commit(s) after {tag} '
                     f'that aren\'t in any release yet')
    return True, lines


def main():
    ok, lines = check(staged='--staged' in sys.argv[1:])
    print('\n'.join(lines))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
