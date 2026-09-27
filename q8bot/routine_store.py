'''
Written by yufeng.wu0902@gmail.com

Persistence for user-taught routines (see Q8Bot.capture_pose()/
play_routine()). Each routine is one JSON file: a name and an ordered list
of waypoints, where a waypoint is an 8-angle pose (degrees, move_all()'s
convention) plus how long the move there and the pause after it should take.

Kept separate from Q8Bot itself, the same way routine_generator.py's
hardcoded via-points are - Q8Bot only knows how to read one pose and play
one list back, not where they came from or live on disk.
'''

import json
import re
import time
from pathlib import Path

from .helpers import Q8Logger

# Inside the package but gitignored (see .gitignore) - keeps a user's taught
# routines with the checkout that made them without risking them landing in
# a commit, the way recorded telemetry or other per-robot data would.
DEFAULT_DIR = Path(__file__).resolve().parent / 'routines'

# Applied to every newly captured waypoint (see the teach dialog) - kept here
# rather than only in Q8Bot since this module is what actually stamps them
# onto a waypoint before saving.
DEFAULT_DURATION_MS = 1000  # time move_all() takes to reach the pose ("Move Time" in the teach dialog)
DEFAULT_HOLD_S = 0.0        # pause after arriving, before the next one ("Dwell" in the teach dialog)

# Start's own move is normally just a small settle from wherever hand-posing
# (damping) left the robot, not a full choreographed transition - Q8Bot.idle()
# is a no-op while already idle (the state during teaching), so it does NOT
# reset the pose before Start's own move runs. The general 1s default made
# every routine open with a dead-feeling pause even when there was barely
# anything to correct; still fully user-adjustable per routine via the Move
# Time field if a particular routine's starting pose needs more.
START_DURATION_MS = 300


def _slug(name):
    """Filesystem-safe stem for a routine name: 'Wave Hi!' -> 'wave-hi'."""
    slug = re.sub(r'[^a-z0-9]+', '-', name.strip().lower()).strip('-')
    return slug or 'routine'


def new_waypoint(angles, duration_ms=DEFAULT_DURATION_MS, hold_s=DEFAULT_HOLD_S, name=''):
    """
    One waypoint in the shape play_routine()/save() expect. `name` is a
    label for the teach dialog only (e.g. "Wave up") - purely cosmetic,
    never read by play_routine(). Empty by default; the dialog falls back
    to "Waypoint N" for display rather than storing that as a real name.
    """
    return {'angles': list(angles), 'duration_ms': duration_ms, 'hold_s': hold_s, 'name': name}


def stance_waypoint(role, duration_ms=None, hold_s=DEFAULT_HOLD_S):
    """
    A placeholder meaning "move to whatever the current gait's neutral
    stance is" rather than a captured pose - see Q8Bot.play_routine().
    Every taught routine starts and ends on one of these (role 'start' or
    'end'), so playback always begins and ends at a known, repeatable pose
    without the user needing to manually capture "the robot standing
    normally" as if it were just another waypoint. Deliberately computed at
    play time rather than storing fixed angles here - the neutral stance is
    gait-specific (Q8Bot._pos_x/_pos_y), so a baked-in pose would silently
    go stale the moment the gait or leg calibration changed.

    duration_ms defaults to START_DURATION_MS for role='start' (a quick
    settle, not a full move) and DEFAULT_DURATION_MS for 'end' (a real
    transition back from wherever the routine's last waypoint left it).
    """
    if duration_ms is None:
        duration_ms = START_DURATION_MS if role == 'start' else DEFAULT_DURATION_MS
    return {'angles': None, 'stance': role, 'duration_ms': duration_ms, 'hold_s': hold_s}


def list_routines(directory=None):
    """
    [(name, path), ...] for every saved routine, sorted by name (case-
    insensitive). A file that fails to parse is skipped with a log line
    rather than raising - one bad file on disk shouldn't take down the list.
    """
    # None rather than defaulting the parameter straight to DEFAULT_DIR - a
    # default argument is bound once at import time, so reassigning
    # DEFAULT_DIR later (a test pointing it at a temp dir, say) would
    # silently have no effect on calls that rely on the default.
    if directory is None:
        directory = DEFAULT_DIR
    if not directory.exists():
        return []
    routines = []
    for path in sorted(directory.glob('*.json')):
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            routines.append((data['name'], path))
        except Exception as exc:
            Q8Logger.warning(f"Skipping unreadable routine {path.name}: {exc}")
    routines.sort(key=lambda pair: pair[0].lower())
    return routines


def load(path):
    """The waypoint list from a routine file (see Q8Bot.play_routine())."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    return data['waypoints']


def save(name, waypoints, directory=None):
    """
    Write a routine to <directory>/<slug>.json, overwriting any existing
    file for that name (so re-teaching "wave" replaces it rather than
    piling up wave, wave-1, wave-2...).

    Returns:
        Path: the file written.
    """
    if directory is None:
        directory = DEFAULT_DIR
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{_slug(name)}.json"
    data = {
        'name': name,
        'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'waypoints': waypoints,
    }
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')
    return path


def delete(path):
    """Remove a routine file. Quiet if it's already gone."""
    Path(path).unlink(missing_ok=True)
