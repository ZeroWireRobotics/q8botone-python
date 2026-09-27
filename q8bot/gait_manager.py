'''
Written by yufeng.wu0902@gmail.com

Gait management module for Q8bot.
This module manages gait trajectories, movement state, and direction switching
for cyclic locomotion patterns.
'''

from .gait_generator import (
    generate_trot_trajectories,
    generate_walk_trajectories,
    generate_bound_trajectories,
    generate_pronk_trajectories,
    generate_crawl_trajectories
)


# Gait parameters dictionary
# Format: 'NAME': [STACKTYPE, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]
GAITS = {
    # 'DRIBBLE':      ['trot', 9.75, 35, 30, 15, 0, 15, 30],
    # 'TROT':      ['trot', 9.75, 43.36, 40, 20, 0, 15, 30],
    'TROT_FAST': ['trot', 9.75, 43.36, 50, 20, 0, 12, 24],
    # Slower companions to TROT_FAST: progressively smaller stride (less
    # xrange/yrange), finer step resolution (more s1/s2 counts). Not
    # directly selectable via switch_gait - GaitManager blends across these
    # based on joystick magnitude whenever TROT_FAST is active (see
    # SPEED_TIER_CHAINS).
    'TROT_SLOW': ['trot', 9.75, 43.36, 30, 20, 0, 18, 36],
    'TROT_MED':  ['trot', 9.75, 43.36, 40, 20, 0, 15, 30],
    'TROT_HIGH': ['trot', 9.75, 60, 20, 10, 0, 15, 30],
    'TROT_LOW':  ['trot', 9.75, 25, 20, 10, 0, 15, 30],
    # Trot in place: same stacktype as the other trots but xrange=0, so legs
    # lift/lower with no horizontal stride at all - direction is irrelevant
    # (f/b/l/r all collapse to the same motion). Not in the normal
    # switch_gait rotation (see operate.py's _internal_only_gaits) - driven
    # directly by the LT trigger via its own dedicated GaitManager instance.
    # yrange/s1/s2 match TROT_MED's "moderate" feel as a starting point.
    'TROT_INPLACE': ['trot', 9.75, 43.36, 0, 30, 0, 15, 30],
    'CRAWL':     ['crawl', 9.75, 40, 10, 20, 0, 50, 25],
    'BOUND':     ['bound', 9.75, 33.36, 40, 0, 20, 50, 10],
    # 'PRONK':     ['pronk', 9.75, 33.36, 40, 0, 20, 60, 10],
}

# Human-readable names for anything a user picks between - a GUI dropdown or
# a printed menu wants these, not the raw dict keys. Descriptions state what
# the parameters above actually do, not what a gait is "for". Gaits with no
# entry (the speed-tier companions) are never shown to a user.
GAIT_LABELS = {
    'TROT_FAST':    ("Trot", "Diagonal pairs, standing height. Stride and step rate "
                             "follow how far the stick is pushed."),
    'TROT_HIGH':    ("High Trot", "Trot from a taller stance with a shorter stride "
                                  "and less foot lift. One speed."),
    'TROT_LOW':     ("Low Trot", "Trot from a crouched stance with a shorter stride "
                                 "and less foot lift. One speed."),
    'TROT_INPLACE': ("Trot in Place", "Trot with no stride at all - the feet lift and "
                                      "land without going anywhere."),
    'CRAWL':        ("Crawl", "One foot at a time, three always down. Slow and "
                              "stable; forward and back only."),
    'BOUND':        ("Bound", "Front and rear pairs move together. Forward and "
                              "back only."),
}


class GaitInfo:
    """
    What a front end needs to show a gait, without reaching into GAITS'
    positional lists. Build with gait_info(); see Q8Bot.catalog.
    """

    __slots__ = ('name', 'label', 'description', 'stance', 'speed_tiers')

    def __init__(self, name, label, description, stance, speed_tiers):
        self.name = name
        self.label = label
        self.description = description
        self.stance = stance          # (x, y) mm the gait rests at
        self.speed_tiers = speed_tiers  # 1 unless the gait blends by stick push

    def __repr__(self):
        return (f"GaitInfo(name={self.name!r}, label={self.label!r}, "
                f"stance={self.stance}, speed_tiers={self.speed_tiers})")


def gait_info(name, gaits=None):
    """
    Display metadata for one gait. Falls back to the raw name as its label
    if it has no GAIT_LABELS entry, so a newly added gait still shows up
    rather than disappearing from a menu.

    Returns:
        GaitInfo, or None if the gait doesn't exist.
    """
    gaits = gaits if gaits is not None else GAITS
    params = gaits.get(name)
    if params is None:
        return None

    label, description = GAIT_LABELS.get(name, (name, ""))
    chain = GaitManager.SPEED_TIER_CHAINS.get(name)
    return GaitInfo(name, label, description, (params[1], params[2]),
                    len(chain) if chain else 1)


class GaitManager:
    """
    Manages gait trajectories, movement state, and direction switching.

    This class encapsulates all state related to cyclic locomotion:
    - Pre-calculated trajectory storage
    - Phase tracking across direction changes
    - Fallback logic for limited movement types
    - Movement state management
    """

    # Fallback mapping for gaits with limited movement types
    FALLBACK_MAP = {
        'fl_0.5': ['fl_0.75', 'fl', 'f'],
        'fl_0.75': ['fl', 'f'],
        'fr_0.5': ['fr_0.75', 'fr', 'f'],
        'fr_0.75': ['fr', 'f'],
        'bl_0.5': ['bl_0.75', 'bl', 'b'],
        'bl_0.75': ['bl', 'b'],
        'br_0.5': ['br_0.75', 'br', 'b'],
        'br_0.75': ['br', 'b'],
        'fl': ['f'],
        'fr': ['f'],
        'bl': ['b'],
        'br': ['b'],
    }

    # Speed-tier chains: when the given gait is active, joystick push
    # magnitude blends across this list of companions, ordered slowest to
    # fastest (the gait itself is always last/fastest). Gaits not listed
    # here are unaffected - always run at their one speed.
    SPEED_TIER_CHAINS = {
        'TROT_FAST': ['TROT_SLOW', 'TROT_MED', 'TROT_FAST'],
    }

    # (up_threshold, down_threshold) hysteresis pair for each boundary
    # between consecutive tiers in a chain above, ordered slowest->fastest
    # (length must be len(chain) - 1). A deadband per boundary (rather than
    # one shared threshold) avoids rapid flicker/phase-resets when the
    # stick sits near it.
    TIER_BOUNDARIES = [
        (0.60, 0.50),  # slow <-> medium
        (0.85, 0.75),  # medium <-> fast
    ]

    def __init__(self, leg, available_gaits=None):
        """
        Initialize the GaitManager.

        Args:
            leg: Kinematics solver instance
            available_gaits: Optional dict of gait definitions (defaults to GAITS)
        """
        self.leg = leg
        self.available_gaits = available_gaits if available_gaits else GAITS
        self.current_trajectories = {}
        self.current_gait = None
        self.current_direction = None
        self.phase_index = 0
        self.ongoing = False
        self.current_trajectory = None

        # Speed-tier state (see SPEED_TIER_CHAINS) - inactive unless the
        # loaded gait has a tier chain. _tier_trajectories is a list aligned
        # with _tier_chain (both ordered slowest->fastest); _speed_tier_index
        # indexes into both (starts at the fastest/last entry).
        self._tiering_active = False
        self._tier_chain = []
        self._tier_trajectories = []
        self._speed_tier_index = 0

    def _generate_trajectories(self, gait_name):
        """
        Compute trajectories for a gait by name, routing to the appropriate
        generator based on its stacktype.

        Returns:
            dict or None: {direction: [frame, ...]} trajectories, or None if
                the gait name is unknown/unsupported or generation failed.
        """
        if gait_name not in self.available_gaits:
            return None

        gait_params = self.available_gaits[gait_name]
        stacktype = gait_params[0]

        if stacktype == 'trot':
            return generate_trot_trajectories(self.leg, gait_params)
        elif stacktype == 'walk':
            return generate_walk_trajectories(self.leg, gait_params)
        elif stacktype == 'bound':
            return generate_bound_trajectories(self.leg, gait_params)
        elif stacktype == 'pronk':
            return generate_pronk_trajectories(self.leg, gait_params)
        elif stacktype == 'crawl':
            return generate_crawl_trajectories(self.leg, gait_params)
        else:
            return None

    def load_gait(self, gait_name):
        """
        Pre-calculate and load trajectories for a given gait. If gait_name
        has a speed-tier chain (see SPEED_TIER_CHAINS), every companion in
        that chain is also precomputed and speed-tier blending is enabled
        via update_speed_tier().

        Args:
            gait_name: Name of the gait (e.g., 'TROT_FAST', 'CRAWL')

        Returns:
            bool: True if successful, False otherwise
        """
        trajectories = self._generate_trajectories(gait_name)
        if trajectories is None:
            return False

        self.current_trajectories = {gait_name: trajectories}
        self.current_gait = gait_name

        chain = self.SPEED_TIER_CHAINS.get(gait_name)
        if chain:
            tier_trajectories = [self._generate_trajectories(name) for name in chain]
            self._tiering_active = all(t is not None for t in tier_trajectories)
        else:
            tier_trajectories = []
            self._tiering_active = False

        if self._tiering_active:
            self._tier_chain = chain
            self._tier_trajectories = tier_trajectories
            self._speed_tier_index = len(chain) - 1  # start at fastest (the gait itself)
        else:
            self._tier_chain = []
            self._tier_trajectories = []
            self._speed_tier_index = 0

        return True

    @staticmethod
    def resolve_tier_index(chain, boundaries, current_index, magnitude):
        """
        Climb/drop through an ordered tier chain based on magnitude, with
        per-boundary hysteresis to avoid flicker near a threshold. Shared
        logic: used internally by update_speed_tier() for gaits that stream
        (e.g. TROT_FAST), and reusable externally (e.g. operate.py's native
        gait tier selection) for identical thresholds/feel.

        Args:
            chain: list, tier names ordered slowest->fastest
            boundaries: list of (up_threshold, down_threshold), len(chain)-1
            current_index: int, current position in chain
            magnitude: float, joystick push distance from center (0.0-1.0)

        Returns:
            int: the resolved index into chain
        """
        idx = current_index

        # Climb toward faster tiers while magnitude clears each boundary's
        # up-threshold (handles a big sudden push skipping several tiers).
        while idx < len(chain) - 1 and magnitude >= boundaries[idx][0]:
            idx += 1

        # Drop toward slower tiers while magnitude falls under each
        # boundary's down-threshold.
        while idx > 0 and magnitude < boundaries[idx - 1][1]:
            idx -= 1

        return idx

    def update_speed_tier(self, magnitude):
        """
        Blend across the active gait's speed-tier chain based on joystick
        push magnitude (0.0-1.0). No-op unless the currently loaded gait has
        a tier chain (see SPEED_TIER_CHAINS) - other gaits are unaffected.

        Args:
            magnitude: float, joystick push distance from center (0.0-1.0)
        """
        if not self._tiering_active:
            return

        idx = self.resolve_tier_index(self._tier_chain, self.TIER_BOUNDARIES, self._speed_tier_index, magnitude)

        if idx != self._speed_tier_index:
            self._speed_tier_index = idx
            self.current_trajectories = {self.current_gait: self._tier_trajectories[idx]}
            self.phase_index = 0  # accept the small stutter on tier change

    def resolve_direction(self, requested_direction):
        """
        Resolve which direction key the currently loaded gait actually has
        trajectory data for - exact match, else FALLBACK_MAP chain, else
        unsupported. Read-only (does not touch movement state); shared by
        start_movement() below, operate.py's native gait execution path, and
        export_gait_tables.py, so all three agree on exactly the same
        directions for a given gait.

        Args:
            requested_direction: Direction string (e.g., 'f', 'b', 'fl_0.75')

        Returns:
            str or None: the resolved direction key, or None if unsupported
        """
        if self.current_gait not in self.current_trajectories:
            return None

        gait_trajectories = self.current_trajectories[self.current_gait]

        if requested_direction in gait_trajectories:
            return requested_direction

        for fallback in self.FALLBACK_MAP.get(requested_direction, []):
            if fallback in gait_trajectories:
                return fallback

        return None

    def start_movement(self, direction):
        """
        Start or switch to a new movement direction.

        Args:
            direction: Direction string (e.g., 'f', 'b', 'fl_0.75')

        Returns:
            bool: True if movement started, False if trajectory not found
        """
        resolved = self.resolve_direction(direction)
        if resolved is None:
            return False

        self.current_trajectory = self.current_trajectories[self.current_gait][resolved]
        self.current_direction = direction  # Remember requested direction
        self.ongoing = True
        return True

    def tick(self):
        """
        Get the next position in the current trajectory.

        Returns:
            list: Joint positions for all motors, or None if no movement active
        """
        if not self.ongoing or self.current_trajectory is None:
            return None

        # Get current position using phase index
        current_index = self.phase_index % len(self.current_trajectory)
        pos = self.current_trajectory[current_index]

        # Increment phase for next tick
        self.phase_index = (self.phase_index + 1) % len(self.current_trajectory)

        return pos

    def stop(self):
        """Stop current movement and reset state."""
        self.ongoing = False
        self.current_direction = None
        self.current_trajectory = None
        self.phase_index = 0

    def is_moving(self):
        """Check if currently executing a movement."""
        return self.ongoing

    def get_current_direction(self):
        """Get the current movement direction."""
        return self.current_direction

    def get_phase(self):
        """Get the current phase index."""
        return self.phase_index
