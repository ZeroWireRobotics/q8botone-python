'''
Written by yufeng.wu0902@gmail.com

Q8Bot - one object that owns the link, the gait catalog, and the movement
state machine, so every front end drives the robot the same way: operate.py's
pygame app, a desktop GUI, or a plain script.

    from robot import Q8Bot

    with Q8Bot.connect() as bot:
        bot.set_torque(True)
        bot.stand(1000)
        bot.move('f')          # keeps trotting - returns immediately
        time.sleep(3)
        bot.idle()

Movement continues until told otherwise, and it does so identically in both
execution modes. A background thread ticks at TICK_HZ: in local mode the
robot runs the gait off its own flashed tables and the thread only refreshes
the command periodically, while in remote mode that same thread streams one
frame per tick. Callers never have to know which mode they're in - that's
the whole reason the thread exists.

Commands are safe to call from any thread. The scripted routines (jump,
greet, show_range, shoot) block for as long as the routine takes.
'''

import functools
import math
import threading
import time
from enum import Enum

from . import routine_generator
from .espnow import q8_espnow
from .gait_manager import GAITS, GaitManager, gait_info
from .helpers import Q8Logger, XiaoPortFinder
from .kinematics_solver import k_solver
from .protocol import (
    DIRECTION_ORDER,
    DIRECTION_TO_ID,
    GAIT_NAME_TO_ID,
    ReplyType,
    parse_battery,
    parse_telemetry,
    ticks_to_degrees,
)

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Distance between leg pivots, for body tilt. Measured off the robot's own
# mesh (q8bot/assets, the two actuator blocks sit at x = -39.2 and +38.7, and
# the hips span the full 79.6mm width), so treat these as good estimates
# rather than drawing dimensions - they only scale how many degrees a given
# tilt is *called*, not whether it works. Correct them from CAD when handy.
TRACK_MM = 80.0      # left-to-right, the axis roll tilts about
WHEELBASE_MM = 78.0  # front-to-back, the axis pitch tilts about

# Order of the 8 joint angles on the wire, matching gait_generator's
# append_pos_list(): front-left, front-right, back-left, back-right.
LEG_ORDER = ('FL', 'FR', 'BL', 'BR')

# How often the background thread runs. Matches operate.py's original loop
# rate, which is what the streamed trajectories were tuned against.
TICK_HZ = 200

# Local-mode keep-alive: bounds how long a single dropped GAIT_RUN frame can
# leave host and robot out of sync (there's no ACK/retry). The robot itself
# needs nothing - GaitRunner keeps playing until GAIT_STOP - so this is
# purely insurance, and cheap compared to falling back to streaming.
NATIVE_GAIT_RESEND_INTERVAL = 0.25  # seconds

# Driven by trot_in_place(), never part of the switch-gait rotation.
INPLACE_GAIT = 'TROT_INPLACE'
INTERNAL_ONLY_GAITS = {INPLACE_GAIT}

# Bound per tick so a chatty robot can't stall the loop.
MAX_REPLIES_PER_TICK = 8

# Joint telemetry is request/response - one ask, one snapshot - so there is no
# streaming state on the robot that a crashed host could leave running. Off by
# default: a front end that displays it opts in (the desktop UI uses 10 Hz,
# which costs the servo bus about one percent).
DEFAULT_TELEMETRY_HZ = 0

# Streaming advances the trajectory by elapsed time, not by one frame per
# wakeup, so gait speed doesn't depend on how precisely the host can sleep -
# Windows' timer granularity alone makes a 5ms sleep land nearer 8ms, which
# would otherwise run every streamed gait ~30% slow. When a wakeup covers
# more than one frame the skipped ones are dropped rather than sent late;
# this caps how far a single wakeup may jump, so a long stall (a blocking
# routine, a busy GUI) resumes smoothly instead of flinging the legs.
MAX_STREAM_CATCHUP = 4

# Fallback timing for a taught waypoint that doesn't specify its own (see
# play_routine()) - routine_store stamps every newly captured waypoint with
# these explicitly, so this only matters against a hand-edited routine file.
DEFAULT_ROUTINE_DURATION_MS = 1000  # time move_all() takes to reach the pose
DEFAULT_ROUTINE_HOLD_S = 0.0        # pause after arriving, before the next

# Matches firmware/q8bot_robot/src/main.cpp's jumpEnabledNow gate (pct >= 50).
# The robot enforces this itself regardless - a raw JUMP command below the
# threshold is silently dropped, no reply either way - so this only exists to
# let a host skip jump()'s multi-second settle wait for a jump that was never
# going to happen. Keep in sync if the firmware threshold ever changes.
JUMP_BATTERY_THRESHOLD = 50

# How far (degrees) a joint has to move from _damping_tick()'s last anchor
# before it's treated as a real hand move rather than present-position sensor
# noise. Re-anchoring on every sample (no deadband) was actively
# counterproductive: it reset the position error to ~0 before DAMPING_GAIN's
# already-low P-term ever built up real restoring torque, so a leg crept
# slowly downward under gravity instead of settling, and ordinary read noise
# made a perfectly still joint jitter as the goal ping-ponged by a degree or
# two every sample. Tune this if real hand movement ever gets mistaken for
# noise (raise it) or a held pose still creeps/jitters (lower it).
DAMPING_DEADBAND_DEG = 3.0


class Mode(Enum):
    """What the robot is currently being told to do."""
    IDLE = 'idle'
    NATIVE = 'native'   # robot is running a gait from its own tables
    STREAM = 'stream'   # host is sending one frame per tick


class JointMode(Enum):
    """See Q8Bot.set_joint_mode()."""
    OFF = 'off'         # fully backdrivable, no holding force
    ON = 'on'           # normal full-stiffness position control
    DAMPING = 'damping'  # torqued but soft - holds a hand-posed joint without fighting it


def selectable_gaits(gaits=GAITS):
    """
    The gaits worth offering a user - i.e. what a GUI dropdown or
    next_gait() should cycle through.

    Excludes the non-fastest speed-tier companions (TROT_SLOW/TROT_MED are
    blended in automatically by magnitude, see GaitManager.SPEED_TIER_CHAINS)
    and anything driven by its own dedicated control, like TROT_INPLACE.
    """
    companions = {name for chain in GaitManager.SPEED_TIER_CHAINS.values()
                  for name in chain[:-1]}
    return [name for name in gaits
            if name not in companions and name not in INTERNAL_ONLY_GAITS]


def _needs_link(method):
    """
    Marks a command as needing a live robot. Once the link is gone the
    command does nothing and reports False, rather than going through the
    motions and reporting success against a robot that isn't there.
    """
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        if not self._connected:
            return False
        return method(self, *args, **kwargs)
    return wrapper


class Q8Bot:
    def __init__(self, link, use_local_gaits=True, gaits=GAITS, leg=None,
                 tick_hz=TICK_HZ):
        """
        Args:
            link: a q8_espnow (or anything with the same command surface)
            use_local_gaits: True to run gaits from the robot's flashed
                tables via GAIT_RUN, False to stream every frame from here
            gaits: gait catalog, defaults to gait_manager.GAITS
            leg: kinematics solver, defaults to Q8bot's own geometry
            tick_hz: background thread rate
        """
        self.link = link
        self.use_local_gaits = use_local_gaits
        self.gaits = gaits
        self.leg = leg if leg is not None else k_solver(CENTER_DIST, L1, L2, L1, L2)

        # Guards every field below. Reentrant because the higher-level
        # commands are built out of the lower-level ones (idle -> stand).
        self._lock = threading.RLock()

        # Reply-derived state gets its own lock rather than sharing the one
        # above: a UI polls it every frame, and the scripted routines hold the
        # main lock for seconds at a time (show_range for ~13), which would
        # otherwise freeze that poll - and the UI thread with it.
        self._reply_lock = threading.Lock()

        self._mode = Mode.IDLE
        self._stream_manager = None   # which GaitManager the tick pulls from
        self._native_cmd = None       # (gait_id, direction_id) in flight
        self._native_last_sent = 0.0
        self._native_tier_index = 0
        self._trimmed = False
        self._stream_due = 0.0        # monotonic time the next frame is due
        self._battery_pct = None
        self._battery_volts = None
        self._battery_fresh = False
        self._telemetry = None
        self._telemetry_hz = DEFAULT_TELEMETRY_HZ
        self._telemetry_last_request = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._damping = False          # see set_joint_mode()/_damping_tick()
        self._damping_last_echo = 0.0
        self._damping_anchor = None    # last pose _damping_tick() committed as the goal
        self._connected = True
        self._last_error = None

        # Separate managers so trot_in_place() never disturbs the phase or
        # tier state of whichever gait the sticks are driving.
        self._gait_manager = GaitManager(self.leg, gaits)
        self._inplace_manager = GaitManager(self.leg, gaits)

        self.selectable = selectable_gaits(gaits)
        if not self._load_gait(self.selectable[0]):
            raise RuntimeError(f"Failed to generate trajectories for {self.selectable[0]}")
        if not self._inplace_manager.load_gait(INPLACE_GAIT):
            raise RuntimeError(f"Failed to generate trajectories for {INPLACE_GAIT}")

        self._tick_period = 1.0 / tick_hz
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name='q8bot-tick', daemon=True)
        self._thread.start()

    @classmethod
    def connect(cls, port=None, **kwargs):
        """
        Open a link to the first (or given) ESP32C3 controller and wrap it.

        Moves nothing and leaves torque alone - call set_torque()/stand()
        when you actually want the robot to do something.
        """
        if port is None:
            port = XiaoPortFinder.find()
            if port is None:
                raise RuntimeError("No ESP32C3 controller found - is it plugged in?")
        return cls(q8_espnow(port), **kwargs)

    #-------------------#
    # State             #
    #-------------------#

    @property
    def connected(self):
        """
        False once the link has failed (cable pulled, controller reset) or
        close() has been called. Every command becomes a no-op returning
        False from that point, so a front end should poll this rather than
        trusting a command's return value alone.
        """
        return self._connected

    @property
    def last_error(self):
        """The exception that took the link down, or None."""
        return self._last_error

    @property
    def mode(self):
        return self._mode

    @property
    def gait(self):
        """Name of the gait the sticks/move() are currently driving."""
        return self._gait_manager.current_gait

    @property
    def catalog(self):
        """
        [GaitInfo, ...] for the user-selectable gaits, in set_gait()/
        next_gait() order - label, description, stance and speed-tier count
        for a menu or dropdown.
        """
        return [gait_info(name, self.gaits) for name in self.selectable]

    @property
    def can_turn(self):
        """
        Whether the current gait actually steers.

        CRAWL and BOUND only generate forward and backward; their turn
        variants fall back to straight f/b, so lateral input does nothing at
        all. A front end can use this to disable its steering axis rather
        than let someone push a control that silently no-ops.
        """
        with self._lock:
            return self._gait_manager.resolve_direction('l') == 'l'

    def supported_directions(self):
        """
        Which of protocol.DIRECTION_ORDER the current gait can actually do,
        resolved exactly as move() will (exact match, else fallback chain).
        CRAWL and BOUND, for instance, have no true 'l'/'r'. Lets a front end
        grey out the controls that would do nothing.

        Returns:
            list[str]: direction keys, in DIRECTION_ORDER order.
        """
        with self._lock:
            return [d for d in DIRECTION_ORDER
                    if self._gait_manager.resolve_direction(d) is not None]

    @property
    def stance(self):
        """(x, y) mm baseline the current gait rests at."""
        return self._pos_x, self._pos_y

    @property
    def torque(self):
        return self.link.torque_on

    @property
    def joint_mode(self):
        """Current JointMode - OFF/ON/DAMPING, derived from live state."""
        if self._damping:
            return JointMode.DAMPING
        return JointMode.ON if self.torque else JointMode.OFF

    #-------------------#
    # Movement          #
    #-------------------#

    @_needs_link
    def move(self, direction, magnitude=1.0):
        """
        Walk in `direction` (see protocol.DIRECTION_ORDER) until told
        otherwise. Returns immediately; the background thread keeps it going.

        Safe to call every frame with live stick values - re-issuing the same
        direction is a no-op, and a changed one carries the gait phase over.

        Args:
            direction: e.g. 'f', 'b', 'l', 'fl_0.75'
            magnitude: 0.0-1.0 stick push, picks the speed tier for gaits
                that have one (see GaitManager.SPEED_TIER_CHAINS)

        Returns:
            bool: False if the current gait has no such direction (e.g.
                CRAWL has no true 'l'/'r'), leaving the robot as it was.
        """
        with self._lock:
            if self.use_local_gaits:
                resolved = self._gait_manager.resolve_direction(direction)
                if resolved is None:
                    return False
                self._start_native(self._tiered_gait_id(magnitude),
                                   DIRECTION_TO_ID[resolved])
            else:
                self._gait_manager.update_speed_tier(magnitude)
                if not self._gait_manager.start_movement(direction):
                    return False
                self._start_stream(self._gait_manager)
            return True

    @_needs_link
    def trot_in_place(self):
        """
        Trot without going anywhere (TROT_INPLACE - a trot with zero stride,
        so direction is irrelevant). Runs until idle(), like move().
        """
        with self._lock:
            if self.use_local_gaits:
                self._start_native(GAIT_NAME_TO_ID[INPLACE_GAIT], DIRECTION_TO_ID['f'])
            else:
                if self._stream_manager is not self._inplace_manager:
                    self._inplace_manager.start_movement('f')
                self._start_stream(self._inplace_manager)
            return True

    @_needs_link
    def idle(self):
        """
        Stop whatever gait is running and settle onto the current stance.

        A no-op if already idle, so a polling front end can call it every
        frame without re-sending anything.
        """
        with self._lock:
            if self._mode is Mode.IDLE:
                return
            if self._mode is Mode.NATIVE:
                self._guard(self.link.stop_gait)
            elif self._stream_manager is not None:
                self._stream_manager.stop()

            self._native_cmd = None
            self._stream_manager = None
            self._mode = Mode.IDLE

            self.stand()

    @_needs_link
    def stand(self, dur=0):
        """Move to the current gait's neutral stance, clearing any trim."""
        with self._lock:
            self._trimmed = False
            return self._move_to(self._pos_x, self._pos_y, dur)

    @_needs_link
    def stance_trim(self, dx, dy):
        """
        Shift the standing pose by (dx, dy) mm, keeping the body level.
        Kept as its own name because the gamepad's right stick drives it.
        """
        return self.set_stance(dx, dy)

    @_needs_link
    def set_stance(self, dx=0.0, dy=0.0, roll=0.0, pitch=0.0):
        """
        Pose the robot while standing: shift it by (dx, dy) mm and/or tilt the
        body by roll/pitch in degrees. Idle only - a running gait owns the leg
        positions. All-zero eases back to the baseline rather than adopting
        the current pose as a new one.

        Roll is positive leaning right, pitch positive nose up. Tilt is done
        by extending the legs on one side and retracting the other, which is a
        small-angle approximation of a body rotation - good enough to look and
        behave right, and it needs no model the solver doesn't already have.

        Returns:
            bool: False if not idle, or if any leg's target is unreachable.
        """
        with self._lock:
            if self._mode is not Mode.IDLE:
                return False

            if not (dx or dy or roll or pitch):
                if self._trimmed:
                    self._trimmed = False
                    return self._move_to(self._pos_x, self._pos_y, 300)
                return True

            self._trimmed = True
            x = self._pos_x + dx
            y = self._pos_y + dy

            if not (roll or pitch):
                # Every leg lands on the same target, so one mirrored write
                # does it - a quarter of the payload and no per-leg IK.
                return self._move_to(x, y, 0)

            # Raising a corner means extending that leg, so height offsets go
            # straight onto y. Half-spacing because a tilt lifts one side by
            # as much as it drops the other.
            roll_mm = (TRACK_MM / 2) * math.tan(math.radians(roll))
            pitch_mm = (WHEELBASE_MM / 2) * math.tan(math.radians(pitch))
            offsets = {
                'FL': +roll_mm + pitch_mm,
                'FR': -roll_mm + pitch_mm,
                'BL': +roll_mm - pitch_mm,
                'BR': -roll_mm - pitch_mm,
            }

            angles, reachable = [], True
            for leg in LEG_ORDER:
                q1, q2, ok = self.leg.ik_solve(x, y + offsets[leg], True, 1)
                angles += [q1, q2]
                reachable = reachable and ok

            self._guard(self.link.move_all, angles, 0)
            return reachable

    #-------------------#
    # Gait selection    #
    #-------------------#

    @_needs_link
    def set_gait(self, name):
        """Load `name` and settle onto its stance. False if it won't build."""
        with self._lock:
            if not self._load_gait(name):
                return False
            self.stand(500)
            return True

    def next_gait(self):
        """
        Advance to the next selectable gait (see selectable_gaits()).

        Returns:
            str or None: the new gait's name, None if it failed to load.
        """
        with self._lock:
            order = self.selectable
            nxt = order[(order.index(self._gait_manager.current_gait) + 1) % len(order)]
            return nxt if self.set_gait(nxt) else None

    #-------------------#
    # One-shot commands #
    #-------------------#

    @_needs_link
    def set_torque(self, on):
        """Enable/disable motor torque. Enabling settles onto the stance."""
        with self._lock:
            # Plain on/off always supersedes damping - otherwise a direct
            # caller (the gamepad's RB, say) would leave _damping_tick()
            # still echoing poses at a robot it just told to torque off.
            self._damping = False
            if on:
                self._guard(self.link.enable_torque)
                self.stand(500)
            else:
                self._guard(self.link.disable_torque)
            return on

    @_needs_link
    def set_joint_mode(self, mode):
        """
        Put the joints in one of three states - see JointMode. OFF/ON just
        delegate to set_torque(); DAMPING is new: still torqued, but at a
        much lower position gain (firmware's DAMPING_GAIN) than normal
        moves use, with the last known pose continuously re-issued as the
        goal by the background thread (_damping_tick()) - so a hand can
        push a joint into a new pose against soft resistance and it stays
        there, rather than a fixed low-gain target pulling it back to
        wherever damping started.

        DAMPING is idle-only, like set_stance()/capture_pose() - a running
        gait owns the legs otherwise - and needs telemetry_hz > 0 to have
        anything fresh to re-anchor to (the GUI already sets this whenever
        connected).

        Returns:
            bool: False if DAMPING was requested while not idle, or the
            command failed to send.
        """
        if mode is not JointMode.DAMPING:
            return self.set_torque(mode is JointMode.ON)
        with self._lock:
            if self._mode is not Mode.IDLE:
                return False
            self._damping = True
            self._damping_last_echo = 0.0  # sample on the very next tick
            self._damping_anchor = None    # ...and anchor unconditionally there
            return self._guard(self.link.set_damping)

    def _exit_damping(self):
        """
        Scripted routines and gaits need full stiffness - drop out of
        damping first if it was active, restoring normal gain along with
        it (see q8Dynamixel::setJointMode's mode==1 branch).
        """
        if self._damping:
            self._damping = False
            self._guard(self.link.enable_torque)

    @_needs_link
    def set_led_brightness(self, brightness):
        """0-255, persisted on the robot."""
        return self._guard(self.link.set_led_brightness, brightness)

    @_needs_link
    def request_battery(self):
        """Ask for a battery reading; collect it with take_battery_reading()."""
        return self._guard(self.link.check_battery)

    def take_battery_reading(self):
        """
        The latest battery percentage, once. Returns None until a new reply
        arrives, so a caller can poll this every frame and only act on fresh
        values - no callbacks off the tick thread to worry about.
        """
        with self._reply_lock:
            if not self._battery_fresh:
                return None
            self._battery_fresh = False
            return self._battery_pct

    @property
    def battery_volts(self):
        """
        Raw pack voltage from the last reading, or None if the robot hasn't
        answered yet (or is running firmware that only reports a percentage).

        Latest-value rather than take-once like take_battery_reading(), since
        a UI wants it whenever it repaints.
        """
        with self._reply_lock:
            return self._battery_volts

    @property
    def battery_percent(self):
        """
        Latest known battery percentage, or None before the first reply.
        Latest-value rather than take-once like take_battery_reading() -
        same reasoning as battery_volts: for "what do we know right now"
        rather than "did a new one just arrive".
        """
        with self._reply_lock:
            return self._battery_pct

    @property
    def jump_allowed(self):
        """
        Best guess at whether jump() will actually jump, from the last known
        battery reading. True when nothing is known yet, so this never
        preemptively refuses a jump the firmware might well allow - only
        once a reading has actually come in under JUMP_BATTERY_THRESHOLD
        does this go False.

        A front end can check this before running jump() on a worker thread,
        to skip the wait entirely instead of discovering the no-op only
        after paying for it.
        """
        percent = self.battery_percent
        return percent is None or percent >= JUMP_BATTERY_THRESHOLD

    @property
    def telemetry_hz(self):
        """How often joint snapshots are requested. 0 disables them."""
        return self._telemetry_hz

    @telemetry_hz.setter
    def telemetry_hz(self, hz):
        self._telemetry_hz = max(0.0, float(hz))

    @property
    def telemetry(self):
        """
        The most recent joint snapshot, or None if none has arrived.

        A dict of 'position' (raw ticks), 'current' (mA, signed) and
        'temperature' (degrees C), each 8 long. Latest-value rather than
        take-once like the battery, because this is a continuous stream a UI
        wants to render whenever it repaints.

        Readable with torque off - current just sits near zero because none
        is flowing, while position still tracks the leg if you back-drive it.
        """
        with self._reply_lock:
            return self._telemetry

    #-------------------#
    # Scripted routines #
    #-------------------#
    # These block for the length of the routine and hold the lock, so the
    # tick thread stays quiet and nothing else drives the robot mid-sequence.

    @_needs_link
    def jump(self, settle=5.0):
        """
        Skips the jump - and the multi-second settle wait - if jump_allowed
        is False: the firmware drops CmdType.JUMP outright below
        JUMP_BATTERY_THRESHOLD, so pressing ahead here would just block
        whichever thread called this for `settle` seconds for nothing.

        Returns:
            bool: True if the jump was actually attempted, False if skipped.
        """
        with self._lock:
            if not self.jump_allowed:
                return False
            self.idle()
            self._exit_damping()  # needs full stiffness, not a soft hold
            self._guard(self.link.send_jump)
            time.sleep(settle)
            # A jump can brown out the robot's MCU. If it reset, torque comes
            # back up disabled, so re-enable explicitly rather than assuming
            # the pre-jump state still holds.
            self._guard(self.link.enable_torque)
            self.stand(500)
            return True

    @_needs_link
    def greet(self):
        with self._lock:
            self.idle()
            self._exit_damping()  # needs full stiffness, not a soft hold
            self._guard(routine_generator.greet, self.link)
            self.stand(1000)

    @_needs_link
    def show_range(self):
        with self._lock:
            self.idle()
            self._exit_damping()
            self._guard(routine_generator.show_range, self.link)
            self.stand(1000)

    @_needs_link
    def shoot(self, on_kick=None):
        """on_kick: optional callable fired as the kick starts (see routine_generator)."""
        with self._lock:
            self.idle()
            self._exit_damping()
            self._guard(routine_generator.shoot, self.link, on_kick)
            self.stand(500)

    #-------------------#
    # Taught routines   #
    #-------------------#
    # A taught routine is a list of waypoints in the same shape
    # routine_generator's hardcoded via-points use internally -
    # {'angles': [8 degrees], 'duration_ms', 'hold_s'} - except the angles
    # come from capture_pose() instead of a developer's guesswork. Naming,
    # saving and listing them lives in routine_store.py; Q8Bot only knows how
    # to read one pose and play one list back, the same split
    # routine_generator.py already has from the robot session itself.

    def capture_pose(self):
        """
        The robot's current joint angles, in move_all()'s degrees - read from
        the latest telemetry snapshot rather than commanded state, so this
        reflects wherever the robot actually is: hand-posed with torque off,
        or sculpted with the Stance/Body Tilt pads with torque on. Either
        way, handing the result straight to move_all() (or play_routine())
        returns the servos to exactly this pose.

        Returns:
            list[float] of 8 degrees, or None if no telemetry has arrived
            yet - telemetry_hz must be set above 0 first (see that property).
        """
        if not self._connected:
            return None
        sample = self.telemetry
        if sample is None:
            return None
        return [ticks_to_degrees(t) for t in sample['position']]

    @_needs_link
    def play_routine(self, waypoints):
        """
        Play back a taught routine - a list of waypoints as capture_pose()
        and routine_store.load() produce them. Blocks for the length of the
        routine and holds the lock, exactly like jump()/greet() do, so
        nothing else drives the robot mid-sequence.

        Returns:
            bool: False if a waypoint failed to send (e.g. the link died
            partway through) - the routine stops there rather than pressing
            on into a longer failure.
        """
        with self._lock:
            self.idle()
            self._exit_damping()  # needs full stiffness, not a soft hold
            for waypoint in waypoints:
                duration_ms = waypoint.get('duration_ms', DEFAULT_ROUTINE_DURATION_MS)
                hold_s = waypoint.get('hold_s', DEFAULT_ROUTINE_HOLD_S)
                if not self._guard(self.link.move_all, waypoint['angles'], duration_ms):
                    return False
                time.sleep(hold_s)
            self.stand(500)
            return True

    #-------------------#
    # Lifecycle         #
    #-------------------#

    def close(self, disable_torque=True):
        """Stop the tick thread, park the robot, and close the link."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        with self._lock:
            if self._mode is Mode.NATIVE:
                self._guard(self.link.stop_gait)
            self._mode = Mode.IDLE
            if disable_torque:
                self._guard(self.link.disable_torque)
            # Deliberately after the parting commands, so they still go out,
            # and before link.close() so nothing races in behind them.
            self._connected = False
            try:
                self.link.close()
            except Exception as exc:
                self._last_error = exc

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    #-------------------#
    # Private Functions #
    #-------------------#

    def _guard(self, fn, *args, **kwargs):
        """
        Run something that talks to the link, treating a failure as "the
        robot is gone" rather than letting it escape. A pulled cable raises
        from whichever thread happened to be writing - the tick thread most
        often, where an escaping exception would kill it silently and leave
        the front end showing a robot that stopped moving minutes ago.

        Returns False (and marks the bot disconnected) instead.
        """
        if not self._connected:
            return False
        try:
            return fn(*args, **kwargs)
        except Exception as exc:
            self._link_failed(exc)
            return False

    def _link_failed(self, exc):
        """Record a dead link once, and stop trying to drive it."""
        if not self._connected:
            return
        self._connected = False
        self._last_error = exc
        self._mode = Mode.IDLE
        self._stream_manager = None
        self._native_cmd = None
        Q8Logger.error(f"Lost connection to the robot: {exc}")

    def _load_gait(self, name):
        """Build trajectories for `name` and adopt its stance. No movement."""
        if not self._gait_manager.load_gait(name):
            return False
        params = self.gaits[name]
        self._pos_x, self._pos_y = params[1], params[2]
        chain = GaitManager.SPEED_TIER_CHAINS.get(name)
        # Start fresh at the fastest tier, matching load_gait()'s own convention.
        self._native_tier_index = len(chain) - 1 if chain else 0
        return True

    def _tiered_gait_id(self, magnitude):
        """Pick the gait_id for the current gait, blending tiers by magnitude."""
        chain = GaitManager.SPEED_TIER_CHAINS.get(self._gait_manager.current_gait)
        if not chain:
            return GAIT_NAME_TO_ID[self._gait_manager.current_gait]
        self._native_tier_index = GaitManager.resolve_tier_index(
            chain, GaitManager.TIER_BOUNDARIES, self._native_tier_index, magnitude
        )
        return GAIT_NAME_TO_ID[chain[self._native_tier_index]]

    def _start_native(self, gait_id, direction_id):
        cmd = (gait_id, direction_id)
        if self._mode is not Mode.NATIVE or cmd != self._native_cmd:
            self._guard(self.link.run_gait, gait_id, direction_id)
            self._native_last_sent = time.monotonic()
        self._native_cmd = cmd
        self._mode = Mode.NATIVE

    def _start_stream(self, manager):
        if self._mode is not Mode.STREAM:
            self._stream_due = time.monotonic()
        self._stream_manager = manager
        self._mode = Mode.STREAM

    def _move_to(self, x, y, dur):
        q1, q2, ok = self.leg.ik_solve(x, y, True, 1)
        self._guard(self.link.move_mirror, [q1, q2], dur)
        return ok

    def _tick(self):
        """One pass of the background thread."""
        with self._lock:
            if self._damping and self._mode is not Mode.IDLE:
                # A gait started (e.g. the move pad) while damping was
                # active - damping is idle-only, and a gait needs full
                # stiffness to actually walk, so drop back to full torque
                # rather than let a low-gain leg try to plant a footfall.
                self._exit_damping()

            if self._mode is Mode.STREAM and self._stream_manager is not None:
                self._stream_tick()
            elif self._mode is Mode.NATIVE and self._native_cmd is not None:
                now = time.monotonic()
                if now - self._native_last_sent >= NATIVE_GAIT_RESEND_INTERVAL:
                    self._guard(self.link.run_gait, *self._native_cmd)
                    self._native_last_sent = now
            elif self._damping:
                self._damping_tick()

            self._request_telemetry_if_due()
            self._drain_replies()

    def _damping_tick(self):
        """
        While damping is on, re-anchors the goal position to wherever a
        joint actually is - but only once it's moved more than
        DAMPING_DEADBAND_DEG from the last anchor, not on every fresh
        telemetry sample. A fixed low-gain target would just pull a
        hand-posed joint back toward wherever damping started; re-anchoring
        past a deadband is what makes it feel soft to push and then hold
        still once released, without either of the failure modes a naive
        "echo every sample" version has: re-anchoring continuously erases
        the position error before DAMPING_GAIN's low P-term ever builds up
        real restoring torque (a leg creeps downward under gravity instead
        of settling), and it treats ordinary present-position sensor noise
        as "the user moved it" (a perfectly still joint jitters as the goal
        ping-pongs by a degree or two every sample).

        Throttled to telemetry_hz's own cadence - no point checking for
        movement between fresh samples - and does nothing if telemetry
        isn't flowing (see set_joint_mode()'s note on needing
        telemetry_hz > 0).
        """
        if self._telemetry_hz <= 0:
            return
        now = time.monotonic()
        if now - self._damping_last_echo < 1.0 / self._telemetry_hz:
            return
        sample = self.telemetry
        if sample is None:
            return
        self._damping_last_echo = now
        pose = [ticks_to_degrees(t) for t in sample['position']]

        if self._damping_anchor is not None:
            moved = max(abs(a - b) for a, b in zip(pose, self._damping_anchor))
            if moved < DAMPING_DEADBAND_DEG:
                return  # within noise/droop tolerance - leave the anchor alone

        self._damping_anchor = pose
        self._guard(self.link.move_all, pose, 0)

    def _stream_tick(self):
        """
        Send the trajectory frame that is due now. Advances by however many
        frames the elapsed time covers (see MAX_STREAM_CATCHUP) and sends
        only the last, so the gait runs at its intended speed regardless of
        how accurately the thread wakes up.
        """
        now = time.monotonic()
        if now < self._stream_due:
            return

        pos = None
        steps = 0
        while now >= self._stream_due and steps < MAX_STREAM_CATCHUP:
            pos = self._stream_manager.tick()
            self._stream_due += self._tick_period
            steps += 1

        if now >= self._stream_due:
            # Stalled for longer than the cap allows - give up on the
            # backlog rather than sprinting through it.
            self._stream_due = now + self._tick_period

        if pos:
            self._guard(self.link.move_all, pos, 0)

    def _request_telemetry_if_due(self):
        """One ask per interval. Runs on the tick thread, like the keep-alive."""
        if self._telemetry_hz <= 0:
            return
        now = time.monotonic()
        if now - self._telemetry_last_request < 1.0 / self._telemetry_hz:
            return
        self._telemetry_last_request = now
        self._guard(self.link.request_telemetry)

    def _drain_replies(self):
        self._guard(self._drain_replies_unguarded)

    def _drain_replies_unguarded(self):
        for _ in range(MAX_REPLIES_PER_TICK):
            if self.link.bytes_waiting <= 0:
                return
            reply = self.link.read_reply()
            if reply is None:
                return
            reply_type, payload = reply
            if reply_type == ReplyType.BATTERY_REPLY and payload:
                percent, volts = parse_battery(payload)
                with self._reply_lock:
                    self._battery_pct = percent
                    self._battery_volts = volts
                    self._battery_fresh = True
            elif reply_type == ReplyType.TELEMETRY_REPLY:
                sample = parse_telemetry(payload)
                if sample is not None:
                    with self._reply_lock:
                        self._telemetry = sample

    def _run(self):
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            # Belt and braces on top of _guard(): whatever happens in a tick,
            # this thread has to survive it. A thread that dies here takes
            # every gait with it and tells nobody.
            try:
                self._tick()
            except Exception as exc:
                self._link_failed(exc)
            next_tick += self._tick_period
            delay = next_tick - time.monotonic()
            if delay > 0:
                # time.sleep() rather than the stop event's wait(): on
                # Windows, Event.wait() is stuck with the ~15.6ms system
                # timer granularity while sleep() gets a high-resolution
                # timer, which is the difference between waking 64 and 200
                # times a second. Cost is that close() takes up to one tick
                # to be noticed, which nothing cares about.
                time.sleep(delay)
            else:
                # Fell behind - a blocking routine held the lock, most
                # likely. Resync instead of trying to replay lost ticks.
                next_tick = time.monotonic()
