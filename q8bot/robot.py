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
)

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

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


class Mode(Enum):
    """What the robot is currently being told to do."""
    IDLE = 'idle'
    NATIVE = 'native'   # robot is running a gait from its own tables
    STREAM = 'stream'   # host is sending one frame per tick


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
        Shift the standing pose by (dx, dy) mm. Idle only - a running gait
        owns the leg positions. Passing (0, 0) eases back to the baseline
        rather than adopting the trim as a new one.

        Returns:
            bool: False if not idle.
        """
        with self._lock:
            if self._mode is not Mode.IDLE:
                return False
            if dx or dy:
                self._trimmed = True
                return self._move_to(self._pos_x + dx, self._pos_y + dy, 0)
            if self._trimmed:
                self._trimmed = False
                return self._move_to(self._pos_x, self._pos_y, 300)
            return True

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
            if on:
                self._guard(self.link.enable_torque)
                self.stand(500)
            else:
                self._guard(self.link.disable_torque)
            return on

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
        with self._lock:
            self.idle()
            self._guard(self.link.send_jump)
            time.sleep(settle)
            # A jump can brown out the robot's MCU. If it reset, torque comes
            # back up disabled, so re-enable explicitly rather than assuming
            # the pre-jump state still holds.
            self._guard(self.link.enable_torque)
            self.stand(500)

    @_needs_link
    def greet(self):
        with self._lock:
            self.idle()
            self._guard(routine_generator.greet, self.link)
            self.stand(1000)

    @_needs_link
    def show_range(self):
        with self._lock:
            self.idle()
            self._guard(routine_generator.show_range, self.link)
            self.stand(1000)

    @_needs_link
    def shoot(self, on_kick=None):
        """on_kick: optional callable fired as the kick starts (see routine_generator)."""
        with self._lock:
            self.idle()
            self._guard(routine_generator.shoot, self.link, on_kick)
            self.stand(500)

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
            if self._mode is Mode.STREAM and self._stream_manager is not None:
                self._stream_tick()
            elif self._mode is Mode.NATIVE and self._native_cmd is not None:
                now = time.monotonic()
                if now - self._native_last_sent >= NATIVE_GAIT_RESEND_INTERVAL:
                    self._guard(self.link.run_gait, *self._native_cmd)
                    self._native_last_sent = now

            self._request_telemetry_if_due()
            self._drain_replies()

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
