'''
The robot's firmware, as much of it as moves the legs - so the q8bot library
drives the model just as it drives the robot:

    from q8bot import Q8Bot
    sim = Q8Sim()
    robot = Firmware(sim)
    bot = Q8Bot(SimLink(robot))     # the library's own Q8Bot, unchanged
    bot.move('f')                   # the robot walks its flashed trot table
    robot.step(1000)                # 1 s of the robot's time

Firmware is q8botone-firmware/robot's q8Dynamixel and GaitRunner redone
against the model: the gait tables, the servos' time-based motion profiles,
holding the present pose when torque comes on, the jump and the greet. Its
timing is the robot's, in simulated time - a gait frame every 5 ms, a move
taking its duration. The library's own timing - the gait keep-alive, the
pauses between a routine's poses - is on the wall clock, so run the sim in
real time while the library drives it, as the poser does.

SimLink stands in for q8_espnow, the radio link: whatever Q8Bot sends
becomes a command queued for Firmware, from any thread, and Firmware acts on
it at its next step. Its replies come back the same way. It answers what the
desktop app asks a robot - the joint angles (telemetry), a battery reading so
the app sees it's there, and its Qwiic sensors (none, or an MPU6050 given as
`imu`, see imu.SimMPU6050) - with only what the model has: servo current,
temperature and supply voltage aren't modelled, nor is the battery, which
reads full.
'''

import collections
import struct

import numpy as np
from q8bot import export_gait_tables as flashed
from q8bot.gait_manager import GAITS, GaitManager
from q8bot.kinematics_solver import k_solver
from q8bot.protocol import DIRECTION_ORDER, GAIT_IDS, TICKS_PER_REV, ZERO_OFFSET, ReplyType
from q8bot.routine_generator import G1, G2, G3, G4, G5, G6

from . import imu
from .sim import LEGS, SERVOS, WIRE

# The firmware's numbers - keep in step with q8botone-firmware/robot's
# q8Dynamixel.h/.cpp, gaitRunner.h and main.cpp.
FRAME_S = 0.005             # gaitRunnerTask plays a gait frame every 5 ms
KEEPALIVE_S = 1.0           # a gait not repeated for this long stops (KEEPALIVE_TIMEOUT_MS)
GAIN = 400                  # POSITION_P_GAIN for every normal move - the model's servo gain
JUMP_GAIN = 800             # jump()'s push
DAMPING_GAIN = 30
BOOT_PROFILE_MS = 1000      # the move duration begin() leaves the servos with
IDLE = (30, 150)            # each leg's (front, rear): _idlePos
JUMP_LOW = (-90, 270)       # jump()'s crouch: both upper links straight up
JUMP_HIGH = (90, 90)        # its push: both straight down
JUMP_TUCK = (30, 150)

# The replies, in the library's protocol.py formats. Telemetry is protocol 3's
# shorter one, without the servos' supply voltage and error status; its
# current and temperature go out as 0. The battery reply is the oldest one,
# a percentage and no pack voltage.
TELEMETRY = struct.Struct('<8i8h8b')
SENSORS_HEADER = struct.Struct('<IB')
SENSOR_BLOCK = struct.Struct('<BBB')        # kind, I2C address, how many values
BATTERY_PERCENT = 100


def gait_tables():
    '''The gait tables flashed on the robot, rebuilt as export_gait_tables.py
    writes gaitTables.h: {gait_id: twelve entries, in DIRECTION_ORDER}. Each
    entry is (key, frames) - the table played and its n x 8 servo angles - or
    None for a direction the gait can't do. key is the direction itself, or
    the one it falls back to (CRAWL's curves walk straight).'''
    leg = k_solver(flashed.CENTER_DIST, flashed.L1, flashed.L2, flashed.L1, flashed.L2)
    tables = {}
    for gait_id, name in GAIT_IDS.items():
        manager = GaitManager(leg, GAITS)
        if not manager.load_gait(name):
            raise RuntimeError(f'The library could not build {name}')
        frames = manager.current_trajectories[name]
        keys = [manager.resolve_direction(d) for d in DIRECTION_ORDER]
        tables[gait_id] = [None if k is None else (k, np.array(frames[k], float)) for k in keys]
    return tables


def _progress(t, total, accel):
    '''How far through a Dynamixel time-based motion profile, 0 to 1, at time
    t: total long, speeding up for its first `accel` and slowing for its last.'''
    if accel <= 0:
        return t / total
    v = 1 / (total - accel)                 # the cruising speed, covering the whole move
    if t < accel:
        return 0.5 * v * t * t / accel
    if t > total - accel:
        return 1 - 0.5 * v * (total - t) ** 2 / accel
    return v * (t - accel / 2)


class Firmware:
    '''The robot's firmware, driving the model's servos. Everything here runs
    on the thread that steps the sim; SimLink is the way in from others.'''

    def __init__(self, sim, imu=None):
        self.sim = sim
        self.imu = imu                      # an imu.SimMPU6050 on the Qwiic port, or None
        self.tables = gait_tables()
        self.inbox = collections.deque()    # commands from SimLink, oldest first
        self.outbox = collections.deque()   # replies for it: (ReplyType, payload)
        self._half_step = sim.model.opt.timestep / 2
        self._frame_steps = round(FRAME_S / sim.model.opt.timestep)
        self.reset()

    def reset(self):
        '''As the robot is once a host has its servos on: holding the sim's
        servo targets (call it after Q8Sim.reset()), nothing playing, its
        servos still on the move duration they booted with.'''
        self.inbox.clear()
        self.outbox.clear()
        self._gait = None                   # (gait_id, direction_id) being played
        self._phase = 0
        self._last_run = 0.0                # when GAIT_RUN last came, sim time
        self._script = None                 # a jump or greet under way: a generator of waits, s
        self._script_name = ''
        self._wake = 0.0
        self._steps = 0
        self._profile_ms = BOOT_PROFILE_MS  # the servos' Profile Velocity: how long a move takes
        self._prev_profile = 0              # parseFrame()'s note of the last one a MOVE asked for
        self._turns = np.zeros(8)           # whole turns each servo counts itself off by, deg
        self._mode = self._prev_mode = 1 if self.sim.torque_on else 0   # TORQUE_SET: 0 off, 1 on, 2 damping
        self._set_gain(GAIN)
        self._hold(self.sim.all_targets())

    # ---- for the thread that steps the sim ----------------------------------

    def step(self, n=1):
        '''n physics steps, the firmware doing its part before each.'''
        for _ in range(n):
            self.drain()
            t = self.sim.time
            if self._script is not None and t >= self._wake - self._half_step:
                self._resume(t)
            if self._steps % self._frame_steps == 0:
                self._gait_task(t)
            self._steps += 1
            self.sim.set_all(self._setpoint(t))
            self.sim.step()
            if self.imu is not None:
                self.imu.step()

    def drain(self):
        '''Act on what the link has sent - unless a jump or greet has the robot,
        which, like the real one's radio task, takes nothing more until it's done.'''
        while self._script is None and self.inbox:
            command, *args = self.inbox.popleft()
            getattr(self, '_on_' + command)(*args)

    def targets(self):
        '''The servo targets right now, {leg: (front, rear)} in degrees - partway
        through a move, as far as its motion profile has got.'''
        a = self._setpoint(self.sim.time)
        return {leg: (a[2 * i], a[2 * i + 1]) for i, leg in enumerate(LEGS)}

    def pose(self, targets):
        '''Servo targets set straight away, as the poser's tabs set them -
        {leg: (front, rear)}, legs left out keeping theirs. Stops a gait.'''
        a = np.array(self._setpoint(self.sim.time))
        for i, (leg, s) in enumerate(WIRE):
            if leg in targets:
                a[i] = targets[leg][SERVOS.index(s)]
        self._gait = None
        self._hold(a)
        self.sim.set_all(a)

    def set_torque(self, on):
        '''Servos on (holding the pose they're in) or off, as the host's
        TORQUE_SET would.'''
        self._on_torque(1 if on else 0)

    @property
    def torque_on(self):
        return self._mode != 0

    @property
    def busy(self):
        '''A jump or greet has the robot.'''
        return self._script is not None

    def doing(self):
        '''What the robot is doing, in a few words.'''
        if self._script is not None:
            return self._script_name
        if self._mode == 0:
            return 'limp - servos off'
        if self._gait is not None:
            gait_id, direction_id = self._gait
            asked = DIRECTION_ORDER[direction_id]
            entry = self.tables[gait_id][direction_id]
            if entry is None:
                return f'standing - {GAIT_IDS[gait_id]} has no {asked}'
            key, frames = entry
            via = '' if key == asked else f' (for {asked})'
            return (f'walking {GAIT_IDS[gait_id]} {key}{via}, '
                    f'frame {(self._phase - 1) % len(frames) + 1} of {len(frames)}')
        if self._dur > 0 and self.sim.time - self._t0 < self._dur:
            return f'moving to a pose over {self._dur:g} s'
        return 'holding a pose softly (damping)' if self._mode == 2 else 'holding a pose'

    # ---- the commands, as parseFrame() takes them ---------------------------

    def _on_move(self, angles, dur):
        self._gait = None                   # a raw MOVE always stops a gait
        if dur != self._prev_profile:
            self._set_profile(dur)
            self._prev_profile = dur
        self._write(angles)

    def _on_torque(self, mode):
        if mode != self._prev_mode:
            self._joint_mode(mode)
            self._prev_mode = mode

    def _on_gait_run(self, gait_id, direction_id):
        # GaitRunner::start(): from the top for a new gait, but a new
        # direction in the same one carries the phase over
        if self._gait is None or gait_id != self._gait[0]:
            self._phase = 0
            self._set_profile(0)
        self._gait = (gait_id, direction_id)
        self._last_run = self.sim.time

    def _on_gait_stop(self):
        self._gait = None

    def _on_jump(self):
        self._run('jumping', self._jump())  # the sim's battery never drops below the jump's 50%

    def _on_greet(self):
        self._run('greeting', self._greet())

    def _on_telemetry(self):
        ticks = [round(a * TICKS_PER_REV / 360) + ZERO_OFFSET for a in self.sim.all_angles()]
        self.outbox.append((ReplyType.TELEMETRY_REPLY, TELEMETRY.pack(*ticks, *[0] * 16)))

    def _on_battery(self):
        self.outbox.append((ReplyType.BATTERY_REPLY, bytes([BATTERY_PERCENT])))

    def _on_sensors(self):
        payload = SENSORS_HEADER.pack(round(self.sim.time * 1000), 0 if self.imu is None else 1)
        if self.imu is not None:
            values = self.imu.values()
            payload += (SENSOR_BLOCK.pack(imu.KIND, self.imu.address, len(values))
                        + struct.pack(f'<{len(values)}f', *values))
        self.outbox.append((ReplyType.SENSOR_REPLY, payload))

    # ---- what the robot does ------------------------------------------------

    def _gait_task(self, t):
        '''gaitRunnerTask, every 5 ms: the next frame of the gait being played.'''
        if self._gait is None or self._script is not None:
            return                          # (mid-jump the servo bus is taken: the frame is skipped)
        if t - self._last_run > KEEPALIVE_S:
            self._gait = None               # the host went quiet: stop and stand
            self._run('standing - no GAIT_RUN for a second', self._move_to_idle(500))
            return
        gait_id, direction_id = self._gait
        entry = self.tables.get(gait_id, [None] * 12)[direction_id]
        if entry is None:
            return
        frames = entry[1]
        self._write(frames[self._phase % len(frames)])
        self._phase = (self._phase + 1) % len(frames)

    def _jump(self):
        '''q8Dynamixel::jump()'''
        self._set_profile(500)              # crouch
        yield 0.1
        self._write(np.tile(JUMP_LOW, 4))
        yield 1.0
        self._set_profile(0)                # push, at full speed and double gain
        self._set_gain(JUMP_GAIN)
        yield 0.1
        self._write(np.tile(JUMP_HIGH, 4))
        yield 0.1                           # time for the legs to extend
        self._write(np.tile(JUMP_TUCK, 4))  # tuck, in the air
        yield 0.15
        yield 1.0                           # the landing: the legs hold the tuck (the firmware's limp is off)
        self._set_profile(500)
        self._enable_torque()
        yield 0.1
        self._write(np.tile(IDLE, 4))
        yield 1.0
        self._prev_profile = 500

    def _greet(self):
        '''q8Dynamixel::greet() - the library's greet poses, played by the robot.'''
        self._set_profile(1000)
        yield 0.1
        self._write(G1)
        yield 1.1
        self._write(G2)
        yield 1.0
        self._set_profile(500)
        yield 0.1
        self._write(G3)
        yield 1.0
        self._set_profile(200)
        yield 0.1
        for _ in range(3):
            self._write(G4)
            yield 0.25
            self._write(G5)
            yield 0.25
        yield 0.75
        self._set_profile(500)
        yield 0.1
        self._write(G6)
        yield 0.7

    def _move_to_idle(self, ms):
        '''q8Dynamixel::moveToIdle()'''
        self._set_profile(ms)
        yield 0.1
        self._write(np.tile(IDLE, 4))

    def _run(self, name, script):
        self._script, self._script_name = script, name
        self._resume(self.sim.time)

    def _resume(self, t):
        try:
            self._wake = t + next(self._script)
        except StopIteration:
            self._script = None

    # ---- the servos ---------------------------------------------------------

    def _joint_mode(self, mode):
        '''setJointMode(): anything but off (0) and damping (2) is on.'''
        self._mode = mode
        if mode == 0:
            self.sim.set_torque(False)
            return
        self._enable_torque()
        self._set_gain(DAMPING_GAIN if mode == 2 else GAIN)

    def _enable_torque(self):
        '''enableTorque(): first re-learn which turn each servo counts itself
        in - within half a turn of its idle angle - and make its goal where
        it is now, so torque coming on moves nothing.'''
        present = self.sim.all_angles()
        self._turns = 360 * np.floor((present - (np.tile(IDLE, 4) - 180)) / 360)
        self._hold(present)
        self.sim.set_torque(True)

    def _set_profile(self, ms):
        '''setProfile(): how long the next moves take - and the normal gain back.'''
        self._profile_ms = ms
        self._set_gain(GAIN)

    def _set_gain(self, gain):
        self.sim.set_stiffness(gain / GAIN)

    def _write(self, angles):
        '''bulkWriteDeg(): goals in the library's angles. The servos get there
        over their current move duration - a third of it speeding up, a third
        slowing down (setProfile()'s acceleration) - from wherever they're
        headed now.'''
        t = self.sim.time
        self._from = np.array(self._setpoint(t))
        self._to = np.asarray(angles, float) + self._turns
        self._t0, self._dur, self._acc = t, self._profile_ms / 1000, self._profile_ms // 3 / 1000

    def _hold(self, a):
        self._from = self._to = np.array(a, float)
        self._t0, self._dur, self._acc = self.sim.time, 0.0, 0.0

    def _setpoint(self, t):
        '''Where the servos' motion profile has the targets at time t.'''
        if self._dur <= 0 or t - self._t0 >= self._dur:
            return self._to
        return self._from + (self._to - self._from) * _progress(t - self._t0, self._dur, self._acc)


class SimLink:
    '''q8_espnow's commands - the radio link Q8Bot drives - going to a
    Firmware instead of the robot. Safe from any thread, as the real one is.'''

    DEVICENAME = 'the sim'                  # for Q8Bot's repr: "<Q8Bot on the sim: ...>"

    def __init__(self, robot):
        self.robot = robot

    @property
    def torque_on(self):
        return self.robot.torque_on

    @property
    def bytes_waiting(self):
        return len(self.robot.outbox)

    def enable_torque(self):
        return self._send('torque', 1)

    def disable_torque(self):
        return self._send('torque', 0)

    def set_damping(self):
        return self._send('torque', 2)

    def send_jump(self):
        return self._send('jump')

    def send_greet(self):
        return self._send('greet')

    def run_gait(self, gait_id, direction_id):
        return self._send('gait_run', gait_id, direction_id)

    def stop_gait(self):
        return self._send('gait_stop')

    def move_all(self, joints_pos, dur=0):
        # The radio carries tenths of a degree
        return self._send('move', [round(a * 10) / 10 for a in joints_pos], dur)

    def move_mirror(self, joint_pos, dur=0):
        return self.move_all(list(joint_pos) * 4, dur)

    def check_battery(self):
        return self._send('battery')

    def request_telemetry(self):
        return self._send('telemetry')

    def request_sensors(self):
        return self._send('sensors')

    def set_led_brightness(self, _):
        return True                         # sent, and dropped: the model has no LED

    def read_reply(self):
        try:
            return self.robot.outbox.popleft()
        except IndexError:
            return None

    def flush(self):
        return True

    def close(self):
        pass

    def _send(self, *command):
        self.robot.inbox.append(command)
        return True

