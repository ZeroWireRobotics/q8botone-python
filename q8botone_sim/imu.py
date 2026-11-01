'''
The robot's IMU - an MPU6050 on its Qwiic port - for the digital twin: which
way its axes point in the chassis, found by calibration, and the chassis's
orientation, tracked from its readings.

    mount = fit_mount(poses)                # the robot standing still, tilted a few ways
    attitude = Attitude(mount)
    attitude.update(t, accel, gyro)         # every reading, in order
    attitude.orientation()                  # 3x3, world <- chassis

Nothing about where or how the chip is mounted is assumed: calibration finds
its axes from readings taken while the robot tilts its body on its own feet -
the joint angles say which way up the chassis is then (Q8Sim.floor_up), the
accelerometer which way up the chip is. The gyro's offset, which on a cheap
MPU6050 can be tens of degrees a second, is measured while it stands still
and kept up to date whenever it does.

SimMPU6050 is one on the model, for the sim's robot to answer with - the
chip's faults included, if asked.
'''

import json
import math

import numpy as np

KIND = 2                            # an MPU6050's sensor kind in the robot's SENSOR_REPLY
VALUES = 7                          # accel x y z (m/s^2), gyro x y z (rad/s), temperature (C)
STANDARD_G = 9.80665

# Calibration: the body tilts (roll, pitch, deg) the robot holds still in,
# each eased into from level and back - a servo told to go straight there
# goes at full speed, and a body thrown from one side to the other can
# tip over - then left to settle, then read
CALIBRATION_TILTS = ((0, 0), (12, 0), (-12, 0), (0, 12), (0, -12))
EASE_STEPS = 10
EASE_STEP_S = 0.05
SETTLE_S = 0.8
GATHER_S = 0.6
# How far the readings may stray from one mounting (deg RMS), and how
# differently the tilts must point the chip's up - two ways, or the turn
# about up isn't pinned down (deg)
FIT_LIMIT_DEG = 3.0
MIN_SPREAD_DEG = 4.0


def ease(frm, to, steps=EASE_STEPS):
    '''The (roll, pitch) steps from one tilt to another, the last one `to`.'''
    return [tuple(f + (t - f) * k / steps for f, t in zip(frm, to)) for k in range(1, steps + 1)]


class Mount:
    '''A calibrated IMU: `rotation` takes the chip's axes to the chassis's
    (3x3, chassis <- chip); `gyro_bias` is its gyro's reading at rest (rad/s,
    chip axes); `gravity` what its accelerometer reads for 1 g (m/s^2); `fit`
    how far the calibration readings strayed from it (deg RMS).'''

    def __init__(self, rotation, gyro_bias=(0, 0, 0), gravity=STANDARD_G, fit=0.0, address=0x68):
        self.rotation = np.array(rotation, float)
        self.gyro_bias = np.array(gyro_bias, float)
        self.gravity = float(gravity)
        self.fit = float(fit)
        self.address = int(address)

    def to_dict(self):
        return {'rotation': self.rotation.round(6).tolist(), 'gyro_bias': self.gyro_bias.round(6).tolist(),
                'gravity': round(self.gravity, 4), 'fit_deg': round(self.fit, 3), 'address': self.address}

    @classmethod
    def from_dict(cls, d):
        return cls(d['rotation'], d['gyro_bias'], d['gravity'], d.get('fit_deg', 0.0), d.get('address', 0x68))

    def save(self, path):
        path.write_text(json.dumps(self.to_dict(), indent=2) + '\n', encoding='utf-8')

    @classmethod
    def load(cls, path):
        '''The mount saved at path, or None if there's none (or it's unreadable).'''
        try:
            return cls.from_dict(json.loads(path.read_text(encoding='utf-8')))
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def describe(self):
        '''Where the chip's axes point, in words: "X forward, Y left, Z up" - or
        the nearest of those and how far off it is.'''
        names = ('forward', 'left', 'up')
        words = []
        for axis, label in enumerate('XYZ'):
            v = self.rotation[:, axis]
            k = int(np.argmax(np.abs(v)))
            words.append(f"{label} {'' if v[k] > 0 else '-'}{names[k]}".replace('-forward', 'back')
                         .replace('-left', 'right').replace('-up', 'down'))
        snapped = np.zeros((3, 3))
        for axis in range(3):
            k = int(np.argmax(np.abs(self.rotation[:, axis])))
            snapped[k, axis] = np.sign(self.rotation[k, axis])
        off = rotation_angle(snapped.T @ self.rotation)
        return ', '.join(words) + ('' if off < 5 else f' (tilted {off:.0f} deg from that)')


def rotation_angle(R):
    '''How far a rotation turns, deg.'''
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def reading(snapshot, address=None):
    '''The MPU6050's reading in a SensorSnapshot - (t seconds on the robot's
    clock, accel, gyro) - or None if it has none (or none at `address`).'''
    if snapshot is None:
        return None
    for sensor in snapshot.sensors:
        if sensor.kind == KIND and len(sensor.values) == VALUES and address in (None, sensor.address):
            v = np.array(sensor.values[:6], float)
            if np.isfinite(v).all():
                return snapshot.t_ms / 1000, v[:3], v[3:]
    return None


def fit_mount(poses, address=0x68):
    '''The IMU's mounting from readings taken standing still in a few poses:
    each (accel, gyro, up) - the chip's mean readings, and which way up the
    chassis is then (chassis frame, unit: Q8Sim.floor_up). Raises ValueError,
    saying why, if they can't pin it down.'''
    accel = np.array([p[0] for p in poses], float)
    gyro = np.array([p[1] for p in poses], float)
    up = np.array([p[2] for p in poses], float)
    if len(poses) < 3:
        raise ValueError('needs at least three poses')
    # The chassis's ups must differ two ways (around their mean) for the
    # chip's turn about up to show
    spread = np.linalg.svd(up - up.mean(0), compute_uv=False)
    if math.degrees(math.asin(min(1.0, spread[1] / math.sqrt(len(poses))))) < MIN_SPREAD_DEG / 2:
        raise ValueError("the robot didn't tilt enough two ways - is it on the floor, on all four feet?")
    seen = accel / np.linalg.norm(accel, axis=1, keepdims=True)
    # Kabsch: the rotation taking the chip's ups onto the chassis's
    U, _, Vt = np.linalg.svd(seen.T @ up)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    errors = np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', seen @ R.T, up), -1, 1)))
    fit = float(np.sqrt((errors ** 2).mean()))
    if fit > FIT_LIMIT_DEG:
        raise ValueError(f'the readings fit no one mounting (off by {fit:.1f} deg) - '
                         'was the robot moving, or is the sensor loose?')
    return Mount(R, gyro.mean(0), np.linalg.norm(accel, axis=1).mean(), fit, address)


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw_quat(deg):
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0, 0, math.sin(a)])


class Attitude:
    '''The chassis's orientation from IMU readings: the gyro turns it, and
    gravity, wherever the accelerometer can see it clearly, keeps its tilt
    honest (a Mahony filter). Its heading has nothing to hold it but the gyro,
    so it starts where it's told and drifts slowly.'''

    KP = 1.5                        # rad/s per rad: how hard gravity pulls the tilt back...
    TRUST = 0.25                    # ...less the further a reading is from 1 g - none beyond this fraction
    MAX_GAP = 0.5                   # s: after a longer silence the tilt starts over from gravity
    STILL_RATE = 0.02               # rad/s: turning slower than this all along...
    STILL_TILT = 0.5                # ...with gravity staying within this (deg)...
    STILL_S = 0.4                   # ...for this long (s), it's standing still
    BIAS_RATE = 0.02                # how far each still reading then pulls the gyro offset estimate

    def __init__(self, mount, heading=0.0):
        self.mount = mount
        self.bias = mount.gyro_bias.copy()
        self._heading = heading
        self._q = None
        self._t = self._w = None
        self._still = None                  # (since when, gravity then) while it may be standing still

    def update(self, t, accel, gyro):
        '''One reading: t the robot's clock (s), accel and gyro on the chip's
        axes (m/s^2, rad/s).'''
        R_cs = self.mount.rotation
        accel, gyro = np.asarray(accel, float), np.asarray(gyro, float)
        a = R_cs @ accel
        w = R_cs @ (gyro - self.bias)
        g = np.linalg.norm(a)
        seen = a / g if g > 1e-6 else None
        if self._q is None or self._t is None or not 0 < t - self._t <= self.MAX_GAP:
            if abs(g / self.mount.gravity - 1) > 0.05:
                return                      # mid-jolt: start from a reading that's gravity alone
            self._start(seen)
        else:
            dt = t - self._t
            # The pull: from the up the estimate expects towards the up the
            # accelerometer sees, while what it sees is mostly gravity
            trust = max(0.0, 1 - abs(g / self.mount.gravity - 1) / self.TRUST)
            pull = seen * trust if seen is not None else np.zeros(3)
            up = _quat_to_mat(self._q)[2]               # world up, in the chassis frame
            rate = (w + self._w) / 2 + self.KP * np.cross(pull, up)
            angle = np.linalg.norm(rate) * dt
            if angle > 0:
                axis = rate / np.linalg.norm(rate)
                self._q = _quat_mul(self._q, np.r_[math.cos(angle / 2), math.sin(angle / 2) * axis])
                self._q /= np.linalg.norm(self._q)
            # Standing still: what the gyro reads is its offset
            if abs(g / self.mount.gravity - 1) < 0.03 and np.linalg.norm(gyro - self.bias) < self.STILL_RATE:
                if (self._still is None
                        or math.degrees(math.acos(min(1.0, float(seen @ self._still[1])))) > self.STILL_TILT):
                    self._still = (t, seen)
                elif t - self._still[0] >= self.STILL_S:
                    self.bias += self.BIAS_RATE * (gyro - self.bias)
            else:
                self._still = None
        self._t, self._w = t, w

    def _start(self, seen):
        '''Level from gravity alone, at the heading kept.'''
        heading = self.heading() if self._q is not None else self._heading
        if seen is None:
            seen = np.array([0.0, 0.0, 1.0])
        roll = math.atan2(seen[1], seen[2])
        pitch = math.atan2(-seen[0], math.hypot(seen[1], seen[2]))
        q_roll = np.array([math.cos(roll / 2), math.sin(roll / 2), 0, 0])
        q_pitch = np.array([math.cos(pitch / 2), 0, math.sin(pitch / 2), 0])
        self._q = _quat_mul(_yaw_quat(heading), _quat_mul(q_pitch, q_roll))

    @property
    def ready(self):
        return self._q is not None

    def orientation(self):
        '''The chassis's orientation, 3x3, world <- chassis.'''
        return _quat_to_mat(self._q)

    def heading(self):
        '''Which way the chassis points, deg anticlockwise seen from above.'''
        R = self.orientation()
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def set_heading(self, deg):
        '''Turn it about the vertical to point `deg` - as Reset does.'''
        if self._q is None:
            self._heading = deg
            return
        self._q = _quat_mul(_yaw_quat(deg - self.heading()), self._q)


class SimMPU6050:
    '''An MPU6050 on the model's chassis, as the robot's firmware reads one:
    accelerometer (specific force, m/s^2: up reads +1 g at rest) and gyro
    (rad/s) on the chip's own axes, `rotation` (3x3, chassis <- chip) from the
    chassis's, after the 44 Hz low-pass the firmware sets. A real chip's
    faults can be added - a scale error, offsets, noise; the clipping at the
    firmware's +/-8 g and +/-500 deg/s is always there. Its temperature isn't
    modelled: it reads NaN, as a sensor that isn't answering does.

    Call step() after every physics step; values() is a reading now.'''

    BANDWIDTH_HZ = 44
    ACCEL_RANGE = 8 * STANDARD_G
    GYRO_RANGE = math.radians(500)
    # Datasheet noise densities (400 ug/rtHz, 0.005 dps/rtHz) over the
    # low-pass's noise bandwidth, as an RMS
    ACCEL_NOISE = 400e-6 * STANDARD_G * math.sqrt(math.pi / 2 * 44)
    GYRO_NOISE = math.radians(0.005) * math.sqrt(math.pi / 2 * 44)

    def __init__(self, sim, rotation=np.eye(3), accel_scale=1.0, accel_bias=(0, 0, 0), gyro_bias=(0, 0, 0),
                 noise=False, address=0x68, seed=0):
        self.sim = sim
        self.rotation = np.array(rotation, float)
        self.accel_scale = accel_scale
        self.accel_bias = np.array(accel_bias, float)
        self.gyro_bias = np.array(gyro_bias, float)
        self.noise = noise
        self.address = address
        self._rng = np.random.default_rng(seed)
        dt = sim.model.opt.timestep
        self._alpha = dt / (dt + 1 / (2 * math.pi * self.BANDWIDTH_HZ))
        self._accel = self._gyro = None

    def step(self):
        d, s = self.sim.data, self.sim
        to_chip = self.rotation.T
        accel = to_chip @ d.sensordata[s.imu_accel:s.imu_accel + 3]
        gyro = to_chip @ d.sensordata[s.imu_gyro:s.imu_gyro + 3]
        if self._accel is None:
            self._accel, self._gyro = accel, gyro
        else:
            self._accel = self._accel + self._alpha * (accel - self._accel)
            self._gyro = self._gyro + self._alpha * (gyro - self._gyro)

    def values(self):
        '''The seven values the robot sends: accel x y z, gyro x y z, temperature.'''
        if self._accel is None:
            self.step()
        accel = self._accel * self.accel_scale + self.accel_bias
        gyro = self._gyro + self.gyro_bias
        if self.noise:
            accel = accel + self._rng.normal(0, self.ACCEL_NOISE, 3)
            gyro = gyro + self._rng.normal(0, self.GYRO_NOISE, 3)
        accel = np.clip(accel, -self.ACCEL_RANGE, self.ACCEL_RANGE)
        gyro = np.clip(gyro, -self.GYRO_RANGE, self.GYRO_RANGE)
        return [*accel, *gyro, math.nan]
