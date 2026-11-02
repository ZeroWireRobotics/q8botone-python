'''
The robot's IMU - an MPU6050 on its Qwiic port - in the sim.

Calibrating it and tracking the chassis's orientation from it is the q8bot
library's (q8bot.imu: Mount, fit_mount, Attitude and the rest, imported here
under the same names), so the desktop app and the library do it the same way
with or without the sim. What's the sim's own is SimMPU6050: one on the
model, for the sim's robot to answer with - a real chip's faults included,
if asked.
'''

import math

import numpy as np
from q8bot.imu import (CALIBRATION_TILTS, EASE_STEP_S, EASE_STEPS, FIT_LIMIT_DEG, GATHER_S, KIND,  # noqa: F401
                       MIN_SPREAD_DEG, SETTLE_S, STANDARD_G, VALUES, Attitude, Leveller, Mount, ease,
                       fit_mount, floor_up, reading, roll_pitch, rotation_angle)


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
