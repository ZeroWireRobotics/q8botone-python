'''
The Q8botOne model, loaded and given the handles a controller needs: servo
targets in the library's degrees, where each foot is, the chassis's pose, the
test stand, and the servos' torque on or off.

    sim = Q8Sim()
    sim.set_targets({'FL': (30, 150)})      # front servo, rear servo
    sim.step(1000)                          # 1 s
    sim.foot('FL')                          # (x, y) in the leg's frame, mm

Lengths are mm and angles degrees throughout, as the library's; the frames
are those of kinematics.py. The model itself stays SI.
'''

import math
from pathlib import Path

import mujoco
import numpy as np

from .kinematics import Leg, roll_pitch

MODEL = Path(__file__).resolve().parent.parent / 'model' / 'q8botone.xml'
LEGS = ('FL', 'FR', 'RL', 'RR')
SERVOS = ('front', 'rear')
# The eight servos in the order the library and the robot list them: each
# leg's front servo then its rear, legs FL FR RL RR (the library's FL FR BL BR).
WIRE = tuple((leg, s) for leg in LEGS for s in SERVOS)
# show_pose with an orientation: a foot within this of the lowest one is on the
# floor too (mm) - wide enough for the orientation being a degree or two out
FOOT_DOWN_MM = 5.0


class Q8Sim:
    def __init__(self, path=MODEL):
        m = self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(m)
        # The leg's geometry, as the model has it
        lo, hi = np.degrees(m.tendon_range[m.tendon('FL_foot_angle').id])
        self.leg = Leg(d=(m.body('FL_front_arm').pos[0] - m.body('FL_rear_arm').pos[0]) * 1000,
                       l1=m.body('FL_front_lower').pos[0] * 1000,
                       l2=m.site('FL_front_foot').pos[0] * 1000,
                       foot_angle=(float(lo), float(hi)))
        # Each leg frame's origin - its rear servo's shaft - in the chassis frame
        self.shaft = {leg: m.body(f'{leg}_rear_arm').pos * 1000 for leg in LEGS}
        self._act = {(leg, s): m.actuator(f'{leg}_{s}').id for leg in LEGS for s in SERVOS}
        self._qpos = {(leg, s): m.jnt_qposadr[m.joint(f'{leg}_{s}').id] for leg in LEGS for s in SERVOS}
        self._knee = {(leg, s): m.jnt_qposadr[m.joint(f'{leg}_{s}_knee').id] for leg in LEGS for s in SERVOS}
        self._wire_act = [self._act[k] for k in WIRE]
        self._wire_qpos = [self._qpos[k] for k in WIRE]
        self._gains = m.actuator_gainprm.copy(), m.actuator_biasprm.copy()
        self._stall = m.actuator_forcerange[:, 1].copy()
        self._key = m.key('idle').id
        self.idle = {leg: tuple(math.degrees(m.key_ctrl[self._key][self._act[leg, s]]) for s in SERVOS)
                     for leg in LEGS}
        # The test stand: a weld to a body that doesn't move
        self._weld = m.equality('stand').id
        stand = m.body('stand').id
        self._mocap = m.body_mocapid[stand]
        self._stand_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == stand]
        self.stand_height = float(m.body('stand').pos[2])
        self._feet = {leg: m.geom(f'{leg}_foot').id for leg in LEGS}
        self._box = m.geom('chassis_box').id
        self._floor = m.geom('floor').id
        self._chassis = m.body('chassis').id
        # The IMU board drawn on top (show_imu), and the IMU's sensors
        self._imu_body = m.body('imu_board').id
        boards = {self._imu_body, m.body('imu_pcb').id}
        self._imu_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in boards]
        self._chassis_top = float(m.geom_pos[self._box][2] + m.geom_size[self._box][2]) * 1000     # mm
        self.imu_accel = m.sensor_adr[m.sensor('imu_accel').id]     # sensordata index: x, y, z follow
        self.imu_gyro = m.sensor_adr[m.sensor('imu_gyro').id]
        self._planted = {}                  # show_pose: feet on the floor last time, {leg: world xy}
        self.on_stand = False
        self.torque_on = True
        self.stiffness = 1.0
        self.reset()
        # Each foot's plane across the robot: its foot points' y in the chassis
        # frame - fixed by the build, the same in every pose.
        self._foot_y = {leg: float(self._in_chassis(self.data.site(f'{leg}_front_foot').xpos)[1]) for leg in LEGS}

    # ---- running ----------------------------------------------------------

    def reset(self):
        '''Back to the idle keyframe - on the floor, or on the stand if it's in use.'''
        mujoco.mj_resetDataKeyframe(self.model, self.data, self._key)
        mujoco.mj_forward(self.model, self.data)
        self._planted = {}
        if self.on_stand:
            self.set_on_stand(True)

    def recentre(self):
        '''Back in the middle of the floor, facing forward and upright - set
        down level on the floor, or on the stand if it's in use - with the
        legs as they are, still holding their targets.'''
        targets, angles = self.all_targets(), self.all_angles()
        mujoco.mj_resetDataKeyframe(self.model, self.data, self._key)
        self.show_pose(angles)
        self.set_all(targets)
        self.set_on_stand(self.on_stand)

    def step(self, n=1):
        mujoco.mj_step(self.model, self.data, nstep=n)

    @property
    def time(self):
        return self.data.time

    # ---- servos -----------------------------------------------------------

    def set_targets(self, angles):
        '''Servo targets, {leg: (front, rear)} in degrees below forward; legs
        left out keep theirs.'''
        for leg, pair in angles.items():
            for s, a in zip(SERVOS, pair):
                self.data.ctrl[self._act[leg, s]] = math.radians(a)

    def targets(self):
        return {leg: tuple(math.degrees(self.data.ctrl[self._act[leg, s]]) for s in SERVOS) for leg in LEGS}

    def angles(self):
        '''Where the servos actually are, {leg: (front, rear)} in degrees.'''
        return {leg: tuple(math.degrees(self.data.qpos[self._qpos[leg, s]]) for s in SERVOS) for leg in LEGS}

    def set_all(self, degs):
        '''All eight servo targets at once, in WIRE order, degrees.'''
        self.data.ctrl[self._wire_act] = np.radians(degs)

    def all_targets(self):
        return np.degrees(self.data.ctrl[self._wire_act])

    def all_angles(self):
        '''All eight servos' angles, in WIRE order, degrees.'''
        return np.degrees(self.data.qpos[self._wire_qpos])

    def effort(self):
        '''Each servo's torque as a fraction of its stall torque, {leg: (front, rear)}.'''
        f = self.data.actuator_force
        return {leg: tuple(abs(f[self._act[leg, s]]) / self._stall[self._act[leg, s]] for s in SERVOS)
                for leg in LEGS}

    def set_torque(self, on):
        '''Servos holding their targets, or limp.'''
        self.torque_on = on
        self._apply_gains()

    def set_stiffness(self, scale):
        '''The servos' position gain, as a multiple of the model's: the robot
        doubles it for a jump's push and drops it for damping.'''
        self.stiffness = scale
        self._apply_gains()

    def _apply_gains(self):
        m = self.model
        m.actuator_gainprm[:], m.actuator_biasprm[:] = self._gains
        if not self.torque_on:
            m.actuator_gainprm[:, 0] = 0
            m.actuator_biasprm[:, :3] = 0
        else:                                           # a position actuator's bias is (0, -kp, -kv)
            m.actuator_gainprm[:, 0] *= self.stiffness
            m.actuator_biasprm[:, 1] *= self.stiffness

    # ---- frames -----------------------------------------------------------

    def leg_to_chassis(self, leg, x, y):
        '''A point (x, y) in a leg's frame, in the chassis frame (mm) - on the
        plane the foot moves in.'''
        o = self.shaft[leg]
        return np.array([o[0] + x, self._foot_y[leg], o[2] - y])

    def chassis_to_leg(self, leg, p):
        o = self.shaft[leg]
        return float(p[0] - o[0]), float(o[2] - p[2])

    def _in_chassis(self, world):
        '''A world point (m) in the chassis frame (mm).'''
        d = self.data
        R = d.xmat[self._chassis].reshape(3, 3)
        return R.T @ (np.asarray(world) - d.xpos[self._chassis]) * 1000

    # ---- what the robot is doing ------------------------------------------

    def foot(self, leg):
        '''Where the foot actually is, (x, y) in its leg's frame - midway
        between its two foot points, which the leg's loop holds together.'''
        d = self.data
        p = (d.site(f'{leg}_front_foot').xpos + d.site(f'{leg}_rear_foot').xpos) / 2
        return self.chassis_to_leg(leg, self._in_chassis(p))

    def chassis(self):
        '''The chassis's pose: x, y, z (mm; z is the servo shafts' height above
        the floor), roll and pitch (deg, as kinematics.rotation).'''
        d = self.data
        x, y, z = d.xpos[self._chassis] * 1000
        roll, pitch = roll_pitch(d.xmat[self._chassis].reshape(3, 3))
        return x, y, z, roll, pitch

    def heading(self):
        '''Which way the chassis points: degrees anticlockwise from the world's
        x axis, seen from above.'''
        R = self.data.xmat[self._chassis].reshape(3, 3)
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def feet_down(self):
        '''The legs whose foot touches the floor.'''
        d, feet = self.data, {g: leg for leg, g in self._feet.items()}
        down = set()
        for c in d.contact[:d.ncon]:
            pair = set(c.geom)
            if self._floor in pair:
                down |= {feet[g] for g in pair if g in feet}
        return [leg for leg in LEGS if leg in down]

    def loop_gap(self):
        '''The widest any leg's loop is pulled open: the distance between its
        two foot points, mm.'''
        d = self.data
        return max(np.linalg.norm(d.site(f'{leg}_front_foot').xpos - d.site(f'{leg}_rear_foot').xpos)
                   for leg in LEGS) * 1000

    # ---- posed, not simulated ----------------------------------------------

    def floor_up(self, angles):
        '''Which way is up, in the chassis frame, with the robot's four feet on
        a level floor and its servos at `angles` (WIRE order, degrees): the
        plane through the feet, from the legs' geometry alone. A unit vector;
        None if a leg can't close its loop at those angles.'''
        feet = []
        for i, leg in enumerate(LEGS):
            foot = self.leg.fk(angles[2 * i], angles[2 * i + 1])
            if foot is None:
                return None
            feet.append(self.leg_to_chassis(leg, *foot))
        feet = np.array(feet)
        up = np.linalg.svd(feet - feet.mean(0))[2][2]       # the plane's normal: its least spread
        return up * np.sign(up[2])

    def show_pose(self, angles, orientation=None):
        '''The model put in a pose at once, without physics - a picture of the
        robot from its eight servo angles (WIRE order, degrees), as measured on
        the real one. Each leg's loop is closed by its own geometry. The servo
        targets follow, so physics can carry on from here.

        The chassis stays where it is (on the test stand, say) - or, given its
        orientation (3x3, world <- chassis, as an IMU tracks it), is turned so
        and set down on the floor on whatever is lowest. Feet on the floor
        stay where they were the last time, so the model walks as the robot
        does.'''
        m, d = self.model, self.data
        a = np.asarray(angles, float)
        for i, leg in enumerate(LEGS):
            q = a[2 * i], a[2 * i + 1]
            for s, angle in zip(SERVOS, q):
                d.qpos[self._qpos[leg, s]] = math.radians(angle)
            foot = self.leg.fk(*q)
            if foot is None:
                continue                    # no way to close this loop: its knees stay as they were
            for s, angle, (kx, ky) in zip(SERVOS, q, self.leg.knees(*q)):
                link = math.degrees(math.atan2(foot[1] - ky, foot[0] - kx))    # the lower link, deg below forward
                d.qpos[self._knee[leg, s]] = math.radians((link - angle + 180) % 360 - 180)
        d.qvel[:] = 0
        d.ctrl[self._wire_act] = np.radians(a)
        if orientation is not None and not self.on_stand:
            self._place(np.asarray(orientation, float))
        mujoco.mj_forward(m, d)

    def _place(self, R):
        '''The chassis turned to R and set down: its lowest point on the floor,
        and moved over the floor by the feet that were down last time and
        still are - they don't slide.'''
        m, d = self.model, self.data
        mujoco.mju_mat2Quat(d.qpos[3:7], R.flatten())
        mujoco.mj_kinematics(m, d)
        d.qpos[2] -= self._lowest()
        mujoco.mj_kinematics(m, d)
        r = m.geom_size[self._feet['FL']][0]
        bottom = {leg: d.geom_xpos[g][2] - r for leg, g in self._feet.items()}
        lowest = min(bottom.values())
        down = [leg for leg in LEGS if bottom[leg] - lowest < FOOT_DOWN_MM / 1000
                and bottom[leg] < FOOT_DOWN_MM / 1000]                 # (none, lying on its back)
        where = {leg: d.geom_xpos[self._feet[leg]][:2].copy() for leg in down}
        held = [leg for leg in down if leg in self._planted]
        if held:
            shift = np.mean([self._planted[leg] - where[leg] for leg in held], axis=0)
            d.qpos[:2] += shift
            where = {leg: p + shift for leg, p in where.items()}
        self._planted = where

    def show_imu(self, rotation=None):
        '''The IMU board drawn with its axes turned as `rotation` has them
        (3x3, chassis <- chip: the calibrated mounting) - or hidden, for None.
        Where on the robot it sits isn't known, so it's drawn on top of the
        chassis, its lowest corner just clear.'''
        m = self.model
        if rotation is None:
            m.geom_rgba[self._imu_geoms, 3] = 0.0
            return
        R = np.asarray(rotation, float)
        corners = np.array([[x, y, z] for x in (-8.89, 8.89) for y in (-12.7, 12.7) for z in (-1.6, 2.95)])
        m.body_pos[self._imu_body] = [0, 0, (self._chassis_top - (corners @ R.T)[:, 2].min() + 1.0) / 1000]
        mujoco.mju_mat2Quat(m.body_quat[self._imu_body], R.flatten())
        m.geom_rgba[self._imu_geoms, 3] = 1.0
        mujoco.mj_forward(m, self.data)

    # ---- the test stand ---------------------------------------------------

    def set_on_stand(self, on):
        '''Onto the test stand - held still with the shafts stand_height up,
        feet in the air - or off it, set down level on the floor. Either way
        it keeps its place, heading and leg pose.'''
        m, d = self.model, self.data
        R = d.xmat[self._chassis].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        level = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[3:7] = level
        d.qvel[:6] = 0
        if on:
            d.mocap_pos[self._mocap] = [d.qpos[0], d.qpos[1], self.stand_height]
            d.mocap_quat[self._mocap] = level
            d.qpos[:3] = d.mocap_pos[self._mocap]
        d.eq_active[self._weld] = on
        m.geom_rgba[self._stand_geoms, 3] = 1.0 if on else 0.0
        mujoco.mj_forward(m, d)
        if not on:
            d.qpos[2] -= self._lowest() - 0.0002            # lowest point 0.2 mm above the floor
            mujoco.mj_forward(m, d)
        self._planted = {}
        self.on_stand = on

    def _lowest(self):
        '''The height (m) of the lowest thing that touches the floor: a foot or
        a corner of the chassis's contact box.'''
        m, d = self.model, self.data
        feet = [d.geom_xpos[g][2] - m.geom_size[g][0] for g in self._feet.values()]
        b = self._box
        corners = d.geom_xpos[b] + (np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
                                    * m.geom_size[b]) @ d.geom_xmat[b].reshape(3, 3).T
        return min(min(feet), corners[:, 2].min())
