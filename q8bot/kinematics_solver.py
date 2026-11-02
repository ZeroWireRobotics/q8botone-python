'''
Written by yufeng.wu0902@gmail.com

Python class for FK/IK calculations on Q8bot.
'''

import math

# Q8botOne's geometry, from its CAD (confirmed to match the robot's
# linkages), mm - the one place it's set: robot.py, the gait tables exported
# for the firmware (export_gait_tables.py) and the IMU's calibration
# (imu.py) all take it from here. Until 2026-11 these were older round
# numbers (19.5 / 25 / 40, track 80, wheelbase 78), which stood the robot
# 1-2 mm lower than asked and leaned it ~15% further than a roll called.
CENTER_DIST = 20.0  # between each leg's two servo shafts
L1 = 24.0           # upper links, shaft to knee
L2 = 39.0           # lower links, knee to foot
# Between the feet, for body tilt (Q8Bot.set_stance()): side to side, which a
# roll tilts across, and front to back - each leg's shafts are 80 mm from the
# other end's - which a pitch tilts along
TRACK_MM = 68.6
WHEELBASE_MM = 80.0


# d - distance between motors; l1/l1p - upper linkage length; l2/l2p - lower linkage length
# Unit is in mm.
class k_solver:
    def __init__(self, d = CENTER_DIST, l1 = L1, l2 = L2, l1p = L1, l2p = L2):
        self.d = d
        self.l1 = l1
        self.l2 = l2
        self.l1p = l1p
        self.l2p = l2p
        self.prev_ik = [45, 135]
        self.prev_est = [self.d/2, (self.l1 + self.l2)]

    # Check whether a solution exist. To be completed later.
    def ik_check(self, x, y):
        if y < 0:
            return False
        return True

    # Solve inverse kinematics at an given end effector position
    def ik_solve(self, x, y, deg = True, rounding = 3):
        try:
            c1 = math.sqrt((x - self.d)**2 + y**2)
            c2 = math.sqrt(x**2 + y**2)
            a1 = math.acos((c1**2 + self.d**2 - c2**2) / (2*c1*self.d))
            a2 = math.acos((c2**2 + self.d**2 - c1**2) / (2*c2*self.d))
            b1 = math.acos((c1**2 + self.l1**2 - self.l2**2) / (2*c1*self.l1))
            b2 = math.acos((c2**2 + self.l1p**2 - self.l2p**2) / (2*c2*self.l1p))
            q1 = math.pi - a1 - b1
            q2 = a2 + b2
            if deg:
                q1, q2 = q1*180/math.pi, q2*180/math.pi
            self.prev_ik = [q1, q2]
            return round(q1, rounding), round(q2, rounding), True
        except:
            return self.prev_ik[0], self.prev_ik[1], False
    
    # Check whether a solution exist. To be completed later.
    def fk_check(self):
        return True
    
    # Solve inverse kinematics at an given end effector position
    def fk_solve(self, q1, q2, deg = True, rounding = 3):
        # scipy is imported here rather than at module scope so it stays an
        # optional dependency - this is the only thing in the library that
        # needs it, and nothing on the control path calls it.
        from scipy.optimize import fsolve

        if deg:
            angles = (self._deg2rad(q1), self._deg2rad(q2))
        x, y = fsolve(self._fk_calc, [10,60], args=angles)
        # x, y = fsolve(self._fk_calc, self.prev_est, args=angles)
        # self.prev_est = [x, y]
        return round(x, rounding), round(y, rounding)

    #-------------------#
    # Private Functions #
    #-------------------#
    def _fk_calc(self, x, *angle):
        q1, q2 = angle
        Xa = self.l1 * math.cos(q1) + self.d
        Ya = self.l1 * math.sin(q1)
        Xb = self.l1p * math.cos(q2)
        Yb = self.l1p * math.sin(q2)
        return [x[0]*x[0] - 2*x[0]*Xa + Xa**2 + x[1]*x[1] - 2*x[1]*Ya + Ya**2 - self.l2**2,
                x[0]*x[0] - 2*x[0]*Xb + Xb**2 + x[1]*x[1] - 2*x[1]*Yb + Yb**2 - self.l2p**2]
    
    def _rad2deg(self, ang_rad):
        return ang_rad*180/math.pi
    
    def _deg2rad(self, ang_deg):
        return ang_deg*math.pi/180
