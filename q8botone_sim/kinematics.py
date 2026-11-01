'''
Leg and body kinematics, in the q8bot library's conventions.

A leg's frame is the library's (k_solver's): X forward, Y down, the rear
servo's shaft at the origin and the front servo's at (d, 0). Servo angles are
degrees below forward. All four legs are the same in their own frame.

The lengths are parameters rather than the library's built-in ones - the
model's are measured from the CAD (shafts 20 mm apart, links 24 and 39 mm).
'''

import math
from dataclasses import dataclass

import numpy as np


def _below(vx, vy):
    '''A direction in the leg's plane as degrees below forward (Y is down).'''
    return math.degrees(math.atan2(vy, vx))


@dataclass(frozen=True)
class Leg:
    d: float                                # mm between the two servo shafts
    l1: float                               # upper link (E), shaft to knee
    l2: float                               # lower links (C, D), knee to foot
    foot_angle: tuple = (10.0, 170.0)       # allowed angle between the lower links at the foot, deg

    def knees(self, q_front, q_rear):
        f, r = math.radians(q_front), math.radians(q_rear)
        return ((self.d + self.l1 * math.cos(f), self.l1 * math.sin(f)),
                (self.l1 * math.cos(r), self.l1 * math.sin(r)))

    def angle_at_foot(self, q_front, q_rear, foot):
        '''The angle between the lower links at the foot: D's direction (from
        the front knee) minus C's (from the rear knee), 0-360 deg. Between 0
        and 180 the leg is the right way round; past 180 it's inside out.'''
        (fx, fy), (rx, ry) = self.knees(q_front, q_rear)
        return (_below(foot[0] - fx, foot[1] - fy) - _below(foot[0] - rx, foot[1] - ry)) % 360

    def fk(self, q_front, q_rear):
        '''The foot (x, y) for two servo angles, or None where the lower links
        can't meet. Of the two places they could, the one with the leg the
        right way round.'''
        (fx, fy), (rx, ry) = self.knees(q_front, q_rear)
        vx, vy = fx - rx, fy - ry
        dist = math.hypot(vx, vy)
        if not 0 < dist <= 2 * self.l2:
            return None
        h = math.sqrt(self.l2 ** 2 - (dist / 2) ** 2)
        mx, my = rx + vx / 2, ry + vy / 2
        for sign in (-1, 1):
            foot = (mx + sign * h * vy / dist, my - sign * h * vx / dist)
            if self.angle_at_foot(q_front, q_rear, foot) <= 180:
                return foot
        return foot

    def ik(self, x, y):
        '''Servo angles (front, rear) that put the foot at (x, y), or None if
        it's out of the leg's reach - too near, too far, or the lower links
        closer than `foot_angle` allows to folding flat.'''
        rf, rr = math.hypot(x - self.d, y), math.hypot(x, y)
        try:
            bf = math.acos((self.l1 ** 2 + rf ** 2 - self.l2 ** 2) / (2 * self.l1 * rf))
            br = math.acos((self.l1 ** 2 + rr ** 2 - self.l2 ** 2) / (2 * self.l1 * rr))
        except (ValueError, ZeroDivisionError):
            return None
        # The front knee sits ahead of the line to the foot, the rear knee behind it.
        q = (_below(x - self.d, y) - math.degrees(bf), _below(x, y) + math.degrees(br))
        if not self.foot_angle[0] < self.angle_at_foot(*q, (x, y)) < self.foot_angle[1]:
            return None
        return q


def rotation(roll, pitch):
    '''The chassis's orientation for a roll and pitch (deg): roll + = right
    side down, pitch + = nose up; pitch applied after roll.'''
    r, p = math.radians(roll), math.radians(-pitch)       # + pitch is nose up: a negative turn about +Y
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return ry @ rx


def roll_pitch(R):
    '''The roll and pitch (deg, as rotation() takes them) of an orientation.'''
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0])))))


def move_body(feet, forward=0.0, up=0.0, roll=0.0, pitch=0.0):
    '''Where planted feet are in the chassis's frame after the chassis moves.

    feet: {leg: (x, y, z)} in the chassis frame (mm; X forward, Y left, Z up)
    before the move. The chassis then shifts `forward` and `up` (mm) and
    turns by `roll` and `pitch` (deg, as rotation()) about its origin, while
    the feet stay where they are. Returns {leg: (x, y, z)} in the moved
    chassis's frame.'''
    R, t = rotation(roll, pitch), np.array([forward, 0.0, up])
    return {leg: R.T @ (np.asarray(p, float) - t) for leg, p in feet.items()}
