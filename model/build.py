'''
Build the MuJoCo model of Q8botOne - model/q8botone.xml and model/meshes/ -
from the simplified sim CAD's STL exports and its measured mass properties:

    python model/build.py [--cad ../q8botone-hardware/cad/sim]

The robot's shape all comes from the CAD: the servo shafts and each link's
pivots are found in the meshes (circles fitted to the shaft discs and pivot
holes), so a changed CAD needs no edits here - export, then rebuild. The
chassis's mass properties come from inertials.yaml (measured, from the full
assembly). Each link is one printed material, so its centre of mass and
inertia follow from its mesh and its measured mass.

Frames: the CAD's assembly frame is the chassis's - X forward, Y left, Z up,
the servo shafts on Z = 0. Each link's body frame has its origin on its
proximal pivot (the servo shaft for an upper link, the knee for a lower one),
X along the link to its distal pivot and Y along the hinge axis, with Y = 0 on
the link's inner face. A joint angle is the library's: degrees below forward.
'''

import argparse
import math
import struct
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_CAD = HERE.parent.parent / 'q8botone-hardware' / 'cad' / 'sim'

# ---- Servo model: DYNAMIXEL XL330-M077-T on 6 V ---------------------------
# Stall torque is from ROBOTIS's e-manual; the rest isn't published and is a
# starting point to tune against the real robot.
STALL_TORQUE = 0.228      # N m
KP = 1.0                  # N m / rad - position gain
KV = 0.01                 # N m s / rad - damping on the error's rate
ARMATURE = 1e-5           # kg m^2 - motor + gearbox inertia seen at the shaft
FOOT_FRICTION = 1.0

MESH_TRIANGLES = {'chassis': 40000, 'link': 4000}   # display meshes' size

LEGS = {                  # name: (side, front or rear pair of shafts)
    'FL': (+1, +1), 'FR': (-1, +1), 'RL': (+1, -1), 'RR': (-1, -1),
}
IDLE = (30.0, 150.0)      # firmware's idle pose: front servo, rear servo (deg below forward)
# Angle between a leg's lower links at the foot (see the <tendon> comment):
# 30 deg with both arms straight down or up (the jump), ~104 on the idle pose.
FOOT_ANGLE_RANGE = (10.0, 170.0)
# The test stand holds the servo shafts this high (m): above any foot's reach.
STAND_HEIGHT = 0.11

# ---- The IMU board: Adafruit's MPU-6050 6-DoF breakout (product 3886) ------
# Drawn on the chassis while the digital twin follows a calibrated MPU6050.
# Its layout is from Adafruit's open-source PCB design (CC BY-SA; designed by
# Bryan Siepert for Adafruit Industries, github.com/adafruit/Adafruit-MPU6050-PCB).
# Board coordinates, mm from its centre - which is the chip's: x along the
# long side towards the STEMMA QT socket on the right (with the "MPU-6050
# 6-DoF" pin row at the top), y towards that pin row, z out of the top. The
# chip is turned on the board so its own axes are x down and y right (as
# the silkscreen marks them), so the board's x is the chip's Y and its y the
# chip's -X.
BOARD = (25.4, 17.78, 1.6)          # outline and thickness, mm
BOARD_CORNER_R = 2.54
CHIP = (4.0, 4.0, 0.9)              # the MPU-6050's QFN-24
SOCKET = (4.25, 6.0, 2.95)          # a STEMMA QT socket (JST SH, side entry): deep, wide, tall
SOCKET_X = 10.16                    # the two sockets' x, either side
MOUNTING_HOLES = [(x, y) for x in (-10.16, 10.16) for y in (-6.35, 6.35)]       # 2.5 mm, plated
PINS = [(x, y) for y in (-6.35, 6.35) for x in (-6.35, -3.81, -1.27, 1.27, 3.81, 6.35)]
AXIS_MM = 17                        # the chip's X, Y and Z drawn as red, green and blue arrows, out past the board


# ---- STL ------------------------------------------------------------------

def read_stl(path):
    data = path.read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    if 84 + 50 * n != len(data):
        raise ValueError(f'{path.name}: not a binary STL')
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')]),
                        count=n, offset=84)
    return rec['v'].astype(np.float64)                  # (triangles, 3, xyz), mm


def simplify(tris, target):
    '''The same shape in about `target` triangles - for display only (MuJoCo
    takes at most 200,000 per mesh, and a browser wants far fewer).'''
    if len(tris) <= target:
        return tris
    import fast_simplification
    verts, faces = np.unique(tris.reshape(-1, 3), axis=0, return_inverse=True)
    v, f = fast_simplification.simplify(verts.astype(np.float32), faces.reshape(-1, 3).astype(np.int32),
                                        target_reduction=1 - target / len(tris))
    return v[f].astype(np.float64)


def write_stl(path, tris, header=b'q8botone-sim model/build.py'):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    path.write_bytes(header.ljust(80, b' ') + struct.pack('<I', len(tris)) + rec.tobytes())


def board_mesh(length, width, thickness, r, segments=6):
    '''A circuit board's outline - a rectangle with rounded corners - as a slab
    with its top face on z = 0, centred on the origin (mm).'''
    outline = []
    for cx, cy, start in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        for k in range(segments + 1):
            a = math.radians(start + 90 * k / segments)
            outline.append((cx * (length / 2 - r) + r * math.cos(a), cy * (width / 2 - r) + r * math.sin(a)))
    top = [np.array([x, y, 0.0]) for x, y in outline]          # anticlockwise, seen from above
    bottom = [p - [0, 0, thickness] for p in top]
    tris = []
    for i in range(len(outline)):
        j = (i + 1) % len(outline)
        tris += [(np.zeros(3), top[i], top[j]), ([0, 0, -thickness], bottom[j], bottom[i]),
                 (top[i], bottom[i], bottom[j]), (top[i], bottom[j], top[j])]
    return np.array(tris, float)


def imu_board_xml(top):
    '''The MPU6050 breakout on the chassis's top (z = top, m), hidden: Q8Sim
    turns it to the calibrated mounting and shows it. Its body's frame is
    the chip's axes; the board inside is laid out in board coordinates.'''
    s = 1e-3

    def geom(rgba, **attrs):
        return '<geom class="imu_board" ' + ' '.join(f'{k}="{v}"' for k, v in attrs.items()) + f' rgba="{rgba} 0"/>'

    gold, hole, black = '0.85 0.66 0.32', '0.03 0.03 0.03', '0.06 0.06 0.07'
    parts = [geom(black, type='mesh', mesh='mpu6050_board'),
             geom('0.12 0.12 0.13', type='box', size=fmt(CHIP[0] / 2, CHIP[1] / 2, CHIP[2] / 2, scale=s),
                  pos=fmt(0, 0, CHIP[2] / 2, scale=s))]
    for side in (-1, 1):
        parts.append(geom('0.20 0.20 0.22', type='box', size=fmt(*(np.array(SOCKET) / 2), scale=s),
                          pos=fmt(side * SOCKET_X, 0, SOCKET[2] / 2, scale=s)))
    t = BOARD[2]
    for (x, y), ring, drill in [(p, 2.1, 1.25) for p in MOUNTING_HOLES] + [(p, 0.85, 0.5) for p in PINS]:
        # plated through: the ring shows on both faces, the hole through it
        parts.append(geom(gold, type='cylinder', size=fmt(ring, t / 2 + 0.03, scale=s), pos=fmt(x, y, -t / 2, scale=s)))
        parts.append(geom(hole, type='cylinder', size=fmt(drill, t / 2 + 0.05, scale=s), pos=fmt(x, y, -t / 2, scale=s)))
    arrows = []
    for axis, rgba in ((0, '0.85 0.15 0.15'), (1, '0.15 0.65 0.2'), (2, '0.15 0.35 0.9')):
        tip = np.array([0.0, 0.0, CHIP[2] + 0.3])
        tip[axis] += AXIS_MM
        arrows.append(geom(rgba, type='capsule', size=fmt(0.35, scale=s),
                           fromto=fmt(0, 0, CHIP[2] + 0.3, *tip, scale=s)))
        arrows.append(geom(rgba, type='sphere', size=fmt(0.8, scale=s), pos=fmt(*tip, scale=s)))
    pad = '\n          '
    return f'''
      <!-- The IMU: an Adafruit MPU-6050 breakout, drawn while the twin follows one.
           Its frame is the chip's axes, as calibrated; where on the robot it
           really sits isn't known, so it's drawn on top. -->
      <body name="imu_board" pos="0 0 {top:.6g}">
        {pad.join(arrows)}
        <body name="imu_pcb" euler="0 0 90">
          {pad.join(parts)}
        </body>
      </body>'''


# ---- Mass properties and pivots from a mesh -------------------------------

def mass_properties(tris, mass):
    '''Centre of mass and inertia about it (mesh units) of a uniform closed mesh.'''
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    det = np.einsum('ij,ij->i', a, np.cross(b, c))
    vol = det.sum() / 6
    com = ((a + b + c) * det[:, None]).sum(0) / 24 / vol
    canon = np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]]) / 120
    cov = np.einsum('n,nij,jk,nlk->il', det, np.stack([a, b, c], 2), canon, np.stack([a, b, c], 2))
    cov = (cov - vol * np.outer(com, com)) * mass / vol
    return com, np.trace(cov) * np.eye(3) - cov


def round_holes(tris, max_r):
    '''Circles (centre x, z and radius) fitted to every round patch of surface
    square to Y - hole walls and discs on a part lying in the XZ plane.'''
    t = tris
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    t = t[np.abs(n[:, 1]) < 0.05]
    verts, inv = np.unique(np.round(t.reshape(-1, 3), 4), axis=0, return_inverse=True)
    parent = np.arange(len(verts))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for tri in inv.reshape(-1, 3):
        root = find(tri[0])
        for v in tri[1:]:
            parent[find(v)] = root
    roots = np.array([find(i) for i in range(len(verts))])
    holes = []
    for r in np.unique(roots):
        p = verts[roots == r]
        if len(p) < 12:
            continue
        A = np.column_stack([p[:, 0], p[:, 2], np.ones(len(p))])
        (u, v, w), *_ = np.linalg.lstsq(A, p[:, 0] ** 2 + p[:, 2] ** 2, rcond=None)
        cx, cz = u / 2, v / 2
        rad = math.sqrt(w + cx ** 2 + cz ** 2)
        rms = np.sqrt(np.mean((np.hypot(p[:, 0] - cx, p[:, 2] - cz) - rad) ** 2))
        if rad < max_r and rms < 0.02 * rad + 0.01:
            holes.append((cx, cz, rad, p[:, 1].min(), p[:, 1].max()))
    return holes


# ---- The CAD, measured ----------------------------------------------------

class Link:
    '''One link part, moved into its body frame (see the module doc).'''

    def __init__(self, name, tris, proximal, distal, mass, assembly_com):
        self.name = name
        d = np.array([distal[0] - proximal[0], distal[1] - proximal[1]])
        self.length = float(np.hypot(*d))
        th = math.atan2(d[1], d[0])
        # Rotate about Y so proximal -> distal runs along +X; Y stays put,
        # with Y = 0 on the part's own centre of mass.
        R = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        com_y = mass_properties(tris, mass)[0][1]
        self.tris = (tris - np.array([proximal[0], com_y, proximal[1]])) @ R.T
        self.y_range = (self.tris[..., 1].min(), self.tris[..., 1].max())
        self.mass = mass
        self.com, self.inertia = mass_properties(self.tris, mass)
        # Where it sits across the robot: its centre of mass's Y in the
        # assembly (the right side, as drawn) - from Onshape, since the mesh
        # alone can't say which face mounts where (a raised boss, a recess).
        self.y = assembly_com[1]


def measure_cad(cad, inertials):
    parts = inertials['parts']
    chassis = read_stl(cad / parts['chassis']['stl'])

    # Servo shafts: the round horn discs on the chassis's sides.
    discs = [h for h in round_holes(chassis, 14.0) if min(abs(h[3]), abs(h[4])) > 15]
    xs = sorted({round(h[0], 3) for h in discs})
    zs = {round(h[1], 3) for h in discs}
    side_face = max(max(abs(h[3]), abs(h[4])) for h in discs)   # the discs' outer face, |y|
    assert len(discs) == 8 and len(xs) == 4 and len(zs) == 1, f'expected 8 servo discs, got {discs}'
    shafts = {'rear': (xs[0], xs[1]), 'front': (xs[2], xs[3])}      # (rear servo, front servo) of a pair
    shaft_z = zs.pop()

    def pivots(tris, kind):
        holes = round_holes(tris, 8.0)
        if kind == 'upper':
            # The servo shaft is the biggest hole (the horn's centre); the knee
            # is the hole farthest from it.
            shaft = max(holes, key=lambda h: h[2])
            knee = max(holes, key=lambda h: math.hypot(h[0] - shaft[0], h[1] - shaft[1]))
            return (shaft[0], shaft[1]), (knee[0], knee[1])
        # A lower link: its two farthest-apart holes; the knee is the upper one
        # as the part is drawn (checked against the assembly in check_model).
        a, b = max(((p, q) for p in holes for q in holes),
                   key=lambda pq: math.hypot(pq[0][0] - pq[1][0], pq[0][1] - pq[1][1]))
        knee, foot = (a, b) if a[1] > b[1] else (b, a)
        return (knee[0], knee[1]), (foot[0], foot[1])

    links = {}
    for key, kind in (('upper_link', 'upper'), ('lower_outer_link', 'lower'), ('lower_inner_link', 'lower')):
        tris = read_stl(cad / parts[key]['stl'])
        prox, dist = pivots(tris, kind)
        links[key] = Link(key, tris, prox, dist, parts[key]['mass'], parts[key]['check'][0]['com'])
    return chassis, shafts, shaft_z, side_face, links


# ---- Five-bar kinematics, in a leg's plane --------------------------------
# Leg frame: the rear servo's shaft at the origin, the front one at x = d,
# z up. Angles in degrees below forward, as the library's.

def leg_fk(q_front, q_rear, d, l1, l2):
    '''Knees, foot and the two knee joint angles for given servo angles.'''
    def below(a):
        return np.array([math.cos(math.radians(a)), -math.sin(math.radians(a))])
    kf = np.array([d, 0.0]) + l1 * below(q_front)
    kr = l1 * below(q_rear)
    v = kf - kr
    dist = np.linalg.norm(v)
    if dist > 2 * l2:
        raise ValueError('knees too far apart for the lower links')
    mid = kr + v / 2
    h = math.sqrt(l2 ** 2 - (dist / 2) ** 2)
    n = np.array([v[1], -v[0]]) / dist
    foot = min(mid + h * n, mid - h * n, key=lambda p: p[1])      # the foot is the lower branch

    def angle_below(vec):
        return math.degrees(math.atan2(-vec[1], vec[0]))

    knee_front = angle_below(foot - kf) - q_front
    knee_rear = angle_below(foot - kr) - q_rear
    # Keep each on its own continuous range: the front knee positive, the rear negative.
    knee_front = (knee_front + 360) % 360
    knee_rear = (knee_rear - 360) % -360
    return kf, kr, foot, knee_front, knee_rear


# ---- MJCF -----------------------------------------------------------------

def fmt(*v, scale=1.0):
    return ' '.join(f'{x * scale:.6g}' for x in v)


def inertial(link, flip):
    '''<inertial> of a link body: kg, m, kg m^2. A flipped part is the same
    part turned 180 deg about its own length (X).'''
    R = np.diag([1.0, -1.0, -1.0]) if flip else np.eye(3)
    com = R @ link.com
    I = R @ link.inertia @ R.T
    return (f'<inertial pos="{fmt(*com, scale=1e-3)}" mass="{link.mass * 1e-3:.6g}" '
            f'fullinertia="{fmt(I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2], scale=1e-9)}"/>')


def build(cad):
    inertials = yaml.safe_load((HERE / 'inertials.yaml').read_text())
    chassis, shafts, shaft_z, side_face, links = measure_cad(cad, inertials)
    E, C, D = links['upper_link'], links['lower_outer_link'], links['lower_inner_link']
    d = shafts['front'][1] - shafts['front'][0]
    assert abs(d - (shafts['rear'][1] - shafts['rear'][0])) < 1e-6
    assert abs(C.length - D.length) < 0.01, (C.length, D.length)
    l1, l2 = E.length, C.length

    meshes = HERE / 'meshes'
    meshes.mkdir(exist_ok=True)
    write_stl(meshes / 'chassis.stl', simplify(chassis, MESH_TRIANGLES['chassis']))
    for link in links.values():
        write_stl(meshes / f'{link.name}.stl', simplify(link.tris, MESH_TRIANGLES['link']))
    write_stl(meshes / 'mpu6050_board.stl', board_mesh(*BOARD, BOARD_CORNER_R))

    ch = inertials['parts']['chassis']
    ci = ch['inertia']
    lo, hi = chassis.reshape(-1, 3).min(0), chassis.reshape(-1, 3).max(0)
    box_half, box_mid = (hi - lo) / 2, (hi + lo) / 2

    # Standing height for the keyframe: the idle pose, feet on the floor.
    foot_r = (D.tris[..., 2].max() - D.tris[..., 2].min()) / 2       # the lower link's rounded end
    # An upper link's outline, for the two arms of a leg hitting each other:
    # the horn end's disc, and the link's width at the knee end.
    horn_r = (E.tris[..., 2].max() - E.tris[..., 2].min()) / 2
    verts = E.tris.reshape(-1, 3)
    near_knee = verts[verts[:, 0] > l1 - 1]
    knee_r = (near_knee[:, 2].max() - near_knee[:, 2].min()) / 2
    _, _, foot_idle, kf_idle, kr_idle = leg_fk(*IDLE, d, l1, l2)
    stand_z = (-(foot_idle[1] + shaft_z) + foot_r) * 1e-3

    def across(side, y):        # a right-side Y (as drawn) on this side: the left is mirrored
        return -side * y

    body_xml, equality, tendons, actuators, qpos, ctrl = [], [], [], [], [], []
    for leg, (side, pair) in LEGS.items():
        rear_x, front_x = shafts['front' if pair > 0 else 'rear']
        flip = side > 0                         # the left side's parts are flipped over
        quat = ' quat="0 1 0 0"' if flip else ''
        arms = []
        for servo, x, lower, idle, knee_idle in (('front', front_x, D, IDLE[0], kf_idle),
                                                 ('rear', rear_x, C, IDLE[1], kr_idle)):
            name = f'{leg}_{servo}'
            dy = across(side, lower.y) - across(side, E.y)
            # Both foot points on the foot pin, level with D's middle.
            foot_y = across(side, D.y) - across(side, lower.y)
            foot_geom = ''
            if lower is D:   # the foot that touches the floor: D's rounded end, around the foot pin
                foot_geom = (f'\n            <geom name="{leg}_foot" class="foot" '
                             f'pos="{fmt(l2, foot_y, 0, scale=1e-3)}" size="{foot_r * 1e-3:.6g}"/>')
            arms.append(f'''
        <body name="{name}_arm" pos="{fmt(x, across(side, E.y), shaft_z, scale=1e-3)}">
          <joint name="{name}" class="servo"/>
          {inertial(E, flip)}
          <geom class="visual" mesh="upper_link"{quat} material="arm"/>
          <geom class="arm" type="sphere" size="{horn_r * 1e-3:.6g}"/>
          <geom class="arm" type="capsule" fromto="{fmt(0, 0, 0, l1, 0, 0, scale=1e-3)}" size="{knee_r * 1e-3:.6g}"/>
          <body name="{name}_lower" pos="{fmt(l1, dy, 0, scale=1e-3)}">
            <joint name="{name}_knee" class="knee"/>
            {inertial(lower, flip)}
            <geom class="visual" mesh="{lower.name}"{quat} material="{'link_c' if lower is C else 'link_d'}"/>
            <site name="{name}_foot" pos="{fmt(l2, foot_y, 0, scale=1e-3)}" size="0.0015"/>{foot_geom}
          </body>
        </body>''')
            actuators.append(f'    <position name="{name}" joint="{name}" class="servo"/>')
            qpos += [idle, knee_idle]
            ctrl.append(idle)
        body_xml.append(f'\n        <!-- {leg}: {"left" if side > 0 else "right"} side, '
                        f'{"front" if pair > 0 else "rear"} pair of shafts -->' + ''.join(arms))
        equality.append(f'    <connect name="{leg}_foot" site1="{leg}_rear_foot" site2="{leg}_front_foot"/>')
        tendons.append(f'''    <fixed name="{leg}_foot_angle" range="{fmt(*map(math.radians, FOOT_ANGLE_RANGE))}">
      <joint joint="{leg}_front" coef="1"/>
      <joint joint="{leg}_front_knee" coef="1"/>
      <joint joint="{leg}_rear" coef="-1"/>
      <joint joint="{leg}_rear_knee" coef="-1"/>
    </fixed>''')

    def rad(deg_list):          # hinge qpos is radians even with angle="degree"
        return [math.radians(a) for a in deg_list]

    key_qpos = fmt(0, 0, stand_z, 1, 0, 0, 0) + ' ' + ' '.join(f'{a:.10g}' for a in rad(qpos))
    key_ctrl = ' '.join(f'{a:.10g}' for a in rad(ctrl))

    xml = f'''<!--
  Q8botOne - MuJoCo model. GENERATED by model/build.py from the sim CAD
  (q8botone-hardware/cad/sim) and model/inertials.yaml: edit those, then
  rebuild - not this file.

  Units are SI: metres, kilograms, radians in qpos/ctrl (degrees in this file's
  joint ranges). Frames: X forward, Y left, Z up.

  Leg geometry, measured from the CAD: each leg's two servo shafts {d:g} mm
  apart, upper links (E) {l1:g} mm shaft to knee, lower links (C, D) {l2:g} mm
  knee to foot. A joint angle is the library's: degrees below forward.
-->
<mujoco model="q8botone">
  <compiler angle="degree" meshdir="meshes" inertiafromgeom="false" autolimits="true"/>
  <!-- 1 ms steps; implicitfast handles the servos' damping stably -->
  <option timestep="0.001" integrator="implicitfast"/>

  <visual>
    <global offwidth="1600" offheight="1200"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>

  <default>
    <!-- Meshes are for looks only: contact uses the simple shapes below -->
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"/>
    </default>
    <!-- A servo: XL330-M077 on 6 V. Stall torque from the e-manual; gain,
         damping and armature are starting values to tune against the robot. -->
    <default class="servo">
      <joint axis="0 1 0" armature="{ARMATURE:g}"/>
      <position kp="{KP:g}" kv="{KV:g}" forcerange="{-STALL_TORQUE:g} {STALL_TORQUE:g}"/>
    </default>
    <!-- A knee: a free pin between an upper and a lower link -->
    <default class="knee">
      <joint axis="0 1 0"/>
    </default>
    <!-- An upper link's outline: a leg's two arms share a plane, so they can't
         swing through each other. Touches only the other arms. -->
    <default class="arm">
      <geom contype="8" conaffinity="8" group="3" rgba="0.3 0.5 0.9 0.3"/>
    </default>
    <!-- Each leg's loop closure: as stiff as the 1 ms step allows (MuJoCo's
         default lets the loop stretch millimetres under the robot's weight) -->
    <equality solref="0.002 1" solimp="0.99 0.999 0.0001"/>
    <!-- The foot: the lower link's rounded end. Touches only the floor. -->
    <default class="foot">
      <geom type="sphere" contype="4" conaffinity="0" friction="{FOOT_FRICTION:g} 0.005 0.0001" rgba="0.9 0.3 0.2 0.4" group="3"/>
    </default>
    <!-- The test stand, for looks only. Invisible (alpha 0) until it's in use:
         whoever switches its weld on shows it too. -->
    <default class="stand">
      <geom type="cylinder" contype="0" conaffinity="0" group="2" rgba="0.30 0.32 0.35 0"/>
    </default>
    <!-- The IMU board, for looks only and likewise hidden until it's shown -->
    <default class="imu_board">
      <geom contype="0" conaffinity="0" group="2"/>
    </default>
  </default>

  <asset>
    <mesh name="chassis" file="chassis.stl" scale="0.001 0.001 0.001"/>
    <mesh name="upper_link" file="upper_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="lower_outer_link" file="lower_outer_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="lower_inner_link" file="lower_inner_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="mpu6050_board" file="mpu6050_board.stl" scale="0.001 0.001 0.001"/>
    <!-- Shades of black, a different one for each kind of part -->
    <material name="chassis" rgba="0.17 0.17 0.18 1" specular="0.3"/>
    <material name="arm" rgba="0.32 0.32 0.34 1" specular="0.3"/>
    <material name="link_c" rgba="0.08 0.08 0.09 1" specular="0.3"/>
    <material name="link_d" rgba="0.24 0.24 0.25 1" specular="0.3"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.80 0.85 0.90" rgb2="0.98 0.98 1.0" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.92 0.93 0.94" rgb2="0.84 0.86 0.88" width="512" height="512"/>
    <material name="floor" texture="grid" texuniform="true" texrepeat="10 10"/>
  </asset>

  <worldbody>
    <!-- Size 0: drawn without edges, for walking as far as you like -->
    <geom name="floor" type="plane" size="0 0 0.05" material="floor" contype="1" conaffinity="6"/>

    <!-- A test stand: holds the chassis with its servo shafts {STAND_HEIGHT * 1000:g} mm up,
         out of any foot's reach. Off unless its weld (in <equality>) is on. -->
    <body name="stand" mocap="true" pos="0 0 {STAND_HEIGHT:g}">
      <geom class="stand" fromto="0 0 {lo[2] * 1e-3:.6g} 0 0 {-STAND_HEIGHT + 0.003:.6g}" size="0.008"/>
      <geom class="stand" fromto="0 0 {-STAND_HEIGHT + 0.003:.6g} 0 0 {-STAND_HEIGHT:.6g}" size="0.035"/>
    </body>

    <body name="chassis" pos="0 0 {stand_z:.6g}">
      <freejoint name="root"/>
      <!-- The light goes where the robot goes, pointing the same way, so the
           floor is lit alike and the robot casts a shadow wherever it walks -->
      <light mode="trackcom" pos="0 -0.4 1" dir="0 0.3 -1" directional="true" diffuse="0.6 0.6 0.6"/>
      <!-- Measured: servos, PCB, battery and frame as one part (inertials.yaml) -->
      <inertial pos="{fmt(*ch['com'], scale=1e-3)}" mass="{ch['mass'] * 1e-3:.6g}" fullinertia="{fmt(ci['xx'], ci['yy'], ci['zz'], ci['xy'], ci['xz'], ci['yz'], scale=1e-9)}"/>
      <geom class="visual" mesh="chassis" material="chassis"/>
      <!-- Its outline for contact - for when the robot ends up on its back or side -->
      <geom name="chassis_box" type="box" size="{fmt(*box_half, scale=1e-3)}" pos="{fmt(*box_mid, scale=1e-3)}" contype="2" conaffinity="0" group="3" rgba="0.95 0.75 0.2 0.2"/>
      <site name="imu" pos="0 0 0" size="0.002"/>{imu_board_xml((box_mid[2] + box_half[2]) * 1e-3)}
{''.join(body_xml)}
    </body>
  </worldbody>

  <!-- An IMU at the chassis's origin, on its axes: what a chip there would
       sense (accelerometer: specific force, reading up at rest) -->
  <sensor>
    <accelerometer name="imu_accel" site="imu"/>
    <gyro name="imu_gyro" site="imu"/>
  </sensor>

  <!-- A five-bar leg is a loop, but a body tree can't hold one: each lower
       link carries a foot point, and this pins the two together. -->
  <equality>
{chr(10).join(equality)}
    <!-- The test stand's grip on the chassis: off until switched on -->
    <weld name="stand" body1="stand" body2="chassis" relpose="0 0 0 1 0 0 0" active="false"/>
  </equality>

  <!-- The angle between a leg's two lower links at the foot: D's direction
       minus C's (each = servo + knee angle). A real leg can't turn inside out -
       the foot passing between the knees - which is this reaching 180 deg, so
       it's kept inside {FOOT_ANGLE_RANGE[0]:g}-{FOOT_ANGLE_RANGE[1]:g} deg (in radians here). -->
  <tendon>
{chr(10).join(tendons)}
  </tendon>

  <actuator>
{chr(10).join(actuators)}
  </actuator>

  <keyframe>
    <!-- Standing on the firmware's idle pose: front servos {IDLE[0]:g} deg, rear {IDLE[1]:g} -->
    <key name="idle" qpos="{key_qpos}" ctrl="{key_ctrl}"/>
  </keyframe>
</mujoco>
'''
    (HERE / 'q8botone.xml').write_text(xml, encoding='utf-8')
    return {'d': d, 'l1': l1, 'l2': l2, 'shafts': shafts, 'shaft_z': shaft_z, 'side_face': side_face,
            'foot_r': foot_r, 'stand_z': stand_z, 'links': links}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--cad', type=Path, default=DEFAULT_CAD, help='folder with the STL exports')
    info = build(p.parse_args().cad)
    print(f"leg: shafts {info['d']:g} mm apart, upper links {info['l1']:g} mm, lower links {info['l2']:g} mm")
    print(f"shafts at x {info['shafts']['rear']} (rear legs) and {info['shafts']['front']} (front legs), "
          f"z {info['shaft_z']:g}; discs' outer face at |y| {info['side_face']:g}")
    for link in info['links'].values():
        print(f'{link.name}: {link.length:.3f} mm pivot to pivot, {link.mass:g} g, '
              f'y {link.y_range[0]:.2f}..{link.y_range[1]:.2f} mm in its frame')
    print(f"stands {info['stand_z'] * 1000:.1f} mm up on the idle pose; wrote model/q8botone.xml and model/meshes/")


if __name__ == '__main__':
    main()
