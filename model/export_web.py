'''
The Q8botOne you can drive in a web page (zerowirerobotics.com): this
model, run by MuJoCo's own WebAssembly build in the browser, walking the
robot's own gaits. Writes what the page loads:

    model/q8botone.xml, model/meshes/*.stl.gz   the model, made more like
                                             the robot (as the demos'
                                             common.py: servos held to their
                                             motors' speed, a firm floor),
                                             without the test stand or the
                                             IMU board - and with bananas on
                                             the floor, for scale; meshes
                                             gzipped, as a server may not
                                             compress .stl
    gaits.json                               the gait tables, built by the
                                             q8bot library exactly as it
                                             builds the firmware's
    files.json                               every file the page loads and
                                             its size, for its progress bar

The page's own code (drive.js) and the libraries it loads (vendor/: MuJoCo's
WebAssembly build, three.js) live in the site's repository.

    python model/export_web.py               -> ../zerowirerobotics-site/sim/
    python model/export_web.py OUT_DIR

Run it again after the model or the gaits change.
'''

import gzip
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from q8bot.gait_manager import GAITS, GaitManager, gait_info
from q8bot.kinematics_solver import k_solver
from q8bot.protocol import DIRECTION_ORDER

from q8botone_sim.sim import LEGS, MODEL, SERVOS, WIRE

HERE = Path(__file__).resolve().parent
OUT = HERE.parent.parent / 'zerowirerobotics-site' / 'sim'

# As demos/common.py: a servo can't turn faster than its motor (damping of
# stall torque / no-load speed), and the floor is firm
SERVO_DAMPING = 0.228 / 47.8
FLOOR_SOLREF = [0.004, 1.0]
FRAME_S = 0.005                     # a gait frame every 5 ms, as the firmware plays them

# What the page offers, in its order. Bound is left out: the robot's own
# flips over in the sim (and it can't steer).
OFFERED = ('TROT_FAST', 'TROT_HIGH', 'TROT_LOW', 'CRAWL')

# Bananas, for scale: a real one's size - 19 cm along its curve, 3.6 cm
# across, 120 g - lying on its side, (x, y, heading deg) about the robot,
# which faces +x. Loose on the floor: the robot can bump them about.
BANANA_LENGTH = 0.19
BANANA_RADIUS = 0.018
BANANA_BEND = 1.15                  # rad the curve turns end to end
BANANA_MASS = 0.12
# The first lies alongside the robot, as far from the page's first view as
# the robot is - the one to compare it with. The rest are strewn about, a
# couple in the background and the others to be found walking; none on the
# side the page's first view looks from (behind the robot, to its right),
# where it would fill the picture. At least 0.3 m apart, so none start on
# top of another.
BANANAS = ((0.11, -0.13, -45), (0.36, 0.06, 105), (-0.32, 0.28, -35),
           (0.62, -0.28, 20), (0.05, 0.42, 160), (0.70, 0.45, -70), (-0.10, 0.75, 40), (0.95, -0.05, 10))


def _banana_centre(u):
    '''A point on the banana's centreline, u 0 (stem) to 1 (tip), and the
    centreline's direction there - an arc in its own x-y plane, the ends
    curving towards +y.'''
    radius = BANANA_LENGTH / BANANA_BEND
    a = BANANA_BEND * (u - 0.5)
    return np.array([radius * math.sin(a), radius * (1 - math.cos(a)), 0.0]), np.array([math.cos(a), math.sin(a), 0.0])


def _banana_radius(u):
    '''How thick it is along its length: a thin stem, swelling to the body,
    tapering to a blunt tip.'''
    if u < 0.12:
        return 0.0045 + 0.0015 * (u / 0.12)
    k = (u - 0.12) / 0.88
    return max(0.0035, BANANA_RADIUS * math.sin(math.pi * min(1.0, 0.18 + 0.86 * k)) ** 0.55)


def _tube(u0, u1, rings=24, around=20, ridges=0.05):
    '''Vertices and faces of the banana between u0 and u1, its ends closed -
    five shallow ridges round it, as a real one has.'''
    verts, faces = [], []
    for i in range(rings + 1):
        u = u0 + (u1 - u0) * i / rings
        c, t = _banana_centre(u)
        n = np.array([-t[1], t[0], 0.0])
        b = np.array([0.0, 0.0, 1.0])
        r = _banana_radius(u)
        for j in range(around):
            phi = 2 * math.pi * j / around
            rr = r * (1 + ridges * math.cos(5 * phi))
            verts.append(c + rr * (math.cos(phi) * n + math.sin(phi) * b))
    # Faces wound anticlockwise seen from outside, so they face out
    for i in range(rings):
        for j in range(around):
            a, b2 = i * around + j, i * around + (j + 1) % around
            c2, d = a + around, b2 + around
            faces += [(a, b2, c2), (b2, d, c2)]
    for end, ring in ((u0, 0), (u1, rings)):        # caps: a fan round the end's centre
        verts.append(_banana_centre(end)[0])
        centre = len(verts) - 1
        for j in range(around):
            a, b2 = ring * around + j, ring * around + (j + 1) % around
            faces.append((centre, a, b2) if ring == 0 else (centre, b2, a))
    return np.array(verts), np.array(faces)


def add_bananas(spec):
    '''BANANAS on the floor: peel, stem and tip as meshes to be seen,
    capsules along the curve to touch (a mesh would touch as its convex
    hull, filling the curve in).'''
    # The robot's keyframe gets the bananas' places after its own - read from
    # a compiled copy: the spec's key.qpos reads back empty
    robot_qpos = list(spec.copy().compile().key_qpos[0])
    parts = (('banana_stem', 0.0, 0.13, [0.42, 0.40, 0.18, 1]),
             ('banana_peel', 0.11, 0.975, [1.0, 0.86, 0.26, 1]),
             ('banana_tip', 0.965, 1.0, [0.22, 0.16, 0.10, 1]))
    for name, u0, u1, rgba in parts:
        verts, faces = _tube(u0, u1, rings=4 if name != 'banana_peel' else 40)
        mesh = spec.add_mesh()
        mesh.name = name
        mesh.uservert = verts.flatten().tolist()
        mesh.userface = faces.flatten().tolist()
        material = spec.add_material()
        material.name = name
        material.rgba = rgba
    us = np.linspace(0.06, 0.985, 8)
    centre = np.mean([_banana_centre(u)[0] for u in us], axis=0)
    for k, (x, y, heading) in enumerate(BANANAS, 1):
        body = spec.worldbody.add_body()
        body.name = f'banana{k}'
        body.pos = [x, y, BANANA_RADIUS + 0.002]
        a = math.radians(heading) / 2
        body.quat = [math.cos(a), 0, 0, math.sin(a)]
        body.add_freejoint()
        body.explicitinertial = True
        body.mass = BANANA_MASS
        body.ipos = centre.tolist()
        body.inertia = [BANANA_MASS * BANANA_RADIUS ** 2 / 2 + 1e-6, BANANA_MASS * BANANA_LENGTH ** 2 / 12,
                        BANANA_MASS * BANANA_LENGTH ** 2 / 12]
        for name, *_ in parts:
            geom = body.add_geom()
            geom.name = f'banana{k}_{name.split("_")[1]}'
            geom.type = mujoco.mjtGeom.mjGEOM_MESH
            geom.meshname = name
            geom.material = name
            geom.contype = geom.conaffinity = 0
            geom.group = 2
        for i in range(len(us) - 1):
            geom = body.add_geom()
            geom.type = mujoco.mjtGeom.mjGEOM_CAPSULE
            (p0, _), (p1, _) = _banana_centre(us[i]), _banana_centre(us[i + 1])
            geom.fromto = [*p0, *p1]
            geom.size = [0.95 * _banana_radius((us[i] + us[i + 1]) / 2), 0, 0]
            geom.contype, geom.conaffinity = 1, 7       # the floor, feet, chassis and each other
            geom.group = 3
            geom.friction = [0.8, 0.005, 0.0001]
            geom.solref = FLOOR_SOLREF
    key = spec.key('idle')
    qpos = robot_qpos
    for x, y, heading in BANANAS:
        a = math.radians(heading) / 2
        qpos += [x, y, BANANA_RADIUS + 0.002, math.cos(a), 0, 0, math.sin(a)]
    key.qpos = qpos


def web_model(out):
    spec = mujoco.MjSpec.from_file(str(MODEL))
    for leg in LEGS:
        for s in SERVOS:
            spec.joint(f'{leg}_{s}').damping = [SERVO_DAMPING, 0, 0]       # (linear term only)
        spec.geom(f'{leg}_foot').solref = FLOOR_SOLREF
    spec.geom('floor').solref = FLOOR_SOLREF
    # Not on the robot as sold: the test stand and the MPU6050 board on top
    for sensor in list(spec.sensors):
        spec.delete(sensor)
    spec.delete(spec.equality('stand'))
    for name in ('stand', 'imu_board'):
        spec.delete(spec.body(name))
    spec.delete(spec.mesh('mpu6050_board'))
    add_bananas(spec)
    spec.compile()                                  # (fails here, not in the browser)
    (out / 'model' / 'meshes').mkdir(parents=True, exist_ok=True)
    (out / 'model' / 'q8botone.xml').write_text(spec.to_xml(), encoding='utf-8')
    meshes = [m.file for m in spec.meshes if m.file]           # (the bananas' are in the model itself)
    for name in meshes:
        raw = (MODEL.parent / 'meshes' / name).read_bytes()
        # mtime=0: the same mesh gzips to the same bytes, so git sees no change
        (out / 'model' / 'meshes' / f'{name}.gz').write_bytes(gzip.compress(raw, 9, mtime=0))
    return meshes


def manifest(out):
    '''{path: bytes} of everything the page fetches once Start is pressed,
    for its progress bar - written last. (drive.js and three.js have loaded
    by then.)'''
    files = {}
    for path in sorted(out.rglob('*')):
        rel = path.relative_to(out).as_posix()
        if path.is_file() and (rel.startswith('model/') or rel == 'gaits.json'
                               or rel in ('vendor/mujoco/mujoco.js', 'vendor/mujoco/mujoco.wasm')):
            files[rel] = path.stat().st_size
    (out / 'files.json').write_text(json.dumps(files, indent=1) + '\n', encoding='utf-8')
    return files


def gait_tables():
    '''{name: {label, description, stance (8 angles), directions {dir: table
    key or None}, tables {key: [[8 angles] ...]}}} - the speed tiers too.'''
    leg = k_solver()
    names = list(OFFERED) + [n for chain in GaitManager.SPEED_TIER_CHAINS.values() for n in chain]
    gaits = {}
    for name in dict.fromkeys(names):
        manager = GaitManager(leg, GAITS)
        if not manager.load_gait(name):
            raise RuntimeError(f'the library could not build {name}')
        trajectories = manager.current_trajectories[name]
        directions = {d: manager.resolve_direction(d) for d in DIRECTION_ORDER}
        used = {k for k in directions.values() if k is not None}
        x0, y0 = GAITS[name][1], GAITS[name][2]
        q1, q2, ok = leg.ik_solve(x0, y0, True, 2)
        assert ok, name
        info = gait_info(name)
        gaits[name] = {
            'label': info.label, 'description': info.description,
            'stance': [q1, q2] * 4,
            'directions': directions,
            'tables': {k: [[round(a, 1) for a in frame] for frame in trajectories[k]] for k in sorted(used)},
        }
    return gaits


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT
    out.mkdir(parents=True, exist_ok=True)
    meshes = web_model(out)
    data = {
        'frame_s': FRAME_S,
        'wire': [f'{leg}_{s}' for leg, s in WIRE],          # the actuators, in the order the angles are listed
        'offered': list(OFFERED),
        'tiers': {k: v for k, v in GaitManager.SPEED_TIER_CHAINS.items()},
        'tier_boundaries': GaitManager.TIER_BOUNDARIES,
        'meshes': meshes,
        'gaits': gait_tables(),
    }
    (out / 'gaits.json').write_text(json.dumps(data, separators=(',', ':')), encoding='utf-8')
    files = manifest(out)
    print(f'{out}: model + {len(meshes)} meshes + gaits.json; the page loads {len(files)} files, '
          f'{sum(files.values()) / 1e6:.1f} MB before the server compresses them')


if __name__ == '__main__':
    main()
