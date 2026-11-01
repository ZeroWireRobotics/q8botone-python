'''
Checks on the built model (model/q8botone.xml), in MuJoCo itself:

    python model/check_model.py

1. Placement - one leg posed as in the Onshape assembly: every link's centre
   of mass where Onshape measured it, and each link mesh where its body says.
2. Loops - on the idle keyframe each leg's two foot points coincide.
3. Standing - on the idle pose the robot stands still and level.
4. Limp - servos off, the robot drops onto its belly with every leg loop
   still the right way round.
5. Test stand - off by default; on, it holds the chassis still in the air
   while the legs swing, feet clear of the floor.
'''

import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
failures = []


def check(name, ok, detail=''):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  ' + str(detail)) if detail != '' else ''}")
    if not ok:
        failures.append(name)


def mesh_com_world(m, d, geom):
    '''Centroid of a mesh geom's volume, in the world, from MuJoCo's own vertices.'''
    mid = m.geom_dataid[geom]
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
    w = d.geom_xpos[geom] + v @ d.geom_xmat[geom].reshape(3, 3).T
    a, b, c = w[f[:, 0]], w[f[:, 1]], w[f[:, 2]]
    det = np.einsum('ij,ij->i', a, np.cross(b, c))
    return ((a + b + c) * det[:, None]).sum(0) / 24 / (det.sum() / 6)


def leg_pose(d_mm, l1, l2, q_front, q_rear):
    import build
    return build.leg_fk(q_front, q_rear, d_mm, l1, l2)


def set_leg(m, d, leg, q_front, q_rear, geom):
    _, _, _, kf, kr = leg_pose(*geom, q_front, q_rear)
    for joint, deg in ((f'{leg}_front', q_front), (f'{leg}_front_knee', kf),
                       (f'{leg}_rear', q_rear), (f'{leg}_rear_knee', kr)):
        d.qpos[m.jnt_qposadr[m.joint(joint).id]] = math.radians(deg)


def foot_gap(m, d, leg):
    return np.linalg.norm(d.site(f'{leg}_rear_foot').xpos - d.site(f'{leg}_front_foot').xpos)


def main():
    import sys
    sys.path.insert(0, str(HERE))
    import build
    m = mujoco.MjModel.from_xml_path(str(HERE / 'q8botone.xml'))
    d = mujoco.MjData(m)
    parts = yaml.safe_load((HERE / 'inertials.yaml').read_text())['parts']
    # Leg geometry, as the model has it: knee and foot offsets
    l1 = m.body('RR_front_lower').pos[0] * 1000
    l2 = m.site('RR_front_foot').pos[0] * 1000
    shaft_d = (m.body('RR_front_arm').pos[0] - m.body('RR_rear_arm').pos[0]) * 1000
    geom = (shaft_d, l1, l2)
    print(f'leg geometry in the model: shafts {shaft_d:.3f} mm apart, upper {l1:.3f}, lower {l2:.3f}')

    check('total mass 246.8 g', abs(sum(m.body_mass) * 1000 - 246.8) < 0.01, f'{sum(m.body_mass) * 1000:.2f} g')

    # ---- 1. Placement: the RR leg in the Onshape assembly's pose -----------
    # Each upper link's angle from Onshape's centre of mass (it lies on the
    # link's line), the rest from the leg's kinematics.
    e1, e2 = parts['upper_link']['check']           # <1> on the rear servo, <2> on the front
    rear_shaft = m.body('RR_rear_arm').pos * 1000
    front_shaft = m.body('RR_front_arm').pos * 1000

    def below(com, shaft):
        return math.degrees(math.atan2(-(com[2] - shaft[2]), com[0] - shaft[0]))
    q_rear, q_front = below(e1['com'], rear_shaft), below(e2['com'], front_shaft)
    mujoco.mj_resetData(m, d)
    d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]                # chassis at the origin, level: the assembly frame
    set_leg(m, d, 'RR', q_front, q_rear, geom)
    mujoco.mj_forward(m, d)
    print(f'RR leg posed as the assembly: front servo {q_front:.2f} deg, rear {q_rear:.2f} deg')
    for body, (key, idx), label in (('RR_rear_arm', ('upper_link', 0), 'E <1>'),
                                    ('RR_front_arm', ('upper_link', 1), 'E <2>'),
                                    ('RR_rear_lower', ('lower_outer_link', 0), 'C <1>'),
                                    ('RR_front_lower', ('lower_inner_link', 0), 'D <1>')):
        want = np.array(parts[key]['check'][idx]['com'])
        got = d.body(body).xipos * 1000
        check(f'{label} ({body}) centre of mass where Onshape has it', np.linalg.norm(got - want) < 0.05,
              f'{np.round(got, 3)} vs {want} ({np.linalg.norm(got - want):.3f} mm)')
        g = [i for i in range(m.ngeom) if m.geom_bodyid[i] == m.body(body).id and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH][0]
        mesh_c = mesh_com_world(m, d, g) * 1000
        check(f'   its mesh is where the body puts it', np.linalg.norm(mesh_c - want) < 0.3,
              f'mesh centroid {np.round(mesh_c, 2)} ({np.linalg.norm(mesh_c - want):.2f} mm - the display mesh is simplified)')
    check('RR foot points meet in that pose', foot_gap(m, d, 'RR') * 1000 < 0.01, f'{foot_gap(m, d, "RR") * 1000:.4f} mm')
    # The left side mirrors the right: FL in the same pose, its centres of mass at -y
    set_leg(m, d, 'RL', q_front, q_rear, geom)
    mujoco.mj_forward(m, d)
    off = max(np.linalg.norm(d.body(f'RL_{b}').xipos * [1, -1, 1] - d.body(f'RR_{b}').xipos)
              for b in ('front_arm', 'rear_arm', 'front_lower', 'rear_lower')) * 1000
    check('left side mirrors the right (RL vs RR, same pose)', off < 0.01, f'centres of mass within {off:.4f} mm')
    chassis_c = mesh_com_world(m, d, [i for i in range(m.ngeom) if m.geom_bodyid[i] == m.body('chassis').id
                                      and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH][0]) * 1000
    check('chassis mesh sits on the assembly frame', abs(chassis_c[1]) < 0.5 and abs(chassis_c[0]) < 2,
          f'mesh centroid {np.round(chassis_c, 2)} mm (uniform-density centroid; the measured COM is {parts["chassis"]["com"]})')

    # ---- 2. Loops on the idle keyframe -------------------------------------
    key = m.key('idle').id
    mujoco.mj_resetDataKeyframe(m, d, key)
    mujoco.mj_forward(m, d)
    qf, qr = math.degrees(d.joint('FL_front').qpos[0]), math.degrees(d.joint('FL_rear').qpos[0])
    check('idle keyframe: front servos at 30 deg, rear at 150 (qpos in radians)',
          abs(qf - 30) < 1e-6 and abs(qr - 150) < 1e-6, f'{qf:.6f}, {qr:.6f}')
    gaps = {leg: foot_gap(m, d, leg) * 1000 for leg in ('FL', 'FR', 'RL', 'RR')}
    check('idle keyframe: every leg loop closed', max(gaps.values()) < 0.01, {k: round(v, 4) for k, v in gaps.items()})
    foot_z = min(d.geom(f'{leg}_foot').xpos[2] - m.geom(f'{leg}_foot').size[0] for leg in gaps)
    check('idle keyframe: feet just touch the floor', abs(foot_z) < 1e-4, f'lowest foot {foot_z * 1000:.3f} mm')

    # ---- 3. Standing on the idle pose --------------------------------------
    z0 = d.body('chassis').xpos[2]
    worst_gap = 0.0
    for _ in range(3000):                                       # 3 s
        mujoco.mj_step(m, d)
        worst_gap = max(worst_gap, max(foot_gap(m, d, leg) for leg in gaps))
    z1 = d.body('chassis').xpos[2]
    up = d.body('chassis').xmat.reshape(3, 3)[:, 2]
    tilt = math.degrees(math.acos(min(1.0, up[2])))
    drift = np.linalg.norm(d.body('chassis').xpos[:2]) * 1000
    check('stands 3 s: settles a little, stays up', -3e-3 < z1 - z0 < 0.5e-3,
          f'height {z0 * 1000:.2f} -> {z1 * 1000:.2f} mm')
    check('stands 3 s: level', tilt < 1.0, f'tilt {tilt:.3f} deg')
    check('stands 3 s: stays put', drift < 1.0, f'moved {drift:.3f} mm')
    check('stands 3 s: still', np.abs(d.qvel).max() < 1e-2, f'max joint speed {np.abs(d.qvel).max():.2e}')
    check('stands 3 s: leg loops stay closed', worst_gap * 1000 < 0.2, f'worst foot gap {worst_gap * 1000:.3f} mm')
    floor = m.geom('floor').id
    touching = {m.geom(d.contact[i].geom[1] if d.contact[i].geom[0] == floor else d.contact[i].geom[0]).name
                for i in range(d.ncon) if floor in d.contact[i].geom}
    check('stands 3 s: on its four feet only', touching == {'FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'}, sorted(touching))
    sag = {leg: round(math.degrees(d.joint(f'{leg}_front').qpos[0]) - 30, 2) for leg in gaps}
    print(f'      servo sag under load (deg off the 30 deg target, front servos): {sag}')

    # ---- 4. Limp: servos off -----------------------------------------------
    mujoco.mj_resetDataKeyframe(m, d, key)
    gains = m.actuator_gainprm.copy(), m.actuator_biasprm.copy()
    m.actuator_gainprm[:, 0] = 0
    m.actuator_biasprm[:, :3] = 0
    # The right way round: the angle between the lower links at the foot
    # (D's direction minus C's - the <tendon>) strictly between 0 and 180 deg.
    lo, hi = math.inf, -math.inf
    for _ in range(3000):
        mujoco.mj_step(m, d)
        angles = np.degrees(d.ten_length)
        lo, hi = min(lo, angles.min()), max(hi, angles.max())
    check('limp: falls without blowing up', np.isfinite(d.qpos).all() and np.abs(d.qvel).max() < 50,
          f'max speed {np.abs(d.qvel).max():.2e}')
    check('limp: no leg loop turns inside out', 0 < lo and hi < 180,
          f'foot angles stayed within {lo:.1f}..{hi:.1f} deg')
    arms = {leg: (round(math.degrees(d.joint(f'{leg}_front').qpos[0]), 1),
                  round(math.degrees(d.joint(f'{leg}_rear').qpos[0]), 1)) for leg in gaps}
    print(f'      limp servo angles (front, rear): {arms}')
    touching = {m.geom(d.contact[i].geom[1] if d.contact[i].geom[0] == floor else d.contact[i].geom[0]).name
                for i in range(d.ncon) if floor in d.contact[i].geom}
    print(f'      limp robot rests on: {sorted(touching)}, chassis at {d.body("chassis").xpos[2] * 1000:.1f} mm')
    m.actuator_gainprm[:], m.actuator_biasprm[:] = gains

    # ---- 5. On the test stand ----------------------------------------------
    stand = m.equality('stand').id
    check('test stand: off by default', not m.eq_active0[stand])
    mujoco.mj_resetDataKeyframe(m, d, key)
    h = m.body('stand').pos[2]
    d.mocap_pos[m.body_mocapid[m.body('stand').id]] = [0, 0, h]
    d.qpos[:7] = [0, 0, h, 1, 0, 0, 0]
    d.eq_active[stand] = 1
    servos = [m.actuator(f'{leg}_{s}').id for leg in gaps for s in ('front', 'rear')]
    moved = tilt = 0.0
    feet_down = False
    for i in range(2000):                                       # 2 s of every leg swinging +-30 deg at 2 Hz
        d.ctrl[servos] = m.key_ctrl[key][servos] + math.radians(30) * math.sin(2 * math.pi * 2 * i * 1e-3)
        mujoco.mj_step(m, d)
        moved = max(moved, np.linalg.norm(d.body('chassis').xpos - [0, 0, h]))
        tilt = max(tilt, math.degrees(math.acos(min(1.0, d.body('chassis').xmat[8]))))
        feet_down |= any(floor in d.contact[c].geom for c in range(d.ncon))
    check('test stand: holds the chassis while the legs swing', moved < 0.5e-3 and tilt < 0.5,
          f'moved up to {moved * 1000:.3f} mm, tilted up to {tilt:.3f} deg')
    check('test stand: feet clear of the floor', not feet_down)

    print('\nALL PASS' if not failures else f'\n{len(failures)} FAILED: {failures}')


if __name__ == '__main__':
    main()
