'''
Pictures of the model, standing on its idle pose, into renders/:

    python model/render.py
'''

from pathlib import Path

import mujoco
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / 'renders'

VIEWS = {   # name: lookat (m), distance (m), azimuth, elevation (deg)
    'three_quarter': ((0, 0, 0.03), 0.42, 135, -22),
    'side': ((0, 0, 0.03), 0.34, 90, -2),
    'front': ((0, 0, 0.03), 0.30, 180, -8),
    'leg_closeup': ((0.04, 0.03, 0.02), 0.13, 120, -12),
}


def main():
    OUT.mkdir(exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(HERE / 'q8botone.xml'))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key('idle').id)
    for _ in range(1500):                    # let it settle on its feet
        mujoco.mj_step(m, d)
    with mujoco.Renderer(m, 900, 1200) as r:
        cam = mujoco.MjvCamera()
        for name, (lookat, dist, az, el) in VIEWS.items():
            cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
            r.update_scene(d, cam)
            Image.fromarray(r.render()).save(OUT / f'{name}.png')
            print(OUT / f'{name}.png')


if __name__ == '__main__':
    main()
