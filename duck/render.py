"""Shaded MuJoCo renders of the Open Duck Mini v2 model -> docs/img/duck_*.png"""
import os, sys, mujoco, numpy as np, imageio.v2 as iio
HERE = os.path.dirname(os.path.abspath(__file__))
INIT = [0.002, 0.053, -0.63, 1.368, -0.784, 0, 0, 0, 0, 0, 0, -0.003, -0.065, 0.635, 1.379, -0.796]  # from upstream onnx_AWD_mujoco.py

def load():
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "robot", "scene.xml")); d = mujoco.MjData(m)
    d.qpos[3:7] = [1, 0, 0, 0]; d.qpos[7:7 + 16] = INIT; d.ctrl[:16] = INIT
    return m, d

def shot(m, d, az, el, out, dist=0.9, lookat=(0, 0, 0.2)):
    m.vis.global_.offwidth = m.vis.global_.offheight = 900; r = mujoco.Renderer(m, 900, 900); cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance, cam.lookat[:] = az, el, dist, lookat
    r.update_scene(d, cam); iio.imwrite(out, r.render()); r.close()

if __name__ == "__main__":
    m, d = load()
    mujoco.mj_forward(m, d)
    os.makedirs("docs/img", exist_ok=True)
    print("base z", d.qpos[2], "nq", m.nq, "nu", m.nu)
    for n, (a, e) in dict(front=(180, -8), side=(90, -8), back=(0, -8), iso=(145, -18)).items():
        shot(m, d, a, e, f"docs/img/duck_{n}.png", lookat=(0, 0, d.qpos[2]))
