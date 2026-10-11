"""Open Duck Mini v2 (BDX-style) in MuJoCo: upstream flat-terrain scene with identified STS3215 actuators."""
import os, mujoco, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "sim", "scene_flat_terrain.xml")
LEGS = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13]      # actuator indices of the 10 leg joints (head = 5..8)

def build(dt=0.002):
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep = dt; return m

def home(m):
    return m.key("home").qpos.copy()

def reset(m):
    d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, m.key("home").id); mujoco.mj_forward(m, d); return d

if __name__ == "__main__":
    m = build(); d = reset(m)
    print("mass", round(m.body_subtreemass[1], 3), "nu", m.nu, [m.actuator(i).name for i in range(m.nu)])
    print("ctrl@home", d.ctrl, "kp", m.actuator_gainprm[:3, 0], "frange", m.actuator_forcerange[0])
    q0 = d.qpos[7:].copy(); d.ctrl[:] = q0
    for t in range(2500):
        mujoco.mj_step(m, d)
        if t % 500 == 0: print(t, "z", round(d.qpos[2], 3), "up", round(d.xmat[1].reshape(3, 3)[2, 2], 2))
