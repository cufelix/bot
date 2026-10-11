"""Render the trained duck walking a command script to MP4. python -m duck.video out/rl/duck2/ppo_14000000_steps.zip docs/img/duck_walk.mp4"""
import sys, numpy as np, mujoco, imageio.v2 as iio
from stable_baselines3 import PPO
from .env import DuckEnv

SCRIPT = [(0.12, 0, 0, 5), (0.2, 0, 0, 4), (0, 0, 0, 2), (0.1, 0, 0.6, 5), (0.1, 0, -0.6, 5), (0, 0.06, 0, 3), (-0.06, 0, 0, 3), (0.15, 0, 0, 4)]

def main(ckpt, out):
    m = PPO.load(ckpt, device="cpu"); e = DuckEnv(randomize=False, max_steps=10**6); o, _ = e.reset(seed=0)
    e.m.vis.global_.offwidth, e.m.vis.global_.offheight = 960, 720
    r = mujoco.Renderer(e.m, 720, 960); cam = mujoco.MjvCamera(); cam.distance, cam.elevation = 0.8, -15
    w = iio.get_writer(out, fps=25, codec="libx264", quality=8); falls = 0; t = 0
    for vx, vy, wz, sec in SCRIPT:
        e.cmd = np.array([vx, vy, wz])
        for _ in range(int(sec * 50)):
            o, _, term, _, _ = e.step(m.predict(o, deterministic=True)[0]); t += 1
            if term: falls += 1; o, _ = e.reset(seed=falls); e.cmd = np.array([vx, vy, wz])
            if t % 2 == 0:
                cam.lookat[:] = e.d.qpos[:3]; cam.azimuth = 135 + 0.15 * (t / 2) % 360 * 0
                r.update_scene(e.d, cam); w.append_data(r.render())
    w.close(); print("falls", falls, "->", out)

if __name__ == "__main__": main(*sys.argv[1:3])
