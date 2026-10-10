"""Evaluate a trained policy: tracking error, falls, optional GIF. Usage: python -m rl.evaluate out/rl/run1/final.zip [--gif out/rl/walk.gif]"""
import argparse, numpy as np, mujoco
from stable_baselines3 import PPO
from .env import WalkEnv

SCRIPT = [(0.12, 0, 0, 6), (0.2, 0, 0, 6), (0, 0, 0, 3), (0.1, 0, 0.5, 6), (0, 0.06, 0, 4), (-0.06, 0, 0, 4), (0.15, 0, -0.4, 6)]   # vx, vy, wz, seconds

def run(model, gif=None, seed=0, randomize=False, script=SCRIPT):
    env = WalkEnv(randomize=randomize, seed=seed, max_steps=10**6); obs, _ = env.reset(seed=seed)
    frames, rend = [], None
    if gif:
        rend = mujoco.Renderer(env.m, 360, 480); cam = mujoco.MjvCamera(); cam.distance = 1.1; cam.elevation = -12; cam.azimuth = 120
    errs, falls, dist = [], 0, 0.0
    for vx, vy, wz, sec in script:
        env.cmd = np.array([vx, vy, wz])
        for k in range(int(sec * 50)):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = env.step(a)
            v = env._quat_rot_inv(env.d.qvel[0:3]); errs.append((abs(v[0] - vx), abs(v[1] - vy), abs(env.d.qvel[5] - wz)))
            if term: falls += 1; obs, _ = env.reset(seed=seed + falls); env.cmd = np.array([vx, vy, wz])
            if rend and k % 2 == 0:
                cam.lookat[:] = env.d.subtree_com[1]; rend.update_scene(env.d, cam); frames.append(rend.render().copy())
    e = np.mean(errs, axis=0)
    if gif:
        import imageio; imageio.mimsave(gif, frames[::1], duration=0.04, loop=0)
    return dict(falls=falls, err_vx=e[0], err_vy=e[1], err_wz=e[2])

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ckpt"); ap.add_argument("--gif"); ap.add_argument("--randomize", action="store_true"); a = ap.parse_args()
    print(run(PPO.load(a.ckpt, device="cpu"), a.gif, randomize=a.randomize))
