"""Autonomy demo in simulation: the robot finds an orange ball with its head camera and walks to it using the trained walking policy.
python -m autonomy.sim_demo out/rl/run3/final.zip [--ball 1.2 0.5] [--gif out/rl/autonomy.gif]"""
import argparse, numpy as np, mujoco, imageio
from stable_baselines3 import PPO
from rl.env import WalkEnv
from .vision import detect
from .navigator import Navigator

def run(ckpt, ball=(1.2, 0.5), gif=None, seconds=60, seed=0, randomize=True, pillars=()):
    model = PPO.load(ckpt, device="cpu")
    env = WalkEnv(randomize=randomize, seed=seed, max_steps=10**6, scene=dict(balls=[ball], pillars=list(pillars)))
    obs, _ = env.reset(seed=seed); nav = Navigator(dt=0.1)
    cam_r = mujoco.Renderer(env.m, 120, 160); view_r = mujoco.Renderer(env.m, 300, 400) if gif else None
    vc = mujoco.MjvCamera(); vc.distance = 2.2; vc.elevation = -35; vc.azimuth = 150
    frames, state, t_arrive, fell = [], "SEARCH", None, False
    bpos = np.array([ball[0], ball[1]])
    for k in range(int(seconds * 50)):
        if k % 5 == 0:                                           # camera + behaviour at 10 Hz
            cam_r.update_scene(env.d, "head_cam"); img = cam_r.render()
            cmd, state = nav.step(detect(img)); env.cmd = cmd
        a, _ = model.predict(obs, deterministic=True); obs, r, term, trunc, _ = env.step(a)
        if term: fell = True; break
        if gif and k % 4 == 0:
            vc.lookat[:] = env.d.subtree_com[1]; view_r.update_scene(env.d, vc); big = view_r.render().copy()
            big[:120, :160] = img; frames.append(big)
        if state == "ARRIVED" and t_arrive is None: t_arrive = k / 50
        if t_arrive is not None and k / 50 > t_arrive + 2: break
    dist = float(np.linalg.norm(env.d.subtree_com[1][:2] - bpos))
    if gif: imageio.mimsave(gif, frames, duration=0.08, loop=0)
    return dict(arrived=t_arrive is not None and not fell, t_arrive=t_arrive, fell=fell, final_dist_m=round(dist, 2), state=state)

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ckpt"); ap.add_argument("--ball", nargs=2, type=float, default=[1.2, 0.5]); ap.add_argument("--gif"); ap.add_argument("--seconds", type=float, default=60)
    a = ap.parse_args(); print(run(a.ckpt, tuple(a.ball), a.gif, a.seconds))
