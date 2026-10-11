"""Robustness benchmark under domain randomisation + pushes. python -m rl.benchmark out/rl/run3/final.zip"""
import sys, numpy as np
from stable_baselines3 import PPO
from .env import WalkEnv

CASES = {"forward 0.10": (0.10, 0, 0), "forward 0.18": (0.18, 0, 0), "sideways 0.06": (0, 0.06, 0), "turn 0.5": (0, 0, 0.5), "stand": (0, 0, 0), "back -0.06": (-0.06, 0, 0)}

def trial(policy, cmd, n=10, secs=12):
    falls, sp = 0, []
    for s in range(n):
        e = WalkEnv(randomize=True, seed=100 + s, max_steps=10**6); o, _ = e.reset(seed=100 + s); e.cmd = np.array(cmd, float); x0 = e.d.qpos[:2].copy()
        e.push_t = 150                                      # one shove at 3 s
        for i in range(int(secs * 50)):
            o, r, t, _, _ = e.step(policy(o))
            if t: falls += 1; break
        else:
            v = e._quat_rot_inv(np.r_[(e.d.qpos[:2] - x0) / secs, 0]); sp.append(v[:2])
    return falls, (np.mean(sp, axis=0).round(3) if sp else None)

if __name__ == "__main__":
    m = PPO.load(sys.argv[1], device="cpu"); base = len(sys.argv) > 2
    for name, c in CASES.items():
        f, v = trial((lambda o: np.zeros(12)) if base else (lambda o: m.predict(o, deterministic=True)[0]), c)
        print(f"{name:14s} cmd={c}  falls {f}/10  mean body velocity {v}")
