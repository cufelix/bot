"""PPO training (stable-baselines3, CPU). Usage: python -m rl.train --steps 40e6 --out out/rl/run1"""
import argparse, os, sys, time
import numpy as np, torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.monitor import Monitor
from .env import WalkEnv
from duck.env import DuckEnv
ENVS = dict(humanoid=WalkEnv, duck=DuckEnv)
KIND = "humanoid"

class Log(BaseCallback):
    def __init__(self): super().__init__(); self.t0 = time.time()
    def _on_rollout_end(self):
        eps = [e for e in self.model.ep_info_buffer]
        if eps:
            print(f"{self.num_timesteps/1e6:6.2f}M  ep_len {np.mean([e['l'] for e in eps]):6.1f}  ep_rew {np.mean([e['r'] for e in eps]):8.1f}  {time.time()-self.t0:6.0f}s", flush=True)
    def _on_step(self): return True

def make(i): 
    def f(): return Monitor(ENVS[KIND](seed=i))
    return f

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--steps", type=float, default=40e6); ap.add_argument("--out", default="out/rl/run1")
    ap.add_argument("--envs", type=int, default=8); ap.add_argument("--resume", default=None); ap.add_argument("--lr", type=float, default=None); ap.add_argument("--robot", default="humanoid", choices=list(ENVS)); a = ap.parse_args()
    global KIND; KIND = a.robot
    os.makedirs(a.out, exist_ok=True); torch.set_num_threads(3)
    env = SubprocVecEnv([make(i) for i in range(a.envs)])
    kw = dict(policy_kwargs=dict(log_std_init=-1.5, net_arch=dict(pi=[256, 128, 64], vf=[256, 128, 64]), activation_fn=torch.nn.ELU), n_steps=128, batch_size=4096,
              n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, ent_coef=0.002, clip_range=0.2, device="cpu", verbose=0)
    model = PPO.load(a.resume, env=env, device="cpu", **({"custom_objects": {"learning_rate": a.lr, "lr_schedule": (lambda _: a.lr)}} if a.lr else {})) if a.resume else PPO("MlpPolicy", env, **kw)
    model.learn(int(a.steps), callback=[Log(), CheckpointCallback(2_000_000 // a.envs, a.out, name_prefix="ppo")], reset_num_timesteps=not a.resume)
    model.save(os.path.join(a.out, "final"))

if __name__ == "__main__":
    main()
