"""Command-conditioned walking env for the Open Duck Mini v2: PPO learns residuals around the upstream reference gait.
50 Hz policy / 500 Hz physics. Acts on the 10 leg servos; head holds its pose. Randomised servo gain, mass, friction, latency, pushes, sensor noise."""
import numpy as np, mujoco, gymnasium as gym
from gymnasium import spaces
from . import model as M
from .ref import Ref

DT, DECIM = 0.002, 10
ACT_SCALE = 0.25
CMD_MAX = np.array([0.20, 0.10, 0.7]); CMD_MIN = np.array([-0.10, -0.10, -0.7])
CMD_SCALE = np.array([2.0, 2.0, 0.5])
OBS_DIM = 3 + 3 + 3 + 10 + 10 + 10 + 2 + 10

class DuckEnv(gym.Env):
    def __init__(self, randomize=True, seed=0, max_steps=1000):
        self.m = M.build(DT); self.d = mujoco.MjData(self.m); self.randomize = randomize; self.max_steps = max_steps
        self.rng = np.random.default_rng(seed); self.ref = Ref()
        self.home = M.home(self.m)[7:]; self.leg = np.array(M.LEGS); self.home_leg = self.home[self.leg]
        self.lo, self.hi = self.m.jnt_range[1:, 0], self.m.jnt_range[1:, 1]
        self.feet = [self.m.geom("left_foot_bottom_tpu").id, self.m.geom("right_foot_bottom_tpu").id]
        self.floor = self.m.geom("floor").id; self.foot_site = [self.m.site("left_foot").id, self.m.site("right_foot").id]
        self.nom = dict(kp=self.m.actuator_gainprm[:, 0].copy(), mass=self.m.body_mass.copy(), fric=self.m.geom_friction.copy())
        self.observation_space = spaces.Box(-10, 10, (OBS_DIM,), np.float32); self.action_space = spaces.Box(-1, 1, (10,), np.float32)

    def _up(self): return self.d.xmat[1].reshape(3, 3)[2]                       # world z axis in body frame (gravity direction = -this)
    def _grav(self): return -self._up()
    def _phase(self): return (self.phase0 + self.t / self.ref.n) % 1.0
    def _contacts(self):
        c = [False, False]
        for i in range(self.d.ncon):
            a, b = self.d.contact[i].geom1, self.d.contact[i].geom2
            for k, g in enumerate(self.feet):
                if {a, b} == {g, self.floor}: c[k] = True
        return c
    def _moving(self): return np.linalg.norm(self.cmd[:2]) > 0.02 or abs(self.cmd[2]) > 0.1
    def _qref(self):
        if not self._moving(): return self.home_leg
        w = min(1.0, self.t / 25.0)                       # ease from the home pose into the gait
        return (1 - w) * self.home_leg + w * self.ref.legs(self.cmd, self._phase())

    def _obs(self):
        q = self.d.qpos[7:][self.leg]; qd = self.d.qvel[6:][self.leg]; n = self.randomize
        q = q + (self.rng.normal(0, 0.01, 10) if n else 0); qd = qd + (self.rng.normal(0, 0.05, 10) if n else 0)
        gyro = self.d.qvel[3:6] + (self.rng.normal(0, 0.03, 3) if n else 0); g = self._grav() + (self.rng.normal(0, 0.015, 3) if n else 0)
        ph = 2 * np.pi * self._phase()
        return np.concatenate([gyro * 0.25, g, self.cmd * CMD_SCALE, q - self.home_leg, qd * 0.05, self.last_a, [np.sin(ph), np.cos(ph)], self.qref - self.home_leg]).astype(np.float32)

    def reset(self, seed=None, options=None):
        if seed is not None: self.rng = np.random.default_rng(seed)
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("home").id); self.t = 0
        self.m.actuator_gainprm[:, 0] = self.nom["kp"]; self.m.actuator_biasprm[:, 1] = -self.nom["kp"]
        self.m.body_mass[:] = self.nom["mass"]; self.m.geom_friction[:] = self.nom["fric"]; self.latency = 0
        if self.randomize:
            k = self.nom["kp"] * self.rng.uniform(0.75, 1.3, self.m.nu); self.m.actuator_gainprm[:, 0] = k; self.m.actuator_biasprm[:, 1] = -k
            self.m.body_mass[:] = self.nom["mass"] * self.rng.uniform(0.9, 1.15, self.m.nbody)
            self.m.geom_friction[self.floor, 0] = self.rng.uniform(0.4, 1.2); self.latency = int(self.rng.integers(0, 2))
            self.d.qpos[7:] += self.rng.normal(0, 0.02, self.d.qpos[7:].shape)
        self.cmd = self.rng.uniform(CMD_MIN, CMD_MAX) if self.randomize else np.array([0.12, 0, 0])
        if self.randomize and self.rng.random() < 0.5: self.cmd[0] = self.rng.uniform(0.05, CMD_MAX[0])
        if self.randomize and self.rng.random() < 0.1: self.cmd[:] = 0
        self.phase0 = self.rng.integers(0, self.ref.n) / self.ref.n if self.randomize else 0.0
        self.qref = self._qref(); self.last_a = np.zeros(10); self.prev_a = np.zeros(10); self.last_qd = np.zeros(10)
        self.push_t = int(self.rng.integers(100, 400)) if self.randomize else -1
        mujoco.mj_forward(self.m, self.d)
        return self._obs(), {}

    def step(self, a):
        a = np.clip(a, -1, 1).astype(np.float64); applied = self.prev_a if self.latency else a; self.prev_a = a.copy()
        self.qref = self._qref()
        tgt = self.home.copy(); tgt[self.leg] = self.qref + ACT_SCALE * applied; self.d.ctrl[:] = np.clip(tgt, self.lo, self.hi)
        if self.t == self.push_t: self.d.qvel[0:2] += self.rng.uniform(-0.3, 0.3, 2)
        for _ in range(DECIM): mujoco.mj_step(self.m, self.d)
        self.t += 1; c = self._contacts(); r, info = self._reward(a, c)
        self.last_a = a; self.last_qd = self.d.qvel[6:][self.leg].copy()
        fell = self._up()[2] < 0.6 or self.d.subtree_com[1][2] < 0.12
        if fell: r -= 10.0
        return self._obs(), float(r), bool(fell), self.t >= self.max_steps, info

    def _reward(self, a, c):
        R = self.d.xmat[1].reshape(3, 3); v = R.T @ self.d.qvel[0:3]; wz = self.d.qvel[5]
        e_v = np.sum((self.cmd[:2] - v[:2]) ** 2); e_w = (self.cmd[2] - wz) ** 2
        cn = np.linalg.norm(self.cmd[:2]); prog = float(v[:2] @ self.cmd[:2] / cn) if cn > 1e-3 else 0.0
        r_vel = 1.0 * np.exp(-e_v / 0.01) + 0.6 * np.exp(-e_v / 0.05) + 0.8 * np.exp(-e_w / 0.15) + (6.0 * np.clip(prog, -0.05, cn) if cn > 0.03 else 0.0)
        mov = self._moving(); rc = self.ref.contacts(self.cmd, self._phase()) > 0.5 if mov else np.array([True, True])
        r_gait = 1.0 * sum(1.0 if c[k] == rc[k] else -0.3 for k in range(2)) / 2 if mov else 0.3 * sum(c) / 2
        clear = sum(np.clip(self.d.site_xpos[self.foot_site[k], 2] - 0.03, -0.015, 0.015) for k in range(2) if mov and not rc[k])
        q = self.d.qpos[7:][self.leg]; qd = self.d.qvel[6:][self.leg]
        r_ref = 0.8 * np.exp(-np.sum((q - self.qref) ** 2) / 0.1)
        g = self._grav(); p_orient = np.sum(g[:2] ** 2); p_ang = np.sum(self.d.qvel[3:5] ** 2); p_z = v[2] ** 2
        tau = np.abs(self.d.actuator_force[self.leg]); p_tau = np.sum((tau / 3.0) ** 2)
        p_act = np.sum((a - self.last_a) ** 2); p_acc = np.sum(((qd - self.last_qd) / (DT * DECIM)) ** 2) * 1e-5
        pen = 5.0 * p_orient + 0.05 * p_ang + 1.0 * p_z + 0.02 * p_tau + 0.08 * p_act + min(p_acc, 0.3)
        return r_vel + r_gait + 5.0 * clear + r_ref + 1.0 - min(pen + 0.02 * np.sum(a ** 2), 1.5), dict(vel=r_vel)

if __name__ == "__main__":      # open-loop reference replay: does the prior walk by itself?
    e = DuckEnv(randomize=False); e.reset(); x0 = e.d.qpos[0]; n = 0
    for _ in range(300):
        _, r, term, trunc, _ = e.step(np.zeros(10)); n += 1
        if term: break
    print("steps", n, "fell", term, "dx", round(e.d.qpos[0] - x0, 3), "m  (cmd 0.12 m/s -> expect ~", round(0.12 * n * 0.02, 2), ")")
