"""Command-conditioned bipedal walking environment (gymnasium) on the CAD-derived MuJoCo model.

Policy: 50 Hz, acts on the 12 leg servos as position offsets around a crouched default pose; arms/neck hold their pose.
Obs (what the real robot can measure): gyro, gravity vector (IMU), command, joint pos/vel (servo feedback), last action, gait clock.
Domain randomisation: friction, mass, servo gains, servo latency, push, IMU noise, encoder noise, start pose."""
import numpy as np, mujoco, gymnasium as gym
from gymnasium import spaces
from . import model as M
from .gait_ref import Ref, T as GAIT_T
import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'firmware'))
from humanoid.obs import build_obs, OBS_DIM

DT, DECIM = 0.002, 10                # 500 Hz physics, 50 Hz policy
GAIT_PERIOD = GAIT_T                    # s
ACT_SCALE = 0.15                     # rad
CMD_MAX = np.array([0.20, 0.08, 0.6])   # vx, vy, wz  (m/s, m/s, rad/s)
CMD_MIN = np.array([-0.08, -0.08, -0.6])

class WalkEnv(gym.Env):
    def __init__(self, randomize=True, seed=0, max_steps=1000, scene=None):
        self.m = M.build(dt=DT, scene=scene); self.d = mujoco.MjData(self.m); self.randomize = randomize; self.max_steps = max_steps
        self.rng = np.random.default_rng(seed)
        self.leg = M.leg_names(); self.names = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.m.njnt)]
        self.jid = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.names if n}
        self.qa = {n: self.m.jnt_qposadr[i] for n, i in self.jid.items()}; self.va = {n: self.m.jnt_dofadr[i] for n, i in self.jid.items()}
        self.aid = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.jid}
        self.default = M.default_pose(); self.all_j = [n for n in self.names if n]
        self.default_full = np.array([self.default.get(n, 0.0) for n in self.all_j])
        self.leg_idx = np.array([self.all_j.index(n) for n in self.leg])
        self.ref = Ref(); self.default_full[self.leg_idx] = self.ref.stand; self.phase0 = 0.0; self.qref = self.ref.stand.copy()
        self.lo = np.array([self.m.jnt_range[self.jid[n]][0] for n in self.all_j]); self.hi = np.array([self.m.jnt_range[self.jid[n]][1] for n in self.all_j])
        self.feet = M.foot_geoms(self.m); self.floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_body = {s: self.m.geom_bodyid[g] for s, g in self.feet.items()}
        self.nom = dict(kp=self.m.actuator_gainprm[:, 0].copy(), mass=self.m.body_mass.copy(), fric=self.m.geom_friction.copy())
        self.nl = len(self.leg)
        self.obs_dim = 3 + 3 + 3 + self.nl * 2 + self.nl + 2
        self.observation_space = spaces.Box(-10, 10, (self.obs_dim,), np.float32)
        self.action_space = spaces.Box(-1, 1, (self.nl,), np.float32)
        self.base_h = None

    # ---------- helpers
    def _quat_rot_inv(self, v):
        q = self.d.qpos[3:7]; R = np.zeros(9); mujoco.mju_quat2Mat(R, q); return R.reshape(3, 3).T @ v
    def _contacts(self):
        c = {"L": False, "R": False}
        for i in range(self.d.ncon):
            g1, g2 = self.d.contact[i].geom1, self.d.contact[i].geom2
            for s, g in self.feet.items():
                if (g1 == g and g2 == self.floor) or (g2 == g and g1 == self.floor): c[s] = True
        return c
    def _joint_state(self):
        q = np.array([self.d.qpos[self.qa[n]] for n in self.all_j]); qd = np.array([self.d.qvel[self.va[n]] for n in self.all_j]); return q, qd
    def _phase(self): return (self.phase0 + self.t * DT * DECIM / GAIT_PERIOD) % 1.0
    def _obs(self):
        q, qd = self._joint_state()
        if self.randomize: q = q + self.rng.normal(0, 0.005, q.shape); qd = qd + self.rng.normal(0, 0.05, qd.shape)
        gyro = self.d.qvel[3:6] + (self.rng.normal(0, 0.02, 3) if self.randomize else 0)
        g = self._quat_rot_inv(np.array([0, 0, -1.0])) + (self.rng.normal(0, 0.01, 3) if self.randomize else 0)
        return build_obs(gyro, g, self.cmd, q[self.leg_idx], self.qref, qd[self.leg_idx], self.last_a, self._phase())

    # ---------- gym API
    def reset(self, seed=None, options=None):
        if seed is not None: self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.m, self.d); self.t = 0
        self.m.actuator_gainprm[:, 0] = self.nom["kp"]; self.m.actuator_biasprm[:, 1] = -self.nom["kp"]
        self.m.body_mass[:] = self.nom["mass"]; self.m.geom_friction[:] = self.nom["fric"]
        if self.randomize:
            k = self.rng.uniform(0.75, 1.3, self.m.nu) * self.nom["kp"]; self.m.actuator_gainprm[:, 0] = k; self.m.actuator_biasprm[:, 1] = -k
            self.m.body_mass[:] = self.nom["mass"] * self.rng.uniform(0.9, 1.15, self.m.nbody)
            self.m.geom_friction[self.floor, 0] = self.rng.uniform(0.5, 1.3)
            self.latency = int(self.rng.integers(0, 2))
        else: self.latency = 0
        for n in self.all_j: self.d.qpos[self.qa[n]] = self.default_full[self.all_j.index(n)] + (self.rng.normal(0, 0.03) if self.randomize else 0)
        self.d.qpos[2] = 0.30; self.d.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.m, self.d)
        self.d.qpos[2] += 0.003 - self.foot_low()
        mujoco.mj_forward(self.m, self.d)
        self.cmd = self.rng.uniform(CMD_MIN, CMD_MAX) if self.randomize else np.array([0.12, 0.0, 0.0])
        if self.randomize and self.rng.random() < 0.6: self.cmd[0] = self.rng.uniform(0.06, CMD_MAX[0])
        if self.randomize and self.rng.random() < 0.1: self.cmd[:] = 0.0
        self.phase0 = float(self.rng.random()) if self.randomize else 0.0
        self.qref = self.ref.q(self._phase(), self.cmd)
        self.last_a = np.zeros(self.nl); self.prev_a = np.zeros(self.nl); self.last_qd = np.zeros(len(self.all_j))
        self.ctrl = self.default_full.copy(); self.air = {"L": 0.0, "R": 0.0}; self.x0 = self.d.qpos[0]
        self.push_t = int(self.rng.integers(100, 400)) if self.randomize else -1
        return self._obs(), {}

    def foot_low(self):
        lows = []
        for s, g in self.feet.items():
            R = self.d.geom_xmat[g].reshape(3, 3); half = self.m.geom_size[g]
            lows.append(self.d.geom_xpos[g, 2] - np.abs(R[2]) @ half)
        return min(lows)

    def step(self, a):
        a = np.clip(a, -1, 1).astype(np.float64)
        applied = self.prev_a if self.latency else a
        self.prev_a = a.copy()
        self.qref = self.ref.q(self._phase(), self.cmd)
        tgt = self.default_full.copy(); tgt[self.leg_idx] = self.qref + ACT_SCALE * applied
        tgt = np.clip(tgt, self.lo, self.hi)
        for n, i in zip(self.all_j, range(len(self.all_j))): self.d.ctrl[self.aid[n]] = tgt[i]
        if self.t == self.push_t: self.d.qvel[0:2] += self.rng.uniform(-0.25, 0.25, 2)
        for _ in range(DECIM): mujoco.mj_step(self.m, self.d)
        self.t += 1
        q, qd = self._joint_state(); c = self._contacts()
        r, parts = self._reward(a, q, qd, c)
        self.last_a = a; self.last_qd = qd
        up = self._quat_rot_inv(np.array([0, 0, 1.0]))[2]
        fell = up < 0.6 or self.d.subtree_com[1][2] < 0.17
        trunc = self.t >= self.max_steps
        if fell: r -= 10.0
        return self._obs(), float(r), bool(fell), bool(trunc), parts

    def _reward(self, a, q, qd, c):
        v_body = self._quat_rot_inv(self.d.qvel[0:3]); wz = self.d.qvel[5]
        ph = self._phase(); stance_L = ph < 0.5          # left stance for first half, right for second
        e_v = np.sum((self.cmd[:2] - v_body[:2]) ** 2); e_w = (self.cmd[2] - wz) ** 2
        cn = np.linalg.norm(self.cmd[:2]); prog = float(v_body[:2] @ self.cmd[:2] / cn) if cn > 1e-3 else 0.0
        r_vel = 1.0 * np.exp(-e_v / 0.01) + 0.6 * np.exp(-e_v / 0.05) + 0.8 * np.exp(-e_w / 0.15) + (6.0 * np.clip(prog, -0.05, cn) if cn > 0.03 else 0.0)
        moving = np.linalg.norm(self.cmd[:2]) > 0.02 or abs(self.cmd[2]) > 0.1
        # periodic contact schedule (rewards the intended alternating gait; zero command -> both feet down)
        want = {"L": stance_L or not moving, "R": (not stance_L) or not moving}
        r_gait = 1.2 * sum(1.0 if c[s] == want[s] else -0.3 for s in "LR") / 2 if moving else 0.3 * (c["L"] + c["R"]) / 2
        # foot clearance of the swing foot
        clear = 0.0
        for s in "LR":
            if moving and not want[s]:
                h = self.d.xpos[self.foot_body[s], 2]; clear += np.clip(h - 0.045, -0.02, 0.02)
        r_clear = 6.0 * clear
        g = self._quat_rot_inv(np.array([0, 0, -1.0])); p_orient = np.sum(g[:2] ** 2)
        p_ang = np.sum(self.d.qvel[3:5] ** 2); p_z = v_body[2] ** 2
        tau = np.abs(self.d.actuator_force[[self.aid[n] for n in self.leg]]); p_tau = np.sum((tau / 3.0) ** 2)
        p_act = np.sum((a - self.last_a) ** 2); p_acc = np.sum(((qd - self.last_qd) / (DT * DECIM)) ** 2) * 1e-5
        hip = [self.leg.index(n) for n in self.leg if "hip_yaw" in n or "hip_roll" in n]
        p_hip = np.sum((q[self.leg_idx][hip] - self.default_full[self.leg_idx][hip]) ** 2)
        slide = sum(np.linalg.norm(self.d.cvel[self.foot_body[s]][3:5]) for s in "LR" if c[s])
        
        pen = 6.0 * p_orient + 0.05 * p_ang + 1.0 * p_z + 0.03 * p_tau + 0.08 * p_act + min(p_acc, 0.3) + 1.0 * p_hip + 0.3 * slide
        r_ref = 0.6 * np.exp(-np.sum((q[self.leg_idx] - self.qref) ** 2) / 0.15)
        r = r_vel + r_gait + r_clear + r_ref + 1.0 - min(pen + 0.02 * np.sum(a ** 2), 1.5)
        return r, dict(vel=r_vel, gait=r_gait)
