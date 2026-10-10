"""Feetech-bus emulator backed by MuJoCo physics: lets the REAL firmware stack (Robot, ticks, signs, limits, WalkController) drive the simulated robot.
A goal_pos sync-write advances physics by one control tick (20 ms); reads return the simulated servo state."""
import numpy as np, mujoco
from . import firmware_path  # noqa: F401  (adds firmware/ to sys.path)
from humanoid import feetech as F
from humanoid.joints import load_joints
from . import model as M
from .env import DT, DECIM

class SimWorld:
    def __init__(self, scene=None, seed=0):
        self.m = M.build(dt=DT, scene=scene); self.d = mujoco.MjData(self.m)
        self.joints = load_joints(); self.rng = np.random.default_rng(seed)
        self.qa = {n: self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in self.joints}
        self.va = {n: self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in self.joints}
        self.aid = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.joints}
        self.torque = False; self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        for n, v in M.default_pose().items(): self.d.qpos[self.qa[n]] = v
        self.d.qpos[2] = 0.3; mujoco.mj_forward(self.m, self.d)
        low = min(self.d.geom_xpos[g, 2] - np.abs(self.d.geom_xmat[g].reshape(3, 3)[2]) @ self.m.geom_size[g] for g in M.foot_geoms(self.m).values())
        self.d.qpos[2] += 0.003 - low; mujoco.mj_forward(self.m, self.d)
        for n in self.joints: self.d.ctrl[self.aid[n]] = self.d.qpos[self.qa[n]]

    def tick_to_rad(self, name, tick):
        j = self.joints[name]; return np.radians((tick - F.CENTER - j["offset"]) * 360.0 / F.TICKS * j["dir"])
    def rad_to_tick(self, name, rad):
        j = self.joints[name]; return int(round(F.CENTER + j["offset"] + j["dir"] * np.degrees(rad) * F.TICKS / 360.0))

    def step_physics(self):
        for _ in range(DECIM): mujoco.mj_step(self.m, self.d)

    def gravity_body(self):
        R = np.zeros(9); mujoco.mju_quat2Mat(R, self.d.qpos[3:7]); return R.reshape(3, 3).T @ np.array([0, 0, -1.0])

class SimIMU:
    def __init__(self, world): self.w = world
    def read_full(self): return self.w.d.qvel[3:6].copy(), self.w.gravity_body()

class SimBus:
    """One of the two robot buses ('A' legs, 'B' upper). Bus A also advances the physics when it receives goals (it is written every tick)."""
    def __init__(self, world, ids, advance=False):
        self.w, self.advance = world, advance
        self.by_id = {j["id"]: n for n, j in world.joints.items() if j["id"] in ids}
        self.fake = F.FakeBus(ids)               # holds the register file for configure()/ping/health
    def close(self): pass
    def transact(self, sid, inst, params=b"", reply_len=0):
        if inst == F.SYNC_WRITE:
            addr, n = params[0], params[1]; rest = params[2:]
            for k in range(0, len(rest), n + 1):
                i = rest[k]; val = int.from_bytes(rest[k + 1:k + 1 + n], "little")
                if addr == F.REG["torque_enable"][0]: self.w.torque = bool(val)
                if addr == F.REG["goal_pos"][0] and i in self.by_id:
                    name = self.by_id[i]; self.w.d.ctrl[self.w.aid[name]] = self.w.tick_to_rad(name, val)
            if self.advance and addr == F.REG["goal_pos"][0]: self.w.step_physics()
            return self.fake.transact(sid, inst, params, reply_len)
        if inst == F.SYNC_READ:
            addr, n = params[0], params[1]; out = b""
            for i in params[2:]:
                name = self.by_id.get(i)
                if name is None: continue
                if addr == F.REG["present_pos"][0]: data = F.pack(2, int(np.clip(self.w.rad_to_tick(name, self.w.d.qpos[self.w.qa[name]]), 0, 4095)))
                else: data = bytes(self.fake.mem[i][addr:addr + n])
                out += F.packet(i, 0, data)
            return out
        return self.fake.transact(sid, inst, params, reply_len)

def make(scene=None):
    """-> (robot, imu, world) wired through the real firmware classes."""
    from humanoid.robot import Robot
    w = SimWorld(scene); joints = w.joints
    ids = {"A": [j["id"] for j in joints.values() if j["bus"] == "A"], "B": [j["id"] for j in joints.values() if j["bus"] == "B"]}
    robot = Robot(joints=joints, buses={"A": SimBus(w, ids["A"], advance=True), "B": SimBus(w, ids["B"])})
    return robot, SimIMU(w), w
