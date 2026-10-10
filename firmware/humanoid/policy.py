"""On-robot walking controller: ONNX residual policy on top of the reference gait, 50 Hz.
Mirrors rl/env.py exactly (same observation layout via obs.py). Safety: fall detection -> e-stop, command slew limiting, health checks."""
import time
import numpy as np
from .gaitref import Ref
from .obs import build_obs, ACT_SCALE
from .poses import ARMS_SIM
from .robot import SafetyTrip

LEG = [f"{s}_{j}" for s in "LR" for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")]
HZ = 50
CMD_LIMIT = np.array([0.20, 0.08, 0.6])
CMD_SLEW = np.array([0.15, 0.15, 1.0])          # max change per second
FALL_UP = 0.6                                   # cos(tilt) below this -> e-stop (same threshold as training termination)

def quat_to_gravity_body(qi, qj, qk, qr):
    """Gravity direction (unit) in the body frame from a quaternion that rotates body->world."""
    r00 = 1 - 2 * (qj * qj + qk * qk); r01 = 2 * (qi * qj - qk * qr); r02 = 2 * (qi * qk + qj * qr)
    r10 = 2 * (qi * qj + qk * qr); r11 = 1 - 2 * (qi * qi + qk * qk); r12 = 2 * (qj * qk - qi * qr)
    r20 = 2 * (qi * qk - qj * qr); r21 = 2 * (qj * qk + qi * qr); r22 = 1 - 2 * (qi * qi + qj * qj)
    R = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
    return R.T @ np.array([0.0, 0.0, -1.0])

class OnnxPolicy:
    def __init__(self, path):
        import onnxruntime as ort
        self.s = ort.InferenceSession(path, providers=["CPUExecutionProvider"]); self.inp = self.s.get_inputs()[0].name
    def __call__(self, obs): return self.s.run(None, {self.inp: obs[None].astype(np.float32)})[0][0]

class WalkController:
    """robot: humanoid.robot.Robot;  policy: callable obs->action[12];  imu: object with .read_full() -> (gyro_rad_s[3], gravity_body[3])."""
    def __init__(self, robot, policy, imu, ref=None):
        self.r, self.policy, self.imu = robot, policy, imu
        self.ref = ref or Ref(); self.sign = np.array([robot.joints[n]["sign"] for n in LEG], float)
        self.arm_body = {n: float(np.degrees(v)) * robot.joints[n]["sign"] for n, v in ARMS_SIM.items()}
        self.phase = 0.0; self.last_a = np.zeros(12); self.cmd = np.zeros(3); self.cmd_goal = np.zeros(3)
        self.q_prev = None; self.qref = self.ref.stand.copy(); self.t_health = 0.0

    def set_command(self, vx, vy, wz): self.cmd_goal = np.clip([vx, vy, wz], -CMD_LIMIT, CMD_LIMIT)

    def _leg_state(self):
        ang = self.r.read_angles()
        if any(n not in ang for n in LEG): raise SafetyTrip("missing servo feedback")
        return np.radians([ang[n] for n in LEG]) * self.sign                # body-frame deg -> sim (out-axis) rad

    def observe(self, q=None, qd=None):
        q = self._leg_state() if q is None else q
        if qd is None: qd = np.zeros(12) if self.q_prev is None else (q - self.q_prev) * HZ
        self.q_prev = q.copy()
        gyro, g = self.imu.read_full()
        return build_obs(gyro, g, self.cmd, q, self.qref, qd, self.last_a, self.phase), g

    def step(self):
        """One 50 Hz control tick: command the targets for this phase, advance the clock, return the next observation's action."""
        dt = 1.0 / HZ
        self.cmd = self.cmd + np.clip(self.cmd_goal - self.cmd, -CMD_SLEW * dt, CMD_SLEW * dt)
        self.qref = self.ref.q(self.phase, self.cmd)
        tgt = self.qref + ACT_SCALE * self.last_a                             # act on the action chosen last tick (same order as training)
        goal = {n: float(np.degrees(v)) * s for n, v, s in zip(LEG, tgt, self.sign)}
        goal.update(self.arm_body)
        self.r.send(goal)
        self.phase = (self.phase + dt / self.ref.period) % 1.0
        obs, g = self.observe()
        if g[2] > -FALL_UP:                                                    # gravity z (body) -> -1 when upright
            self.r.emergency_stop(); raise SafetyTrip(f"fall detected (up={-g[2]:.2f})")
        self.last_a = np.clip(self.policy(obs), -1.0, 1.0)
        return self.last_a

    def run(self, seconds, command_fn=None):
        t0 = time.time(); k = 0
        while time.time() - t0 < seconds:
            if command_fn: self.set_command(*command_fn(time.time() - t0))
            self.step(); k += 1
            if time.time() - self.t_health > 0.5: self.r.check_health(); self.t_health = time.time()
            time.sleep(max(0.0, t0 + k / HZ - time.time()))
