"""Reference walking gait used as a prior for RL (the policy learns residuals around it).
A cyclic gait (period T, phase 0..1: left stance for p<0.5, right stance for p>=0.5) is solved with the leg IK on a phase grid and
stored in linear form:   q_ref(p, cmd) = stand + g * (sway(p) + vx*Dx(p) + vy*Dy(p) + wz*Dz(p)),  g = clip(|cmd|/0.05, 0, 1)
Rebuild with `python -m rl.gait_ref`  (writes rl/gait_ref.npz)."""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "firmware"))
from humanoid.joints import load_joints
from humanoid.kinematics import Leg, LEG

T = 0.8; N = 100; SWAY = 40.0; LIFT = 22.0; DROP = 18.0; HIP_HALF = 52.0
NAMES = [f"{s}_{j}" for s in "LR" for j in LEG]
SIGN = np.array([load_joints()[n]["sign"] for n in NAMES], float)

def smooth(u): u = min(max(u, 0.0), 1.0); return 10 * u**3 - 15 * u**4 + 6 * u**5

def targets(p, vx, vy, wz, sway, lift):
    """Foot targets for the L/R ankle-roll origin in the pelvis frame (mm) plus foot yaw (deg), at phase p."""
    out = {}
    sx, sy, psi = vx * 1000 * T / 2, vy * 1000 * T / 2, math.degrees(wz * T / 2)
    cy = sway * math.sin(2 * math.pi * p)
    for s in "LR":
        ps = (p - (0.5 if s == "L" else 0.0)) % 1.0          # L swings in the second half, R in the first half: ps<0.5 -> swing
        swing = ps < 0.5; u = (ps if swing else ps - 0.5) / 0.5
        k = smooth(u)
        if swing: x, y, z, yaw = -sx / 2 + sx * k, -sy / 2 + sy * k, lift * math.sin(math.pi * u) ** 1.0, -psi / 2 + psi * k
        else:     x, y, z, yaw = +sx / 2 - sx * u, +sy / 2 - sy * u, 0.0, +psi / 2 - psi * u
        out[s] = (x, y, z, yaw)
    return cy, out

def solve(legs, p, vx, vy, wz, sway, lift, crouch):
    cy, t = targets(p, vx, vy, wz, sway, lift); q = []
    for s in "LR":
        x, y, z, yaw = t[s]; leg = legs[s]
        tgt = leg.neutral_foot + np.array([x, y - cy, z + crouch])
        d = leg.ik(tgt, yaw_deg=yaw); q += [d[n] for n in LEG]
    return np.radians(q) * SIGN        # body-frame angles -> sim (out-axis) joint angles

def build():
    legs = {s: Leg(load_joints(), s) for s in "LR"}
    P = np.arange(N) / N
    def table(vx, vy, wz, sway, lift):
        for l in legs.values(): l.last = np.zeros(6)
        return np.array([solve(legs, p, vx, vy, wz, sway, lift, DROP) for p in P])
    stand = table(0, 0, 0, 0, 0)
    sw = table(0, 0, 0, SWAY, LIFT) - stand
    dx = (table(0.1, 0, 0, SWAY, LIFT) - stand - sw) / 0.1
    dy = (table(0, 0.06, 0, SWAY, LIFT) - stand - sw) / 0.06
    dz = (table(0, 0, 0.5, SWAY, LIFT) - stand - sw) / 0.5
    for out in (os.path.join(HERE, "gait_ref.npz"), os.path.join(HERE, "..", "firmware", "config", "gait_ref.npz")):
        np.savez(out, stand=stand[0], sway=sw, dx=dx, dy=dy, dz=dz, period=T)
    return stand[0], sw, dx, dy, dz

from humanoid.gaitref import Ref      # noqa: E402  (runtime lookup shared with the robot firmware)

if __name__ == "__main__":
    st, sw, dx, dy, dz = build()
    print("stand deg", np.round(np.degrees(st), 1))
    print("sway peak deg", np.round(np.degrees(np.abs(sw).max(0)), 1))
    print("dx*0.2 peak deg", np.round(np.degrees(np.abs(dx).max(0) * 0.2), 1))
