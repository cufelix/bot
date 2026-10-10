"""Leg forward/inverse kinematics from the CAD joint table (body-frame angles about canonical axes)."""
import numpy as np
from scipy.optimize import least_squares
from .joints import body_limits

LEG = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]
AXES = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}

def rotmat(axis, deg):
    a = AXES[axis]; t = np.radians(deg); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K

class Leg:
    def __init__(self, joints, side):
        self.side = side
        self.j = [joints[f"{side}_{n}"] for n in LEG]
        self.pos = [np.array(j["pos_mm"], float) for j in self.j]
        self.axis = [j["axis"] for j in self.j]
        self.bounds = [body_limits(j) for j in self.j]
        self.neutral_foot = self.pos[-1].copy()       # ankle-roll axis origin at zero pose
        self.last = np.zeros(6)

    def fk(self, q_deg):
        """Returns (ankle-roll origin position mm, foot rotation matrix) in the body frame."""
        R = np.eye(3); p = self.pos[-1].copy()
        Rs = [rotmat(a, q) for a, q in zip(self.axis, q_deg)]
        # point on foot: apply innermost (ankle roll) first, outward to hip yaw
        for k in range(5, -1, -1):
            p = self.pos[k] + Rs[k] @ (p - self.pos[k])
        Rt = np.eye(3)
        for r in Rs: Rt = Rt @ r
        return p, Rt

    def ik(self, target_mm, yaw_deg=0.0, q0=None, tol=0.05):
        """Ankle-roll origin -> target, foot kept level (roll=pitch=0, yaw=yaw_deg). Raises if unreachable."""
        Rt = rotmat("Z", yaw_deg)
        lo = np.array([b[0] for b in self.bounds]) + 0.5; hi = np.array([b[1] for b in self.bounds]) - 0.5
        def res(q):
            p, R = self.fk(q); E = R @ Rt.T
            e = 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
            return np.concatenate([p - target_mm, 50.0 * e])
        x0 = np.clip(self.last if q0 is None else q0, lo + 1e-6, hi - 1e-6)
        s = least_squares(res, x0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
        if np.linalg.norm(s.fun[:3]) > 3.0 or np.linalg.norm(s.fun[3:]) > 1.0:
            raise ValueError(f"IK unreachable ({np.linalg.norm(s.fun[:3]):.1f} mm err) for {target_mm}")
        self.last = s.x
        return dict(zip(LEG, s.x))
