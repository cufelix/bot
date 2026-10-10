"""Quasi-static walking pattern: weight shift over the stance foot, then swing the other foot forward.
Slow, but the CoM stays over the stance foot, so it needs only light feedback (see stabilizer.py).
`states()` yields (pelvis xy, feet world xyz, crouch fraction); `angles()` converts a state to joint angles via Leg.ik."""
import math
import numpy as np
from .kinematics import Leg

def minjerk(u): u = min(max(u, 0.0), 1.0); return 10 * u**3 - 15 * u**4 + 6 * u**5

class StaticWalker:
    def __init__(self, joints, hip_half=52.0, drop=25.0, step=40.0, lift=14.0, t_shift=1.2, t_swing=0.8,
                 com_lead=10.0, shift_frac=1.0, steps=4, hz=50):
        self.legs = {s: Leg(joints, s) for s in ("L", "R")}
        self.D, self.drop, self.step, self.lift = hip_half, drop, step, lift
        self.t_shift, self.t_swing, self.lead, self.shift_frac = t_shift, t_swing, com_lead, shift_frac
        self.n_steps, self.hz = steps, hz

    def angles(self, c, feet, crouch=1.0):
        """c = pelvis (x, y); feet = {'L': (x,y,z), 'R': ...} world coords, nominal stance feet at (0, +-D, 0)."""
        out = {}
        for s, leg in self.legs.items():
            f = np.array(feet[s], float)
            tgt = leg.neutral_foot + np.array([f[0] - c[0], f[1] - c[1], f[2] + self.drop * crouch]) - np.array([0, (self.D if s == "L" else -self.D), 0])
            for n, v in leg.ik(tgt).items(): out[f"{s}_{n}"] = float(v)
        return out

    def frames(self):
        """Open-loop joint-angle dicts at `hz`."""
        for c, feet, u in self.states():
            yield self.angles(c, feet, u)

    def states(self):
        feet = {"L": np.array([0.0, self.D, 0.0]), "R": np.array([0.0, -self.D, 0.0])}
        c = np.zeros(2)
        n = int(1.5 * self.hz)
        for k in range(n + 1):                                   # crouch
            yield c.copy(), {s: v.copy() for s, v in feet.items()}, minjerk(k / n)
        order = (["R", "L"] * self.n_steps)[: self.n_steps]      # swing foot per step
        for i, sw in enumerate(order):
            st = "L" if sw == "R" else "R"
            tgt = np.array([feet[st][0] + self.lead, feet[st][1] * self.shift_frac]); c0 = c.copy()
            n = int(self.t_shift * self.hz)
            for k in range(1, n + 1):                            # shift over stance foot
                c = c0 + (tgt - c0) * minjerk(k / n)
                yield c.copy(), {s: v.copy() for s, v in feet.items()}, 1.0
            if i == 0: x1 = feet[st][0] + self.step / 2
            elif i == len(order) - 1: x1 = feet[st][0]
            else: x1 = feet[st][0] + self.step
            f0 = feet[sw].copy(); f1 = np.array([x1, f0[1], 0.0])
            n = int(self.t_swing * self.hz)
            for k in range(1, n + 1):                            # swing
                u = k / n; f = f0 + (f1 - f0) * minjerk(u); f[2] = self.lift * 0.5 * (1 - math.cos(2 * math.pi * u))
                fe = {s: v.copy() for s, v in feet.items()}; fe[sw] = f
                yield c.copy(), fe, 1.0
            feet[sw] = f1
        tgt = np.array([(feet["L"][0] + feet["R"][0]) / 2, 0.0]); c0 = c.copy()
        n = int(self.t_shift * self.hz)
        for k in range(1, n + 1):                                # recentre
            c = c0 + (tgt - c0) * minjerk(k / n)
            yield c.copy(), {s: v.copy() for s, v in feet.items()}, 1.0
