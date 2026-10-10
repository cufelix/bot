"""Reference walking gait lookup (numpy only). Table built offline by rl/gait_ref.py:  q_ref(p, cmd) = stand + g*(sway(p) + vx*Dx(p) + vy*Dy(p) + wz*Dz(p))."""
import os
import numpy as np

DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "gait_ref.npz")

class Ref:
    def __init__(self, path=DEFAULT):
        z = np.load(path)
        self.stand, self.sway, self.dx, self.dy, self.dz = (z[k] for k in ("stand", "sway", "dx", "dy", "dz"))
        self.period = float(z["period"]); self.n = len(self.sway)

    def q(self, phase, cmd):
        """12 leg joint angles (rad, sim/out-axis convention: L hip_yaw..ankle_roll, then R)."""
        f = phase * self.n; i = int(f) % self.n; j = (i + 1) % self.n; w = f - int(f)
        lerp = lambda a: a[i] * (1 - w) + a[j] * w
        g = min(1.0, float(np.linalg.norm(cmd)) / 0.05)
        return self.stand + g * (lerp(self.sway) + cmd[0] * lerp(self.dx) + cmd[1] * lerp(self.dy) + cmd[2] * lerp(self.dz))
