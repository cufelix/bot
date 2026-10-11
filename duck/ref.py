"""Reference gait: upstream per-velocity polynomial fits (phase -> 16 joints, 16 joint vels, 2 contacts, base lin/ang vel)."""
import os, pickle, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))

class Ref:
    def __init__(self, path=os.path.join(HERE, "sim", "polynomial_coefficients.pkl")):
        raw = pickle.load(open(path, "rb")); keys = {tuple(map(float, k.split("_"))): v for k, v in raw.items()}
        self.dx, self.dy, self.dt_ = (np.array(sorted({k[i] for k in keys})) for i in range(3))
        v0 = next(iter(raw.values())); self.period, self.fps, self.n = v0["period"], v0["fps"], int(v0["period"] * v0["fps"])
        self._cache = {}
        self.c = {k: np.array([np.flip(c) for c in v["coefficients"].values()]) for k, v in keys.items()}   # (40, deg+1), highest first
    @staticmethod
    def _bracket(a, x):
        x = float(np.clip(x, a[0], a[-1])); i = int(np.clip(np.searchsorted(a, x) - 1, 0, len(a) - 2)); w = (x - a[i]) / (a[i + 1] - a[i])
        return (a[i], a[i + 1]), (1 - w, w)
    def frame(self, cmd, phase):
        """Trilinear blend of the 8 surrounding fitted gaits (the grid has no dy=0, so nearest-neighbour would drift sideways)."""
        (bx, wx), (by, wy), (bt, wt) = (self._bracket(a, x) for a, x in zip((self.dx, self.dy, self.dt_), cmd))
        out = 0.0
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    w = wx[i] * wy[j] * wt[k]
                    if w > 0: out = out + w * np.array([np.polyval(c, phase) for c in self.c[(float(bx[i]), float(by[j]), float(bt[k]))]])
        return out
    def table(self, cmd):
        """(n, 40) frames for one command, cached: a command is constant within an episode."""
        k = tuple(np.round(cmd, 4))
        if k not in self._cache:
            if len(self._cache) > 512: self._cache.clear()
            self._cache[k] = np.array([self.frame(cmd, i / self.n) for i in range(self.n)])
        return self._cache[k]
    def _at(self, cmd, phase): return self.table(cmd)[int(phase * self.n) % self.n]
    def legs(self, cmd, phase): f = self._at(cmd, phase); return np.concatenate([f[:5], f[11:16]])     # drop neck/head/antennas
    def contacts(self, cmd, phase): return self._at(cmd, phase)[32:34]

if __name__ == "__main__":
    r = Ref(); print("period", r.period, "n", r.n, "dx", r.dx, "dy", r.dy, "dth", r.dt_)
    for p in (0, .25, .5, .75): print(p, r.legs((0.15, 0, 0), p).round(2), r.contacts((0.15, 0, 0), p).round(2))
