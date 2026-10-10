"""Numpy FK + static joint-torque model built from out/model.json (no CAD needed)."""
import json, os
import numpy as np

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))

def load(path=None):
    return json.load(open(path or os.path.join(HERE, "..", "out", "model.json")))

def rot(axis, ang_deg, origin, pt):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    t = np.radians(ang_deg); v = np.asarray(pt, float) - origin
    r = v * np.cos(t) + np.cross(a, v) * np.sin(t) + a * np.dot(a, v) * (1 - np.cos(t))
    return origin + r

class Robot:
    def __init__(self, model=None):
        self.m = model or load()
        self.J = self.m["joints"]
        self.parts = self.m["parts"]
        self.mass = sum(p["mass"] for p in self.parts) / 1000.0

    def chain(self, frame):
        out = []
        while frame != "base":
            out.append(frame); frame = self.J[frame]["parent"]
        return out

    def place(self, frame, pt, angles):
        for fr in self.chain(frame):          # innermost first
            a = angles.get(fr, 0.0)
            if a: pt = rot(self.J[fr]["out"], a, np.array(self.J[fr]["pos"], float), pt)
        return np.asarray(pt, float)

    def com(self, angles, only=None):
        tot, msum = np.zeros(3), 0.0
        for p in self.parts:
            if only is not None and not only(p): continue
            w = p["mass"] / 1000.0
            tot += w * self.place(p["frame"], p["com"], angles); msum += w
        return tot / msum, msum

    def downstream(self, joint):
        return lambda p: joint in self.chain(p["frame"])

    def torque_on(self, joint, angles, fz_point=None, fz=0.0, mass_filter=None):
        """Static torque magnitude (N*m) about `joint` axis from gravity of everything downstream
        (+ optional vertical contact force fz at fz_point, e.g. ground reaction through the stance foot)."""
        j = self.J[joint]; org = np.array(j["pos"], float); ax = np.asarray(j["out"], float)
        org = self.place(self.J[joint]["parent"], org, angles) if self.J[joint]["parent"] != "base" else org
        ax = self._axis_world(joint, angles)
        tau = 0.0
        for p in self.parts:
            if joint in self.chain(p["frame"]):
                c = self.place(p["frame"], p["com"], angles)
                tau += ax @ np.cross((c - org) / 1000.0, np.array([0, 0, -G * p["mass"] / 1000.0]))
        return tau

    def _axis_world(self, joint, angles):
        ax = np.asarray(self.J[joint]["out"], float)
        for fr in self.chain(self.J[joint]["parent"]) if self.J[joint]["parent"] != "base" else []:
            a = angles.get(fr, 0.0)
            if a:
                o = np.array(self.J[fr]["pos"], float)
                ax = rot(self.J[fr]["out"], a, np.zeros(3), ax)
        return ax

    def joint_origin(self, joint, angles):
        o = np.array(self.J[joint]["pos"], float)
        return self.place(self.J[joint]["parent"], o, angles) if self.J[joint]["parent"] != "base" else o

    def torque_with_ground(self, joint, angles, cop, force):
        tau = self.torque_on(joint, angles)
        org = self.joint_origin(joint, angles); ax = self._axis_world(joint, angles)
        tau += ax @ np.cross((np.array([cop[0], cop[1], 0.0]) - np.array([org[0], org[1], 0.0])) / 1000.0, np.array([0, 0, force]))
        return tau

    def foot_corners(self, side, angles):
        f = self.m["feet"]; y0 = self.J[side + "ankle_roll"]["pos"][1]
        pts = [(x, y0 + sy * f["half_w"], 0.0) for x in (f["x0"], f["x1"]) for sy in (-1, 1)]
        return np.array([self.place(side + "ankle_roll", p, angles) for p in pts])

    def foot_center(self, side, angles):
        return self.foot_corners(side, angles).mean(axis=0)
