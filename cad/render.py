"""Matplotlib preview renders (front / side / iso) of the posed robot -> docs/img/*.png"""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import robot as Rb

COL = {"servo": (0.18, 0.18, 0.22), "printed": (0.88, 0.86, 0.80), "elec": (0.15, 0.6, 0.3), "misc": (0.5, 0.5, 0.5)}
LIGHT = np.array([0.4, -0.5, 0.75]); LIGHT /= np.linalg.norm(LIGHT)

def tris(shape):
    v, t = shape.tessellate(0.4, 0.3)
    v = np.array([(p.x, p.y, p.z) for p in v]); return v[np.array(t)]

def render(parts, J, angles, views, out, size=(5, 8)):
    meshes = []
    for n, p in parts.items():
        if n.startswith(("camera",)): pass
        meshes.append((p.kind, tris(Rb.posed(p, J, angles))))
    fig = plt.figure(figsize=(size[0] * len(views), size[1]), dpi=110)
    for i, (elev, azim) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        ax.view_init(elev=elev, azim=azim)
        # camera direction for painter-sorting
        e, a = np.radians(elev), np.radians(azim)
        cam = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        polys, cols, depth = [], [], []
        for kind, T in meshes:
            nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nl = np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-9; nrm /= nl
            sh = 0.35 + 0.65 * np.clip(np.abs(nrm @ LIGHT), 0, 1)
            base = np.array(COL.get(kind, COL["misc"]))
            polys.append(T); cols.append(base[None, :] * sh[:, None]); depth.append((T.mean(axis=1) @ cam))
        polys = np.concatenate(polys); cols = np.concatenate(cols); depth = np.concatenate(depth)
        o = np.argsort(depth)
        pc = Poly3DCollection(polys[o], facecolors=np.clip(cols[o], 0, 1), edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
        ax.set_xlim(-200, 200); ax.set_ylim(-200, 200); ax.set_zlim(0, 620)
        ax.set_box_aspect((400, 400, 620), zoom=1.25); ax.set_axis_off()
    plt.subplots_adjust(0, 0, 1, 1, 0, 0)
    fig.savefig(out, facecolor="white"); plt.close(fig)

if __name__ == "__main__":
    parts, J = Rb.build()
    os.makedirs("docs/img", exist_ok=True)
    render(parts, J, {}, [(8, 0), (8, 90), (20, 35)], "docs/img/robot_neutral.png")
    crouch = {"L_hip_pitch": -20, "L_knee": 40, "L_ankle_pitch": -20, "R_hip_pitch": 20, "R_knee": -40, "R_ankle_pitch": 20,
              "L_sh_pitch": -40, "R_sh_pitch": 40, "L_elbow": -50, "R_elbow": 50}
    render(parts, J, crouch, [(8, 35)], "docs/img/robot_crouch.png", size=(6, 8))
    print("rendered")
