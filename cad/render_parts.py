"""Per-part renders (iso / front / top) from the print-oriented STLs -> docs/img/parts/<part>.png + contact sheet.
Plus hero renders of the full robot. Run: python cad/render_parts.py"""
import os, sys, glob
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

LIGHT = np.array([0.4, -0.5, 0.75]); LIGHT /= np.linalg.norm(LIGHT)
BASE = np.array([0.88, 0.86, 0.80])

def draw(ax, T, elev, azim):
    ax.view_init(elev=elev, azim=azim)
    e, a = np.radians(elev), np.radians(azim)
    cam = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
    sh = 0.35 + 0.65 * np.clip(np.abs(n @ LIGHT), 0, 1)
    o = np.argsort(T.mean(axis=1) @ cam)
    ax.add_collection3d(Poly3DCollection(T[o], facecolors=np.clip(BASE[None] * sh[o, None], 0, 1), edgecolors="none"))
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0); c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=1.0); ax.set_axis_off()

VIEWS = [("iso", 25, 35), ("front", 0, 0), ("side", 0, 90), ("top", 90, 0)]

def main():
    out = os.path.join("docs", "img", "parts"); os.makedirs(out, exist_ok=True)
    files = sorted(glob.glob("out/stl_print/*.stl")); sheet = []
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        m = trimesh.load(f); T = m.vertices[m.faces]; dims = m.bounds[1] - m.bounds[0]
        fig = plt.figure(figsize=(14, 3.8), dpi=100)
        for i, (_, el, az) in enumerate(VIEWS): draw(fig.add_subplot(1, 4, i + 1, projection="3d"), T, el, az)
        fig.suptitle(f"{name}   {dims[0]:.0f} x {dims[1]:.0f} x {dims[2]:.0f} mm   {m.volume / 1000:.1f} cm3", y=0.97, fontsize=11)
        plt.subplots_adjust(0, 0, 1, 0.92, 0, 0); fig.savefig(os.path.join(out, name + ".png"), facecolor="white"); plt.close(fig)
        sheet.append((name, T))
    cols = 5; rows = -(-len(sheet) // cols)
    fig = plt.figure(figsize=(4 * cols, 4 * rows), dpi=90)
    for i, (name, T) in enumerate(sheet):
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d"); draw(ax, T, 25, 35); ax.set_title(name, fontsize=9, y=0.9)
    plt.subplots_adjust(0, 0, 1, 1, 0, 0); fig.savefig(os.path.join("docs", "img", "parts_sheet.png"), facecolor="white"); plt.close(fig)
    print(f"rendered {len(sheet)} parts")

if __name__ == "__main__":
    main()
