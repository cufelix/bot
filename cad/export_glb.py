"""Compact GLB for the web viewer: one coloured mesh per part (Z-up, mm). Run after export_model."""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, trimesh
import robot as Rb

COL = {"servo": [40, 40, 48, 255], "printed": [225, 222, 210, 255], "elec": [38, 150, 76, 255], "misc": [128, 128, 128, 255]}
def main(out=os.path.join(os.path.dirname(__file__), "..", "out", "robot.glb")):
    parts, _ = Rb.build()
    sc = trimesh.Scene()
    for n, p in parts.items():
        v, t = p.shape.tessellate(0.25, 0.3)
        m = trimesh.Trimesh(np.array([(a.x, a.y, a.z) for a in v]), np.array(t), process=False)
        m.visual.face_colors = np.tile(COL.get(p.kind, COL["misc"]), (len(m.faces), 1))
        sc.add_geometry(m, node_name=n, geom_name=n)
    sc.export(out)
    print("glb", round(os.path.getsize(out) / 1e6, 2), "MB,", len(parts), "meshes")
if __name__ == "__main__":
    main()
