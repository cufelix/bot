"""Generate sim/robot/humanoid.urdf (+ STL meshes) from the CAD. Run: python sim/make_urdf.py"""
import os, sys, json
import numpy as np
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "cad"))
import cadquery as cq
import robot as Rb

OUT = os.path.join(ROOT, "sim", "robot")
VEL = 4.5   # rad/s (STS3215 0.222 s/60deg @12V)
STALL = {"STS3215": 2.9, "STS3250": 4.9}

def box_inertia(m, d):
    x, y, z = (v / 1000.0 for v in d)
    return np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])

def main():
    model = json.load(open(os.path.join(ROOT, "out", "model.json")))
    J = model["joints"]
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    parts, _ = Rb.build()
    for n, p in parts.items():
        cq.exporters.export(cq.Workplane("XY").add(p.shape), os.path.join(OUT, "meshes", n + ".stl"), tolerance=0.3, angularTolerance=0.3)
    frames = ["base"] + list(J)
    def origin(f): return np.zeros(3) if f == "base" else np.array(J[f]["pos"], float)
    L = ['<?xml version="1.0"?>', '<robot name="humanoid">',
         '<mujoco><compiler meshdir="meshes" balanceinertia="true" discardvisual="false" fusestatic="false"/></mujoco>']
    for f in frames:
        o = origin(f)
        ps = [p for p in model["parts"] if p["frame"] == f]
        m = sum(p["mass"] for p in ps) / 1000.0
        com = sum(np.array(p["com"]) * p["mass"] for p in ps) / (m * 1000.0)
        I = np.zeros((3, 3))
        for p in ps:
            mi = p["mass"] / 1000.0; r = (np.array(p["com"]) - com) / 1000.0
            I += box_inertia(mi, p["bbox"]) + mi * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        I = np.maximum(I, 1e-7 * np.eye(3)) if np.all(np.linalg.eigvalsh(I) > 0) else I + 1e-6 * np.eye(3)
        c = (com - o) / 1000.0
        L.append(f'<link name="{f}_link"><inertial><origin xyz="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}"/><mass value="{m:.4f}"/>'
                 f'<inertia ixx="{I[0,0]:.3e}" iyy="{I[1,1]:.3e}" izz="{I[2,2]:.3e}" ixy="{I[0,1]:.3e}" ixz="{I[0,2]:.3e}" iyz="{I[1,2]:.3e}"/></inertial>')
        for p in ps:
            if p["kind"] == "misc": continue
            vo = -o / 1000.0
            L.append(f'<visual><origin xyz="{vo[0]:.5f} {vo[1]:.5f} {vo[2]:.5f}"/><geometry><mesh filename="{p["name"]}.stl" scale="0.001 0.001 0.001"/></geometry></visual>')
        if f.endswith("ankle_roll"):    # foot collision box
            ft = model["feet"]; y0 = 0.0
            cx = (ft["x0"] + ft["x1"]) / 2 - o[0]; cz = 4.0 - o[2]
            L.append(f'<collision><origin xyz="{cx/1000:.5f} 0 {cz/1000:.5f}"/><geometry><box size="{(ft["x1"]-ft["x0"])/1000:.4f} {2*ft["half_w"]/1000:.4f} 0.008"/></geometry></collision>')
        L.append("</link>")
    for f in frames[1:]:
        j = J[f]; po = origin(j["parent"]); o = origin(f) - po
        lim = j["hi"] - j["lo"]
        L.append(f'<joint name="{f}" type="revolute"><parent link="{j["parent"]}_link"/><child link="{f}_link"/>'
                 f'<origin xyz="{o[0]/1000:.5f} {o[1]/1000:.5f} {o[2]/1000:.5f}"/><axis xyz="{j["out"][0]} {j["out"][1]} {j["out"][2]}"/>'
                 f'<limit lower="{np.radians(j["lo"]):.4f}" upper="{np.radians(j["hi"]):.4f}" effort="{STALL[j["servo"]]}" velocity="{VEL}"/>'
                 f'<dynamics damping="0.05" friction="0.02"/></joint>')
    L.append("</robot>")
    open(os.path.join(OUT, "humanoid.urdf"), "w").write("\n".join(L))
    print("urdf written:", len(frames), "links")

if __name__ == "__main__":
    main()
