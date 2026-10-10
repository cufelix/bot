"""Export STEP/STL/GLB + print plan. Run from repo root:  .venv/bin/python cad/export_all.py"""
import os, sys, csv, json
sys.path.insert(0, os.path.dirname(__file__))
import cadquery as cq
import robot as Rb, params as P
import trimesh, numpy as np

OUT = os.path.join(os.path.dirname(__file__), "..", "out")
COL = {"servo": (0.12, 0.12, 0.14), "printed": (0.86, 0.86, 0.82), "elec": (0.1, 0.55, 0.25), "misc": (0.5, 0.5, 0.5)}
PRINT_G_PER_H = 12.0   # rough 0.4 mm nozzle PETG average

def main():
    parts, J = Rb.build()
    for d in ("stl", "step", "stl_print"): os.makedirs(os.path.join(OUT, d), exist_ok=True)
    asm = cq.Assembly(name="humanoid")
    for n, p in parts.items():
        c = COL.get(p.kind, COL["misc"])
        asm.add(cq.Workplane("XY").add(p.shape), name=n, color=cq.Color(*c))
    asm.save(os.path.join(OUT, "robot.step"))
    asm.save(os.path.join(OUT, "robot.glb"), "GLTF")
    plan = []
    for n, p in parts.items():
        if p.kind != "printed": continue
        cq.exporters.export(cq.Workplane("XY").add(p.shape), os.path.join(OUT, "step", n + ".step"))
        stl = os.path.join(OUT, "stl", n + ".stl")
        cq.exporters.export(cq.Workplane("XY").add(p.shape), stl, tolerance=0.05, angularTolerance=0.1)
        m = trimesh.load(stl)
        poses, probs = m.compute_stable_poses(n_samples=2, threshold=0.0)
        best = poses[int(np.argmax(probs))] if len(poses) else np.eye(4)
        m.apply_transform(best)
        m.apply_translation(-m.bounds[0])
        m.export(os.path.join(OUT, "stl_print", n + ".stl"))
        ext = m.extents
        mass = Rb.mass_of(p)
        plan.append(dict(part=n, material=p.material, qty=1, mass_g=round(mass, 1), size_mm=f"{ext[0]:.0f}x{ext[1]:.0f}x{ext[2]:.0f}",
                         fits_bed=bool(all(e <= b for e, b in zip(sorted(ext), sorted(P.BED)))), est_print_h=round(mass / PRINT_G_PER_H + 0.3, 1),
                         watertight=bool(m.is_watertight)))
    with open(os.path.join(OUT, "print_plan.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(plan[0])); w.writeheader(); w.writerows(plan)
    print(len(plan), "printed parts; total mass", round(sum(r["mass_g"] for r in plan)), "g; est print", round(sum(r["est_print_h"] for r in plan)), "h")
    bad = [r["part"] for r in plan if not r["fits_bed"]]
    print("not fitting", P.BED, ":", bad)
    print("non-watertight:", [r["part"] for r in plan if not r["watertight"]])

if __name__ == "__main__":
    main()
