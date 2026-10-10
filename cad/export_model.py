"""Dump kinematics + mass model (JSON) used by analysis, URDF and firmware generators."""
import json, os
import robot as Rb
import params as P

MISC_G = 300.0   # wiring, fasteners, bearings, tape, connectors (not modelled in CAD)

def main(out="../out/model.json"):
    parts, J = Rb.build()
    model = dict(
        joints={n: dict(servo=j.servo, pos=j.pos, out=j.out, body=j.body, parent=("base" if j.parent in ("pelvis", "torso") else j.parent), lo=j.lo, hi=j.hi) for n, j in J.items()},
        parts=[],
    )
    for n, p in parts.items():
        c = p.shape.Center()
        model["parts"].append(dict(name=n, kind=p.kind, frame=p.frame, material=p.material, mass=round(Rb.mass_of(p), 2),
                                   com=[round(c.x, 2), round(c.y, 2), round(c.z, 2)], volume=round(p.shape.Volume(), 1)))
    model["parts"].append(dict(name="misc_wiring_fasteners", kind="misc", frame="base", material="", mass=MISC_G, com=[0, 0, 440.0], volume=0))
    model["feet"] = dict(x0=P.FOOT_X0, x1=P.FOOT_X0 + P.FOOT_L, half_w=P.FOOT_W / 2)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(model, open(out, "w"), indent=1)
    print("model.json:", len(model["parts"]), "parts, total", round(sum(p["mass"] for p in model["parts"])), "g")

if __name__ == "__main__":
    main()
