"""Find the collision-free range of every joint (others neutral), 5 deg steps."""
import json, sys
import robot as Rb
from clash import bb_overlap, vol
parts, J = Rb.build()
def subtree(j):
    out = set()
    for n, p in parts.items():
        if j in Rb.chain(p.frame, J): out.add(n)
    return out
def first_clash(j, sign, limit, step=5):
    moving = subtree(j)
    a = 0
    while abs(a) < abs(limit):
        a += sign * step
        if abs(a) > abs(limit): a = limit
        shapes = {n: Rb.posed(p, J, {j: a}) for n, p in parts.items()}
        for m in moving:
            for n in shapes:
                if n in moving: continue
                if bb_overlap(shapes[m], shapes[n]) and vol(shapes[m], shapes[n]) > 1.0:
                    return a - sign * step, (m, n)
        if a == limit: break
    return limit, None
res = {}
for j, jt in J.items():
    if j.startswith("R_"): continue   # symmetric
    hi, ch = first_clash(j, +1, jt.hi); lo, cl = first_clash(j, -1, jt.lo)
    res[j] = dict(design=(jt.lo, jt.hi), free=(lo, hi), hit_hi=ch, hit_lo=cl)
    print(f"{j:14s} design [{jt.lo:6.0f},{jt.hi:6.0f}] collision-free [{lo:6.0f},{hi:6.0f}]  {ch or ''} {cl or ''}", flush=True)
json.dump(res, open("../out/joint_sweep.json", "w"), indent=1, default=list)
