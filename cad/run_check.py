import time
import robot as Rb
from clash import clashes
t = time.time()
parts, J = Rb.build()
print("built", len(parts), round(time.time() - t, 1), "s")
tot = 0
for n, p in parts.items():
    m = Rb.mass_of(p); tot += m
print("total mass g", round(tot))
for n, p in parts.items():
    if p.kind == "printed":
        bb = p.shape.BoundingBox(); print(f"{n:28s} {m:7.1f}" if False else f"{n:28s} {Rb.mass_of(p):7.1f} g  {bb.xlen:6.1f} {bb.ylen:6.1f} {bb.zlen:6.1f}")

print("--- neutral pose clashes ---")
items = {n: Rb.posed(p, J, {}) for n, p in parts.items()}
for r in clashes(items): print(r)
