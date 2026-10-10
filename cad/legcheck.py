from skeleton import *
from clash import clashes
js, routes = joints_and_routes()
items = {}
for j in js: items["S:" + j.name] = servo_solid(j)
for j in js:
    ch = [c for c in js if c.parent == j.name]
    items["K:" + j.name] = link_solid(j, ch, routes[j.name])
allow = set()
for j in js:
    allow.add(frozenset(("S:" + j.name, "K:" + j.name)))
    allow.add(frozenset(("S:" + j.name, "K:" + j.parent)))
    allow.add(frozenset(("K:" + j.name, "K:" + j.parent)))  # adjacent links meet at the joint
for r in clashes(items, allow): print(r)
print(len(items), "ok")
print("adjacent link overlaps:")
for j in js:
    if "K:"+j.parent in items:
        print(j.name, round(items["K:"+j.name].intersect(items["K:"+j.parent]).Volume(),1), "| servo vs parent link:", round(items["S:"+j.name].intersect(items["K:"+j.parent]).Volume(),1))
