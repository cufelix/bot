"""Joint table + rigid link generation. One Joint = one servo; its horn drives the child link."""
from dataclasses import dataclass
import cadquery as cq
from cadquery import Vector as V
import params as P

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
neg = lambda v: tuple(-c for c in v)

@dataclass(frozen=True)
class Joint:
    name: str
    servo: str
    pos: tuple        # shaft position
    out: tuple        # horn direction == joint axis
    body: tuple       # servo long-axis direction (from shaft toward far end)
    parent: str       # parent link name
    lo: float = -90.0
    hi: float = 90.0

def mirror_joint(j, side, prefix_parent=True):
    m = lambda v: (v[0], -v[1], v[2])
    par = side + j.parent if j.parent != "pelvis" and j.parent != "torso" else j.parent
    return Joint(side + j.name, j.servo, m(j.pos), m(j.out), m(j.body), par, j.lo, j.hi)

def servo_solid(j):
    s = P.SERVOS[j.servo]
    body = (cq.Workplane("XY").box(s["L"], s["W"], s["T"], centered=(False, True, False))
            .translate((-s["shaft_back"], 0, -s["T"])))
    horn = cq.Workplane("XY").circle(P.HORN_D / 2).extrude(P.HORN_H)
    boss = cq.Workplane("XY").circle(6).extrude(P.HORN_H)
    solid = body.union(horn).union(boss)
    return place(solid, j.pos, j.body, j.out)

def place(wp, pos, xdir, normal):
    return wp.val().moved(cq.Location(cq.Plane(origin=pos, xDir=xdir, normal=normal)))

def cradle(j):
    """printed pocket that holds servo j; open on horn face, wall on the other 5 faces."""
    s = P.SERVOS[j.servo]; w = P.CRADLE_WALL; c = P.CLEAR
    L, W, T = s["L"] + 2 * w, s["W"] + 2 * w, s["T"] + w
    outer = cq.Workplane("XY").box(L, W, T, centered=(False, True, False)).translate((-s["shaft_back"] - w, 0, -s["T"] - w))
    pocket = cq.Workplane("XY").box(s["L"] + 2 * c, s["W"] + 2 * c, s["T"] + 5, centered=(False, True, False)).translate((-s["shaft_back"] - c, 0, -s["T"] - c))
    shell = outer.cut(pocket)
    # 4x M3 screws through back wall (placeholder pattern)
    pts = [(-s["shaft_back"] + 8, 10), (-s["shaft_back"] + 8, -10), (-s["shaft_back"] + s["L"] - 8, 10), (-s["shaft_back"] + s["L"] - 8, -10)]
    holes = cq.Workplane("XY").workplane(offset=-s["T"] - w).pushPoints(pts).circle(1.7).extrude(w + 1)
    return place(shell.cut(holes), j.pos, j.body, j.out)

def flange(j):
    """disc bolted to horn (4x M3 + centre bore)."""
    d = cq.Workplane("XY").workplane(offset=P.HORN_H).circle(14).extrude(P.FLANGE_T)
    d = d.faces(">Z").workplane().circle(3.1).cutThruAll()
    d = d.faces(">Z").workplane().polarArray(P.HORN_BOLT_R, 45, 360, 4).circle(1.6).cutThruAll()
    return place(d, j.pos, j.body, j.out)

def bar(p0, p1, ref, w=20.0, t=8.0):
    """rectangular bar p0->p1; thickness direction ~ ref."""
    a, b = V(*p0), V(*p1); d = b - a
    ln = d.Length
    if ln < 1e-6: return None
    d = d.normalized(); r = V(*ref)
    n = r - d * r.dot(d)
    if n.Length < 1e-6: n = V(0, 0, 1) - d * d.z
    if n.Length < 1e-6: n = V(1, 0, 0)
    e = t / 2
    pl = cq.Plane(origin=(a - d * e).toTuple(), xDir=n.normalized().toTuple(), normal=d.toTuple())
    return cq.Workplane(pl).rect(t, w).extrude(ln + 2 * e).val()

def flange_anchor(j):
    return tuple(V(*j.pos) + V(*j.out) * (P.HORN_H + P.FLANGE_T / 2))

def cradle_center(j):
    s = P.SERVOS[j.servo]
    c = V(*j.pos) + V(*j.body) * (s["L"] / 2 - s["shaft_back"]) - V(*j.out) * (s["T"] / 2)
    return c.toTuple()

def pocket_of(c):
    s = P.SERVOS[c.servo]; cl = P.CLEAR
    pk = (cq.Workplane("XY").box(s["L"] + 2 * cl, s["W"] + 2 * cl, s["T"] + 5, centered=(False, True, False))
          .translate((-s["shaft_back"] - cl, 0, -s["T"] - cl)))
    return place(pk, c.pos, c.body, c.out)

def link_solid(parent_joint, children, routes=None, extras=()):
    """link = flange on parent_joint horn + cradle per child + bars along waypoint routes (+ extras).
    routes: {child_name: (waypoints...)}; path starts at flange anchor, last waypoint must be inside child cradle wall."""
    routes = routes or {}
    parts = list(extras)
    if parent_joint is not None:
        parts.append(flange(parent_joint))
    for c in children:
        parts.append(cradle(c))
        if parent_joint is not None:
            pts = [flange_anchor(parent_joint)] + list(routes.get(c.name, [cradle_center(c)]))
            ref = parent_joint.out
            for a, b in zip(pts, pts[1:]):
                bb = bar(a, b, ref)
                if bb is not None: parts.append(bb)
    if parent_joint is not None and not children and "end" in routes:
        pts = [flange_anchor(parent_joint)] + list(routes["end"])
        for a, b in zip(pts, pts[1:]):
            parts.append(bar(a, b, parent_joint.out))
    sol = parts[0]
    for p in parts[1:]:
        sol = sol.fuse(p)
    for c in children:
        sol = sol.cut(pocket_of(c))
    return sol
