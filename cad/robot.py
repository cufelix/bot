"""Builds the full part registry, FK posing, clash checks and exports."""
import cadquery as cq
from cadquery import Vector as V
import params as P, skeleton as S, parts as Pt
from kin import servo_solid, link_solid, cradle_center
from collections import OrderedDict

class Part:
    def __init__(s, name, shape, kind, frame, material="", mass=0.0, price=0.0, note=""):
        s.name, s.shape, s.kind, s.frame, s.material, s.mass, s.price, s.note = name, shape, kind, frame, material, mass, price, note

def _shape(x):
    return x.val() if hasattr(x, "val") else x

def build():
    J, R = Pt.js_by_name()
    js = list(J.values())
    parts = OrderedDict()
    def add(p): parts[p.name] = p
    # servos
    for j in js:
        rev = j.name == "neck_pitch"
        frame = "neck_pitch" if rev else ("base" if j.parent in ("pelvis", "torso") else j.parent)
        add(Part("servo_" + j.name, servo_solid(j), "servo", frame, j.servo, P.SERVOS[j.servo]["mass"], P.SERVOS[j.servo]["price"]))
    # generic links (flange + cradles + bars); custom ones are replaced below
    custom = {"neck_yaw", "neck_pitch"}
    for j in js:
        if j.name in custom: continue
        ch = [c for c in js if c.parent == j.name]
        extras = []
        if j.name.endswith("ankle_roll"): extras = Pt.foot_extras(J, j.name[:2])
        if j.name.endswith("elbow"): extras = Pt.hand_extras(J, j.name[:2])
        sol = link_solid(j, ch, R[j.name], extras=extras)
        add(Part("link_" + j.name, sol.val() if hasattr(sol, "val") else sol, "printed", j.name, "PETG-CF"))
    for side in ("L_", "R_"):
        add(Part("sole_" + side + "foot", Pt.foot_sole(J, side).val(), "printed", side + "ankle_roll", "TPU 95A"))
    add(Part("pelvis", Pt.pelvis(J).val(), "printed", "base", "PETG-CF"))
    add(Part("torso_shell", Pt.torso_shell(J).val(), "printed", "base", "PETG-CF"))
    add(Part("torso_cover", Pt.torso_cover().val(), "printed", "base", "PETG"))
    add(Part("neck_fork", Pt.neck_fork(J).val(), "printed", "neck_yaw", "PETG-CF"))
    add(Part("head_shell", Pt.head_shell(J).val(), "printed", "neck_pitch", "PETG"))
    add(Part("head_cover", Pt.head_cover(J).val(), "printed", "neck_pitch", "PETG"))
    for n, e in Pt.electronics().items():
        sx, sy, sz = e["size"]; cx, cy, cz = e["c"]
        add(Part(n, Pt.box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2).val(), "elec", "base", "", e["mass"]))
    zc = J["neck_pitch"].pos[2]
    add(Part("camera_usb", Pt.box(22, 32.5, -16, 16, zc + 6, zc + 38).val(), "elec", "neck_pitch", "", 25))
    return parts, J

def chain(frame, J):
    out = []
    while frame != "base":
        out.append(frame)
        p = J[frame].parent
        frame = "base" if p in ("pelvis", "torso") else p
    return out  # innermost first

def posed(part, J, angles):
    s = part.shape
    for fr in chain(part.frame, J):
        a = angles.get(fr, 0.0)
        if a:
            j = J[fr]
            s = s.rotate(V(*j.pos), V(*j.pos) + V(*j.out), a)
    return s

def mass_of(p):
    if p.kind == "printed": return p.shape.Volume() * P.INFILL_DENSITY * (1.0 if "TPU" not in p.material else 0.95 / 1.24)
    return p.mass
