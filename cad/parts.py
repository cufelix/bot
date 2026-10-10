"""Custom printed parts (pelvis, torso, head, neck fork, foot, hand) + electronics dummies."""
import cadquery as cq
from cadquery import Vector as V
import params as P
from kin import cradle, pocket_of, place, bar
import skeleton as S

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def fuse_all(shapes):
    s = shapes[0]
    for t in shapes[1:]: s = s.union(t)
    return s

def js_by_name():
    js, r = S.joints_and_routes()
    return {j.name: j for j in js}, r

def pelvis(J):
    ys = [J["L_hip_yaw"], J["R_hip_yaw"]]
    deck = box(-55, 55, -76, 76, S.PELVIS_TOP - S.DECK_T, S.PELVIS_TOP)
    sol = deck
    for j in ys:
        sol = sol.union(cq.Workplane("XY").add(cradle(j)))
    for j in ys:
        sol = sol.cut(cq.Workplane("XY").add(pocket_of(j)))
    # 4x M3 heat-set pockets for torso (corners), 2x cable slots
    pts = [(-44, -56), (-44, 56), (44, -56), (44, 56)]
    sol = sol.cut(cq.Workplane("XY").workplane(offset=S.PELVIS_TOP - 3.5).pushPoints(pts).circle(2.1).extrude(4))
    sol = sol.cut(box(-12, 12, -10, 10, S.PELVIS_TOP - 4, S.PELVIS_TOP + 1))
    for x in (-11.0, 31.0):                 # 20 mm Velcro strap slots for the battery
        for y in (-30.0, 30.0):
            sol = sol.cut(box(x - 1.5, x + 1.5, y - 11, y + 11, S.PELVIS_TOP - 4, S.PELVIS_TOP + 1))
    return sol

def torso_shell(J):
    ix, iy, w = S.TORSO_HX, S.TORSO_HY, S.WALL
    z0, z1 = S.TORSO_Z0, S.Z_NECK
    outer = box(-ix - w, ix + w, -iy - w, iy + w, z0, z1)
    inner = box(-ix, ix + w + 0.0, -iy, iy, z0 + 0, z1 - w)        # open at -X? open at back (x<0) handled below
    shell = outer.cut(box(-ix, ix, -iy, iy, z0, z1 - w))
    shell = shell.cut(box(-ix - w - 1, -ix + 0.01, -iy + 6, iy - 6, z0 + 6, z1 - 6))   # back opening (cover plate)
    # shoulder pitch cradles on both side walls; wall hole for horn boss
    for nm in ("L_sh_pitch", "R_sh_pitch"):
        j = J[nm]
        shell = shell.union(cq.Workplane("XY").add(cradle(j)))
        shell = shell.cut(cq.Workplane("XY").add(pocket_of(j)))
        hole = cq.Workplane("XY").circle(11.5).extrude(2 * w).translate((0, 0, -w)).val()
        hole = place(cq.Workplane("XY").circle(11.5).extrude(2 * w), j.pos, j.body, j.out)
        shell = shell.cut(cq.Workplane("XY").add(hole))
    # neck yaw cradle through top plate
    n = J["neck_yaw"]
    shell = shell.union(cq.Workplane("XY").add(cradle(n)))
    shell = shell.cut(cq.Workplane("XY").add(pocket_of(n)))
    for y, z in COVER_PTS():   # cover screw bosses (M3 self-tap, 2.5 pilot)
        shell = shell.union(cq.Workplane("YZ").workplane(offset=-ix - w).center(y, z).circle(4).extrude(9))
        shell = shell.cut(cq.Workplane("YZ").workplane(offset=-ix - w - 1).center(y, z).circle(1.25).extrude(10))
    zi = z0 + w
    for x in (-5.0, 40.0):                  # ledges that carry the Pi tray
        for sg in (-1, 1):
            shell = shell.union(box(x - 5, x + 5, min(sg * 56.0, sg * 62.0), max(sg * 56.0, sg * 62.0), zi + 19, zi + 25))
    # screw bosses to pelvis
    pts = [(-44, -56), (-44, 56), (44, -56), (44, 56)]
    for x, y in pts:
        shell = shell.union(cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(4).extrude(6))
        shell = shell.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(x, y).circle(1.7).extrude(10))
    return shell

COVER_PTS = lambda: [(y, z) for y in (-59.0, 59.0) for z in (S.TORSO_Z0 + 6.0, S.Z_NECK - 6.0)]

def pi_tray():
    zi = S.TORSO_Z0 + S.WALL
    t = box(-13, 50, -60, 60, zi + 25.5, zi + 28.5)
    for dx in (-24.5, 24.5):
        for dy in (-29.0, 29.0):
            t = t.cut(cq.Workplane("XY").workplane(offset=zi + 25).center(17 + dx, dy).circle(1.4).extrude(5))   # Pi 5 58x49 hole pattern, M2.5 self-tap
    t = t.cut(box(32, 44, -14, 14, zi + 25, zi + 29))                                                         # cable passage
    t = t.cut(box(-9, 3, -20, 20, zi + 25, zi + 29))
    return t

def torso_cover():
    ix, iy, w = S.TORSO_HX, S.TORSO_HY, S.WALL
    c = box(-ix - w - 3, -ix - w, -iy - w + 2, iy + w - 2, S.TORSO_Z0 + 1, S.Z_NECK - 1)
    for k in range(5):  # vent slots
        c = c.cut(box(-ix - w - 4, -ix - w + 1, -40, 40, S.TORSO_Z0 + 22 + k * 14, S.TORSO_Z0 + 28 + k * 14))
    c = c.cut(box(-ix - w - 4, -ix - w + 1, -7.4, 7.4, S.Z_NECK - 50.2, S.Z_NECK - 29.8))  # kill-switch hole
    for y, z in COVER_PTS():
        c = c.cut(cq.Workplane("YZ").workplane(offset=-ix - w - 4).center(y, z).circle(1.7).extrude(6))
    return c

def neck_fork(J):
    ny, npch = J["neck_yaw"], J["neck_pitch"]
    zc = npch.pos[2]
    parts = []
    from kin import flange
    parts.append(cq.Workplane("XY").add(flange(ny)))
    z_f = S.Z_NECK + P.HORN_H + P.FLANGE_T
    z_cross = zc - 30
    parts.append(box(-10, 10, -10, 10, z_f - 0.5, z_cross + 6))                     # column
    parts.append(box(-10, 10, -21, 19, z_cross, z_cross + 6))                        # crossbar
    yb = npch.pos[1] + P.HORN_H                                                     # horn flange plate
    parts.append(box(-14, 14, yb, yb + P.FLANGE_T, z_cross, zc + 14))
    yl = npch.pos[1] - P.SERVOS["STS3215"]["T"] - 4.0                                # rear support plate
    parts.append(box(-14, 14, yl - 4.0, yl, z_cross, zc + 14))
    sol = fuse_all(parts)
    # horn screw holes (placeholder pattern) + 623 bearing pocket on rear plate
    horn_holes = place(cq.Workplane("XY").workplane(offset=P.HORN_H).polarArray(P.HORN_BOLT_R, 45, 360, 4).circle(1.6).extrude(P.FLANGE_T + 1), npch.pos, npch.body, npch.out)
    sol = sol.cut(cq.Workplane("XY").add(horn_holes))
    bore = cq.Workplane("XZ").workplane(offset=-(yl)).center(0, zc).circle(5.1).extrude(-4.0)   # OD10 bearing seat
    sol = sol.cut(box(-5, 5, yl - 4.0, yl, zc - 5, zc + 5))
    return sol

def head_shell(J):
    npch = J["neck_pitch"]
    zc = npch.pos[2]
    x0, x1, y0, y1, z0, z1 = -36, 36, -40, 40, zc - 30, zc + 45
    w = 3
    shell = box(x0, x1, y0, y1, z0, z1).cut(box(x0 + w, x1 - w, y0 + w, y1 - w, z0 + w, z1 - w))
    shell = shell.cut(box(x0 - 1, x0 + w + 1, y0 + 8, y1 - 8, z0 + 8, z1 - 8))        # rear access (cover)
    shell = shell.cut(box(-30, 30, -25, 25, z0 - 1, z0 + w + 1))                      # neck slot
    cr = cq.Workplane("XY").add(cradle(npch))
    shell = shell.union(cr)
    shell = shell.cut(cq.Workplane("XY").add(pocket_of(npch)))
    # ribs from cradle to side walls (x) at servo mid height
    zr = zc + 2
    shell = shell.union(box(x0 + w, -20.8, -12, 12, zr - 2, zr + 2)).union(box(20.8, x1 - w, -12, 12, zr - 2, zr + 2))
    shell = shell.union(box(-20.8, 20.8, -15.7, 12, zc + 36, z1 - w + 0.5))       # roof bridge
    # camera window (front +X), two eye LED holes
    shell = shell.cut(cq.Workplane("YZ").workplane(offset=x1 - w - 1).center(0, zc + 22).circle(10).extrude(w + 2))
    for yy in (-24, 24):
        shell = shell.cut(cq.Workplane("YZ").workplane(offset=x1 - w - 1).center(yy, zc + 22).circle(4).extrude(w + 2))
    shell = shell.cut(box(x1 - w - 1, x1 + 1, -9, 9, zc - 4, zc + 2))               # mic slot
    for dy in (-14, 14):                                                             # 28 mm camera-board posts (M2 self-tap)
        for dz in (-14, 14):
            shell = shell.union(cq.Workplane("YZ").workplane(offset=30.2).center(dy, zc + 22 + dz).circle(3).extrude(3.0))
            shell = shell.cut(cq.Workplane("YZ").workplane(offset=29.5).center(dy, zc + 22 + dz).circle(0.85).extrude(5))
    return shell

def head_cover(J):
    zc = J["neck_pitch"].pos[2]
    x0, y0, y1, z0, z1 = -36, -40, 40, zc - 30, zc + 45
    return box(x0 - 0.0, x0 + 3, y0 + 8.5, y1 - 8.5, z0 + 8.5, z1 - 8.5)

FOOT_Y_HALF = P.FOOT_W / 2
def foot_extras(J, side):
    y = J[side + "ankle_roll"].pos[1]
    x0, x1 = P.FOOT_X0, P.FOOT_X0 + P.FOOT_L
    plate = box(x0, x1, y - FOOT_Y_HALF, y + FOOT_Y_HALF, 2, 8)
    plate = plate.edges("|Z").fillet(8)
    # 4 FSR pockets underneath (dia 14, 1.5 deep) at heel & forefoot corners + tpu recess 2mm
    pts = [(x0 + 22, y - 18), (x0 + 22, y + 18), (x1 - 22, y - 18), (x1 - 22, y + 18)]
    plate = plate.cut(cq.Workplane("XY").workplane(offset=2).pushPoints(pts).circle(7.2).extrude(1.5))
    # upstand rib joining plate to ankle bar
    return [plate.val()]

def hand_extras(J, side):
    e = J[side + "elbow"]
    z = S.Z_ELB - P.FOREARM
    sg = 1 if e.pos[1] > 0 else -1
    y = e.pos[1] + sg * (P.HORN_H + P.FLANGE_T / 2)
    h = box(S.AX + 5 - 17, S.AX + 5 + 17, y - 8, y + 8, z - 60, z + 5)
    h = h.edges("|Y").fillet(6)
    h = h.cut(box(S.AX + 5 + 4, S.AX + 5 + 18, y - 9, y + 9, z - 62, z - 28))   # finger slot
    return [h.val()]

def foot_sole(J, side):
    y = J[side + "ankle_roll"].pos[1]
    x0, x1 = P.FOOT_X0, P.FOOT_X0 + P.FOOT_L
    return box(x0, x1, y - FOOT_Y_HALF, y + FOOT_Y_HALF, 0, 2).edges("|Z").fillet(8)

# ---- electronics dummies (name, size xyz, centre xyz, mass g, price, note) ----
def electronics():
    zi = S.TORSO_Z0 + S.WALL          # floor top
    return {
        "battery_3S":       dict(size=(34, 105, 24), c=(10, 0, zi + 12), mass=190),
        "bus_adapter_legs": dict(size=(28, 41, 12), c=(-28, -30, zi + 6), mass=15),
        "bus_adapter_arms": dict(size=(28, 41, 12), c=(-28, 30, zi + 6), mass=15),
        "imu_bno085":       dict(size=(26, 18, 4), c=(-20, 0, zi + 2), mass=3),
        "ads1115_fsr":      dict(size=(18, 27, 5), c=(38, 0, zi + 3), mass=3),
        "pi5":              dict(size=(56, 85, 20), c=(17, 0, zi + 34 + 10), mass=60),
        "power_dist":       dict(size=(35, 40, 15), c=(-31, 0, zi + 21), mass=30),
        "buck_5V_5A":       dict(size=(34, 34, 11), c=(-31, 40, zi + 20), mass=20),
        "kill_switch":      dict(size=(20, 14, 20), c=(-S.TORSO_HX - 6, 0, S.Z_NECK - 40), mass=10),
    }
