"""Joint table for the whole robot (left defined here, right mirrored). All numbers in mm."""
from kin import *
import params as P

Zr = P.ANKLE_ROLL_Z
Zp = Zr + P.OFFSET
Zk = Zp + P.SHANK
Zhp = Zk + P.THIGH
Zhr = Zhp + P.OFFSET
Zhy = Zhr + P.OFFSET
HY = P.HIP_HALF
FA = P.HORN_H + P.FLANGE_T / 2   # flange mid-plane offset from shaft plane
XAR = -24.0  # ankle roll servo x
XR = -30.0   # hip roll servo sits behind the pitch line

# routes: {joint_name: {child_name_or_'end': waypoints}} (left side coords)
LEG_ROUTES = {
    "hip_yaw":   {"hip_roll": [(-42, HY, Zhy - FA), (-42, HY, Zhr + 12)]},
    "hip_roll":  {"hip_pitch": [(XR + FA, HY - 7, Zhr - 2), (-19, HY - 7, Zhr - 2)]},
    "hip_pitch": {"knee": [(0, HY + FA, Zk + 35.7), (0, HY - 15, Zk + 35.7)]},
    "knee":      {"ankle_pitch": [(0, HY - 1 + FA, Zp + 36), (0, HY - 15, Zp + 36)]},
    "ankle_pitch": {"ankle_roll": [(0, HY - 1 + FA, Zr + 24), (XAR - 10, HY - 1 + FA, Zr + 24), (XAR - 10, HY - 1 + FA, Zr + 19.3)]},
    "ankle_roll": {"end": [(XAR + FA, HY, 6)]},
}

def left_leg():
    return [
        Joint("hip_yaw",   "STS3215", (0, HY, Zhy), neg(Z), X, "pelvis", -30, 20),
        Joint("hip_roll",  "STS3250", (XR, HY, Zhr), X, neg(Z), "hip_yaw", -15, 25),
        Joint("hip_pitch", "STS3250", (0, HY, Zhp), Y, Z, "hip_roll", -90, 40),
        Joint("knee",      "STS3250", (0, HY - 1, Zk), Y, Z, "hip_pitch", 0, 130),
        Joint("ankle_pitch","STS3250", (0, HY - 1, Zp), Y, Z, "knee", -50, 50),
        Joint("ankle_roll","STS3215", (XAR, HY, Zr), X, neg(Y), "ankle_pitch", -20, 20),
    ]

def mirror_route(r, side, names):
    return {(side + k if k != "end" else k): [(p[0], -p[1], p[2]) if side == "R_" else p for p in v] for k, v in r.items()}

def joints_and_routes():
    js, routes = [], {}
    for side in ("L_", "R_"):
        for j in left_leg():
            par = j.parent if j.parent in ("pelvis", "torso") else side + j.parent
            if side == "L_":
                jj = Joint(side + j.name, j.servo, j.pos, j.out, j.body, par, j.lo, j.hi)
            else:
                m = lambda v: (v[0], -v[1], v[2])
                jj = Joint(side + j.name, j.servo, m(j.pos), m(j.out), m(j.body), par, j.lo, j.hi)
            js.append(jj)
            routes[jj.name] = mirror_route(LEG_ROUTES[j.name], side, None)
    return js, routes

# ---------------- upper body ----------------
DECK_T = 3.0
PELVIS_TOP = Zhy + P.SERVOS["STS3215"]["T"] + P.CRADLE_WALL      # top of hip-yaw cradles
TORSO_Z0 = PELVIS_TOP
TORSO_H = 120.0
Z_NECK = TORSO_Z0 + TORSO_H          # torso top surface == neck-yaw servo front plane
Z_SH = Z_NECK - 32.0                 # shoulder axis height
TORSO_HX, TORSO_HY = 50.0, 62.0      # inner half sizes (x, y)
WALL = 3.0
SHY = TORSO_HY                       # shoulder pitch shaft plane == inner side of wall
AY = 104.0                           # arm roll axis y
AX = -18.0                           # arm roll servo x
Z_ELB = Z_SH - P.UPPER_ARM
YE = 112.0
NK = Z_NECK + 40.0                   # neck pitch axis height

ARM_ROUTES = {
    "sh_pitch": {"sh_roll": [(0, SHY + FA, Z_SH + 19.3), (-30, SHY + FA, Z_SH + 19.3)]},
    "sh_roll":  {"elbow": [(AX + FA, AY, Z_ELB + 35.7)]},
    "elbow":    {"end": [(AX + FA, YE + FA, Z_ELB - P.FOREARM)]},
}

def left_arm():
    return [
        Joint("sh_pitch", "STS3215", (0, SHY, Z_SH), Y, X, "torso", -170, 90),
        Joint("sh_roll",  "STS3215", (AX, AY, Z_SH), X, neg(Y), "sh_pitch", -5, 100),
        Joint("elbow",    "STS3215", (AX + FA, YE, Z_ELB), Y, Z, "sh_roll", -130, 5),
    ]

NECK_ROUTES = {"neck_yaw": {}}
def neck():
    return [
        Joint("neck_yaw",   "STS3215", (0, 0, Z_NECK), Z, X, "torso", -80, 80),
        Joint("neck_pitch", "STS3215", (0, 12, NK), Y, Z, "neck_yaw", -15, 15),
    ]

def mir(v): return (v[0], -v[1], v[2])

def joints_and_routes():
    js, routes = [], {}
    for side in ("L_", "R_"):
        for table, rt in ((left_leg(), LEG_ROUTES), (left_arm(), ARM_ROUTES)):
            for j in table:
                par = j.parent if j.parent in ("pelvis", "torso") else side + j.parent
                if side == "L_":
                    jj = Joint(side + j.name, j.servo, j.pos, j.out, j.body, par, j.lo, j.hi)
                else:
                    jj = Joint(side + j.name, j.servo, mir(j.pos), mir(j.out), mir(j.body), par, -j.hi, -j.lo)
                js.append(jj)
                routes[jj.name] = mirror_route(rt[j.name], side, None)
    for j in neck():
        js.append(j)
        routes[j.name] = {}
    return js, routes
