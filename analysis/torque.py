"""Static joint-torque envelope over a set of representative poses. Run: python analysis/torque.py"""
import json, os, sys
import numpy as np
from scipy.optimize import brentq
sys.path.insert(0, os.path.dirname(__file__))
from kinematics import Robot, G

DYN = 1.8            # dynamic multiplier on static torque (walking / disturbance margin)
STALL_FRAC = 0.65    # peak (static*DYN) must stay below this fraction of stall torque
SERVO = {"STS3215": dict(stall=2.9, rated=1.0), "STS3250": dict(stall=4.9, rated=1.6)}

def stance_pose(R, q_deg, support="single", side="L_"):
    """Squat with flat foot (hip=-q, knee=2q, ankle=-q) and lateral lean so CoM is over the stance foot."""
    sgn = 1.0 if side == "L_" else -1.0
    def ang(r):
        a = {}
        for s in ("L_", "R_"):
            a[s + "hip_pitch"] = -q_deg * (1 if s == "L_" else 1)
            a[s + "knee"] = 2 * q_deg
            a[s + "ankle_pitch"] = -q_deg
        # Mirrored right-leg joints use a mirrored axis: their out-angle is the negative of the body-frame angle for Y axes
        for s in ("R_",):
            a[s + "hip_pitch"] = q_deg; a[s + "knee"] = -2 * q_deg; a[s + "ankle_pitch"] = q_deg
        if support == "single":
            a[side + "hip_roll"] = r; a[side + "ankle_roll"] = -r
            # swing leg: lifted 3 cm worth of knee flex, roll out slightly
            o = "R_" if side == "L_" else "L_"
            a[o + "hip_pitch"] = (-q_deg - 25) * (1 if o == "L_" else -1)
            a[o + "knee"] = (2 * q_deg + 50) * (1 if o == "L_" else -1)
            a[o + "ankle_pitch"] = (-q_deg - 25) * (1 if o == "L_" else -1)
        return a
    if support == "double":
        return stance_pose.__wrapped__(R, q_deg) if False else (lambda: (ang(0.0)))(), None
    return ang, None

def com_y_err(R, side, q):
    ang, _ = stance_pose(R, q, "single", side)
    def f(r):
        a = ang(r)
        return R.com(a)[0][1] - R.foot_center(side, a)[1]
    return ang, f

def single_support(R, q, side="L_", cop_dx=0.0, cop_dy=0.0):
    ang, f = com_y_err(R, side, q)
    lo, hi = (-25, 25)
    try:
        r = brentq(f, lo, hi)
    except ValueError:
        r = lo if abs(f(lo)) < abs(f(hi)) else hi
    a = ang(r); a = {k: v for k, v in a.items()}
    com, M = R.com(a)
    corners = R.foot_corners(side, a)
    xmin, xmax = corners[:, 0].min(), corners[:, 0].max()
    margin = min(com[0] - xmin, xmax - com[0])
    ymin, ymax = corners[:, 1].min(), corners[:, 1].max()
    y_margin = min(com[1] - ymin, ymax - com[1])
    cop = (com[0] + cop_dx, com[1] + cop_dy); F = M * G
    taus = {j: abs(R.torque_with_ground(side + j, a, cop, F)) for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")}
    return dict(q=q, cop_shift=[cop_dx, cop_dy], roll_lean=round(r, 1), com=[round(c, 1) for c in com], x_margin_mm=round(margin, 1), y_margin_mm=round(y_margin, 1), roll_lean_saturated=abs(r) >= 25 - 1e-6, tau=taus)

def double_support(R, q):
    a = {}
    for s, sg in (("L_", 1), ("R_", -1)):
        a[s + "hip_pitch"] = -q * sg; a[s + "knee"] = 2 * q * sg; a[s + "ankle_pitch"] = -q * sg
    com, M = R.com(a)
    corners = R.foot_corners("L_", a)
    margin = min(com[0] - corners[:, 0].min(), corners[:, 0].max() - com[0])
    out = {}
    for j in ("hip_pitch", "knee", "ankle_pitch"):
        out[j] = abs(R.torque_with_ground("L_" + j, a, (com[0], R.J["L_ankle_roll"]["pos"][1]), M * G / 2))
    return dict(q=q, com=[round(c, 1) for c in com], x_margin_mm=round(margin, 1), tau=out)

def arm_cases(R, payload_kg=0.2):
    res = {}
    hand = np.array(R.J["L_elbow"]["pos"]) + np.array([0, 0, -95.0])
    cases = {"arm_forward_horizontal": {"L_sh_pitch": -90.0}, "arm_sideways_horizontal": {"L_sh_roll": 90.0},
             "arm_fwd_elbow90": {"L_sh_pitch": -90.0, "L_elbow": -90.0}}
    for name, a in cases.items():
        t = {}
        for j in ("sh_pitch", "sh_roll", "elbow"):
            base = abs(R.torque_on("L_" + j, a))
            org = R.joint_origin("L_" + j, a); ax = R._axis_world("L_" + j, a)
            hp = R.place("L_elbow", hand, a)
            base2 = abs((ax @ np.cross(hp - org, [0, 0, -G * payload_kg])) + R.torque_on("L_" + j, a) * (1 if True else 0)) if False else None
            pl = ax @ np.cross((hp - org) / 1000.0, [0, 0, -G * payload_kg])
            t[j] = abs(R.torque_on("L_" + j, a) + pl)
        res[name] = t
    return res

def neck_cases(R):
    res = {}
    for ang in (-15, 15):
        res[f"neck_pitch_{ang}"] = {"neck_pitch": abs(R.torque_on("neck_pitch", {"neck_pitch": ang}))}
    return res

def main(write=True):
    R = Robot()
    report = dict(mass_kg=round(R.mass, 3), cases={})
    for q in (0, 10, 20):
        report["cases"][f"single_support_q{q}"] = single_support(R, q)
    for q in (10, 20):
        for dx, dy in ((30, 0), (-30, 0), (0, 20), (0, -20)):
            report["cases"][f"single_support_q{q}_cop{dx}_{dy}"] = single_support(R, q, cop_dx=dx, cop_dy=dy)
    for q in (15, 25, 40, 55):
        report["cases"][f"double_support_q{q}"] = double_support(R, q)
    # swing leg hold
    sw = {"L_hip_pitch": -45.0, "L_knee": 90.0, "L_ankle_pitch": -45.0}
    report["cases"]["swing_leg_hip45_knee90"] = {"tau": {j: abs(R.torque_on("L_" + j, sw)) for j in ("hip_pitch", "knee", "ankle_pitch")}}
    for k, v in arm_cases(R).items(): report["cases"][k] = {"tau": v}
    for k, v in neck_cases(R).items(): report["cases"][k] = {"tau": v}
    # envelope per joint (max over cases), only feasible-balance cases for stance
    env = {}
    for name, c in report["cases"].items():
        if "x_margin_mm" in c and (c["x_margin_mm"] < 5 or c.get("y_margin_mm", 99) < 5 or c.get("roll_lean_saturated")): c["feasible"] = False; continue
        c["feasible"] = True
        for j, t in c["tau"].items():
            env[j] = max(env.get(j, 0), t)
    # joint -> servo mapping from model
    J = R.J
    rows = []
    for j, t in sorted(env.items()):
        servo = J["L_" + j]["servo"] if "L_" + j in J else J[j]["servo"]
        s = SERVO[servo]
        rows.append(dict(joint=j, servo=servo, static_nm=round(t, 2), peak_nm=round(t * DYN, 2), stall_nm=s["stall"], rated_nm=s["rated"],
                         peak_over_stall=round(t * DYN / s["stall"], 2), static_over_rated=round(t / s["rated"], 2),
                         ok=bool(t * DYN <= STALL_FRAC * s["stall"])))
    report["envelope"] = rows
    if write:
        json.dump(report, open(os.path.join(os.path.dirname(__file__), "..", "out", "torque_report.json"), "w"), indent=1)
    return report

if __name__ == "__main__":
    r = main()
    print("mass", r["mass_kg"], "kg")
    for k, c in r["cases"].items():
        print(f"{k:28s}", "feasible" if c["feasible"] else "UNBALANCED", c.get("x_margin_mm", ""), {a: round(b, 2) for a, b in c["tau"].items()})
    print()
    for row in r["envelope"]: print(row)
