"""Named joint poses (body-frame degrees). Missing joints = 0."""
STAND = {}
def crouch(q=15.0):
    a = {}
    for s in ("L_", "R_"):
        a[s + "hip_pitch"] = -q; a[s + "knee"] = 2 * q; a[s + "ankle_pitch"] = -q
    return a
ARMS_READY = {"L_sh_pitch": -20.0, "R_sh_pitch": -20.0, "L_elbow": -40.0, "R_elbow": -40.0, "L_sh_roll": 8.0, "R_sh_roll": -8.0}
def wave(t):
    import math
    return {"R_sh_pitch": -160.0, "R_sh_roll": -30.0, "R_elbow": -60.0 + 25 * math.sin(2 * math.pi * 1.5 * t)}
