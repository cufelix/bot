"""Joint table (IDs, bus, servo type, direction, limits) + angle<->tick conversion."""
import json, os
import numpy as np
from .feetech import TICKS, CENTER

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")

def load_joints(path=None, calib=None):
    j = json.load(open(path or os.path.join(CONFIG, "joints.json")))
    cp = calib or os.path.join(CONFIG, "calibration.json")
    cal = json.load(open(cp)) if os.path.exists(cp) else {}
    for n, v in j.items():
        c = cal.get(n, {})
        v["dir"] = c.get("dir", 1)              # +1 / -1 : servo tick direction vs. out-axis angle
        v["offset"] = c.get("offset", 0)         # ticks: physical zero = CENTER + offset
    return j

def angle_to_tick(joint, deg):
    """body-frame angle (deg, about canonical +X/+Y/+Z) -> servo tick"""
    out_deg = joint["sign"] * deg
    t = int(round(CENTER + joint["offset"] + joint["dir"] * out_deg * TICKS / 360.0))
    return max(0, min(TICKS - 1, t))

def tick_to_angle(joint, tick):
    out_deg = (tick - CENTER - joint["offset"]) * 360.0 / TICKS * joint["dir"]
    return joint["sign"] * out_deg

def body_limits(joint):
    """(lo, hi) in body-frame degrees from out-axis limits."""
    lo, hi = joint["lo"], joint["hi"]
    a, b = joint["sign"] * lo, joint["sign"] * hi
    return (min(a, b), max(a, b))

def clamp(joint, deg, margin=1.0):
    lo, hi = body_limits(joint)
    return float(np.clip(deg, lo + margin, hi - margin))
