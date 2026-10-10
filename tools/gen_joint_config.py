"""Generate firmware/config/joints.json from out/model.json (CAD is the single source of truth).
Bus A ids 1-12 legs, Bus B ids 13-20 arms + neck."""
import json, os
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
model = json.load(open(os.path.join(ROOT, "out", "model.json")))
order = (["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle_pitch", "L_ankle_roll",
          "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle_pitch", "R_ankle_roll"],
         ["L_sh_pitch", "L_sh_roll", "L_elbow", "R_sh_pitch", "R_sh_roll", "R_elbow", "neck_yaw", "neck_pitch"])
AXIS = {"hip_yaw": 2, "hip_roll": 0, "hip_pitch": 1, "knee": 1, "ankle_pitch": 1, "ankle_roll": 0,
        "sh_pitch": 1, "sh_roll": 0, "elbow": 1, "neck_yaw": 2, "neck_pitch": 1}
def canonical(name, out):
    role = name[2:] if name[:2] in ("L_", "R_") else name
    a = AXIS[role]
    return a, (1 if out[a] > 0 else -1)
cfg, sid = {}, 1
for bus, names in zip(("A", "B"), order):
    for n in names:
        j = model["joints"][n]; axis, sign = canonical(n, j["out"])
        cfg[n] = dict(id=sid, bus=bus, servo=j["servo"], axis="XYZ"[axis], sign=sign, lo=j["lo"], hi=j["hi"], pos_mm=j["pos"]); sid += 1
os.makedirs(os.path.join(ROOT, "firmware", "config"), exist_ok=True)
json.dump(cfg, open(os.path.join(ROOT, "firmware", "config", "joints.json"), "w"), indent=1)
print(len(cfg), "joints -> firmware/config/joints.json")
