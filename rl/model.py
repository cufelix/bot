"""Build the MuJoCo training model from the CAD-derived URDF: position servos with stall limits, foot boxes, floor."""
import json, os
import numpy as np, mujoco

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
import sys; sys.path.insert(0, os.path.join(ROOT, "firmware"))
from humanoid.poses import ARMS_SIM
MODEL_JSON = os.path.join(ROOT, "out", "model.json")
URDF = os.path.join(ROOT, "sim", "robot", "humanoid.urdf")
VBAT_SCALE = 11.1 / 12.0                       # servo datasheet numbers are for 12 V
STALL = {"STS3215": 2.9 * VBAT_SCALE, "STS3250": 4.9 * VBAT_SCALE}
NOLOAD_RAD_S = {"STS3215": 4.5, "STS3250": 3.9}  # rad/s, ~0.222 s / 60 deg (3215)
LEG_JOINTS = [f"{s}_{j}" for s in "LR" for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")]
LEG_JOINTS = [j.replace("L_", "L_").replace("R_", "R_") for j in LEG_JOINTS]
ARM_JOINTS = [f"{s}_{j}" for s in "LR" for j in ("sh_pitch", "sh_roll", "elbow")] + ["neck_yaw", "neck_pitch"]

def leg_names():
    return [f"{s}_{j}" for s in ("L", "R") for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")]

def default_pose(q=15.0):
    """Slight crouch (deg -> rad) that the policy acts around; arms folded forward, head level."""
    a = {}
    for s, sg in (("L_", 1), ("R_", -1)):
        a[s + "hip_pitch"] = -q * sg; a[s + "knee"] = 2 * q * sg; a[s + "ankle_pitch"] = -q * sg
    out = {k: np.radians(v) for k, v in a.items()}
    out.update(ARMS_SIM)
    return out

def build(kp=40.0, kv=1.2, dt=0.002, scene=None):
    info = json.load(open(MODEL_JSON))["joints"]
    spec = mujoco.MjSpec.from_file(URDF)
    spec.option.timestep = dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.visual.headlight.ambient = [0.55, 0.55, 0.55]; spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]; spec.visual.headlight.specular = [0.1, 0.1, 0.1]
    spec.visual.rgba.haze = [0.85, 0.9, 0.95, 1]
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.82, 0.84, 0.88], rgb2=[0.70, 0.73, 0.78], width=256, height=256)
    mat = spec.add_material(name="floor_mat"); mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"; mat.texrepeat = [10, 10]
    g = spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[20, 20, 0.1], rgba=[1, 1, 1, 1], material="floor_mat")
    g.friction = [1.0, 0.005, 0.0001]
    spec.body("base_link").add_freejoint()
    if scene:    # optional: head camera + goal ball (not used in training)
        cam = spec.body("neck_pitch_link").add_camera(name="head_cam", pos=[0.0302, -0.012, 0.022], fovy=55)
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES; cam.alt.xyaxes = [0, -1, 0, 0.42, 0, 0.91]   # looks forward, pitched down 25 deg to see the floor ahead
        for i, (x, y) in enumerate(scene.get("balls", [])):
            b = spec.worldbody.add_geom(name=f"ball{i}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.05, 0, 0], pos=[x, y, 0.05], rgba=[1.0, 0.45, 0.0, 1])
            b.contype = 0; b.conaffinity = 0
        for i, (x, y, r) in enumerate(scene.get("pillars", [])):
            p = spec.worldbody.add_geom(name=f"pillar{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r, 0.25, 0], pos=[x, y, 0.25], rgba=[0.2, 0.3, 0.8, 1])
    for j in spec.joints:
        if j.type != mujoco.mjtJoint.mjJNT_HINGE: continue
        st = STALL[info[j.name]["servo"]]
        j.armature = 0.012; j.frictionloss = 0.03; j.damping = [0.05, 0, 0]
        a = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.gainprm[0] = kp; a.biastype = mujoco.mjtBias.mjBIAS_AFFINE; a.biasprm[:3] = [0, -kp, -kv]
        a.forcelimited = 1; a.forcerange = [-st, st]
        a.ctrllimited = 1; a.ctrlrange = [np.radians(info[j.name]["lo"]) - 0.05, np.radians(info[j.name]["hi"]) + 0.05]
    m = spec.compile()
    # name the two foot boxes (the only colliding geoms apart from the floor)
    for gid in range(m.ngeom):
        if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[gid]:
            body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[gid])
            m.geom_condim[gid] = 3
    return m

def foot_geoms(m):
    out = {}
    for gid in range(m.ngeom):
        if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[gid]:
            body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[gid])
            out["L" if body.startswith("L_") else "R"] = gid
    return out
