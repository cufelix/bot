"""MuJoCo check: PD-held pose with a free base. Reports whether the robot stays up and peak joint torques.
Run: python sim/stand_test.py [--seconds 5] [--crouch Q]"""
import os, sys, json, argparse
import numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = json.load(open(os.path.join(HERE, "..", "out", "model.json")))
STALL = {"STS3215": 2.9, "STS3250": 4.9}

def load(kp=22.0, kv=0.9):
    spec = mujoco.MjSpec.from_file(os.path.join(HERE, "robot", "humanoid.urdf"))
    spec.option.timestep = 0.001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    g = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], rgba=[0.8, 0.8, 0.8, 1])
    g.friction = [1.0, 0.005, 0.0001]
    base = spec.body("base_link")
    base.add_freejoint()
    for j in spec.joints:
        if j.type == mujoco.mjtJoint.mjJNT_HINGE:
            a = spec.add_actuator(name="act_" + j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.gainprm[0] = kp; a.biastype = mujoco.mjtBias.mjBIAS_AFFINE; a.biasprm[:3] = [0, -kp, -kv]
            st = STALL[MODEL["joints"][j.name]["servo"]]
            a.forcelimited = 1; a.forcerange = [-st, st]
    m = spec.compile()
    return m

def crouch_pose(q):
    a = {}
    for s, sg in (("L_", 1), ("R_", -1)):
        a[s + "hip_pitch"] = -q * sg; a[s + "knee"] = 2 * q * sg; a[s + "ankle_pitch"] = -q * sg
    return a

def run(seconds=5.0, q=0.0, kp=20.0, kd=0.8, quiet=False):
    m = load(kp, kd); d = mujoco.MjData(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    hinge = [(i, n) for i, n in enumerate(names) if n]
    target = {n: np.radians(v) for n, v in crouch_pose(q).items()}
    qadr = {n: m.jnt_qposadr[i] for i, n in hinge}; vadr = {n: m.jnt_dofadr[i] for i, n in hinge}
    lim = {n: STALL[MODEL["joints"][n]["servo"]] for _, n in hinge}
    # free base initial height: lift until feet clear (sole z = 0 nominal)
    d.qpos[2] = 0.004 + (0.0 if q == 0 else 0.0)
    if q: 
        for n, v in target.items(): d.qpos[qadr[n]] = v
        d.qpos[2] = 0.004 - 0.0
        mujoco.mj_forward(m, d)
        low = min(d.geom_xpos[i, 2] for i in range(m.ngeom) if m.geom_contype[i] and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX)
        d.qpos[2] += 0.004 - low
    peak = {n: 0.0 for _, n in hinge}
    steps = int(seconds / m.opt.timestep)
    z0 = None; tilt_max = 0.0
    for k in range(steps):
        for ai, (_, n) in enumerate(hinge): d.ctrl[ai] = target.get(n, 0.0)
        mujoco.mj_step(m, d)
        for ai, (_, n) in enumerate(hinge): peak[n] = max(peak[n], abs(d.actuator_force[ai]))
        if k == steps // 2: z0 = d.qpos[2]
        # tilt of base from vertical
        zz = np.zeros(9); mujoco.mju_quat2Mat(zz, d.qpos[3:7]); tilt_max = max(tilt_max, np.degrees(np.arccos(np.clip(zz[8], -1, 1))))
    mujoco.mj_forward(m, d); comz = d.subtree_com[0][2]
    ok = comz > 0.25 and tilt_max < 15
    if not quiet:
        print(f"q={q:3.0f}  final CoM z={comz*1000:6.1f} mm  max tilt={tilt_max:5.1f} deg  -> {'STANDS' if ok else 'FELL'}")
        print("peak torque / stall: " + ", ".join(f"{n}:{p/lim[n]:.2f}" for n, p in sorted(peak.items(), key=lambda x: -x[1]/lim[x[0]])[:5]))
    return ok, peak, comz

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--seconds", type=float, default=4.0); ap.add_argument("--crouch", type=float, default=0.0)
    a = ap.parse_args()
    run(a.seconds, a.crouch)
