"""MuJoCo: run the quasi-static gait through the PD-servo model. Reports distance walked / whether it fell.
Run: python sim/walk_test.py [--steps 4] [--step 40] [--video out/walk.mp4]"""
import os, sys, argparse, json
import numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "firmware")); sys.path.insert(0, HERE)
from humanoid.joints import load_joints
from humanoid.gait import StaticWalker
from humanoid.stabilizer import Stabilizer
import stand_test as S

def run(steps=4, step=40.0, kp=60.0, kd=1.5, frames_out=None, quiet=False, stab=True, **gait_kw):
    J = load_joints(calib="/nonexistent")
    walker = StaticWalker(J, steps=steps, step=step, **gait_kw)
    seq = list(walker.states()); stabz = Stabilizer() if stab else None
    m = S.load(kp, kd); d = mujoco.MjData(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    hinge = [(i, n) for i, n in enumerate(names) if n]
    qadr = {n: m.jnt_qposadr[i] for i, n in hinge}; vadr = {n: m.jnt_dofadr[i] for i, n in hinge}
    lim = {n: S.STALL[S.MODEL["joints"][n]["servo"]] for _, n in hinge}
    d.qpos[2] = 0.004; mujoco.mj_forward(m, d)
    dt = m.opt.timestep; per = int(0.02 / dt)
    peak = {n: 0.0 for _, n in hinge}; acc = {n: 0.0 for _, n in hinge}; nacc = 0; tilt_max = 0.0; fell = False; why = ''
    x_start = d.subtree_com[0][0]
    rend = None
    if frames_out:
        import imageio.v2 as imageio
        rend = mujoco.Renderer(m, 360, 480); vid = []
        cam = mujoco.MjvCamera(); cam.azimuth = 120; cam.elevation = -12; cam.distance = 1.1; cam.lookat[:] = [0.1, 0, 0.3]
    for fi, (c, feet, u) in enumerate(seq):
        if stabz:
            B = d.xmat[1].reshape(3, 3); roll = np.degrees(np.arctan2(B[2, 1], B[2, 2])); pitch = np.degrees(np.arcsin(-B[2, 0]))
            w = d.cvel[1][:3] if False else d.qvel[3:6]    # base angular velocity (body frame for free joint)
            dx, dy = stabz.correction(roll, pitch, np.degrees(w[0]), np.degrees(w[1]))
            c = np.array([c[0] + dx, c[1] + dy])
        try:
            f = walker.angles(c, feet, u)
        except ValueError as e:
            fell = True; why = f'IK: {e}'; break
        tgt = {n: np.radians(J[n]["sign"] * a) for n, a in f.items()}
        for _ in range(per):
            for ai, (_, n) in enumerate(hinge): d.ctrl[ai] = tgt.get(n, 0.0)
            mujoco.mj_step(m, d)
            for ai, (_, n) in enumerate(hinge): peak[n] = max(peak[n], abs(d.actuator_force[ai])); acc[n] += abs(d.actuator_force[ai])
            nacc += 1
        zz = np.zeros(9); mujoco.mju_quat2Mat(zz, d.qpos[3:7]); tilt = np.degrees(np.arccos(np.clip(zz[8], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        if d.subtree_com[0][2] < 0.22 or tilt > 35: fell = True; why = f'tilt {tilt:.0f} at frame {fi}'; break
        if rend and fi % 3 == 0:
            cam.lookat[0] = d.subtree_com[0][0]; rend.update_scene(d, cam); vid.append(rend.render())
    dist = (d.subtree_com[0][0] - x_start) * 1000
    if frames_out and vid:
        imageio.mimsave(frames_out, vid, fps=17)
    if not quiet:
        top = sorted(peak.items(), key=lambda x: -x[1] / lim[x[0]])[:4]
        print(f"steps={steps} step={step:.0f}mm  distance={dist:6.1f} mm  max tilt={tilt_max:4.1f} deg  -> {'FELL ' + why if fell else 'WALKED'}")
        print("peak torque/stall:", ", ".join(f"{n}:{p/lim[n]:.2f}" for n, p in top))
    run.mean_torque = {n: a / max(nacc, 1) for n, a in acc.items()}
    return (not fell), dist, peak

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--steps", type=int, default=4); ap.add_argument("--step", type=float, default=40.0)
    ap.add_argument("--video", default=None)
    a = ap.parse_args(); run(a.steps, a.step, frames_out=a.video)
