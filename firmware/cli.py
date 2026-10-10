#!/usr/bin/env python3
"""Robot command line.   python cli.py --dry-run stand      (no hardware, FakeBus)
                          python cli.py ping | monitor | stand | crouch | wave | walk | relax"""
import argparse, sys, time, json
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from humanoid.robot import Robot, SafetyTrip
from humanoid import poses
from humanoid.gait import StaticWalker
from humanoid.stabilizer import Stabilizer

def ping(r):
    ok = True
    for n, j in r.joints.items():
        alive = r.buses[j["bus"]].ping(j["id"]); ok &= alive
        print(f"{n:14s} id={j['id']:2d} bus {j['bus']}  {'ok' if alive else 'NO REPLY'}")
    return 0 if ok else 2

def monitor(r, seconds=10):
    t0 = time.time()
    while time.time() - t0 < seconds:
        h = r.check_health(); a = r.read_angles()
        worst = max(h.items(), key=lambda kv: kv[1]["temp"])
        print(f"vmin {min(v['volt'] for v in h.values()):4.1f} V  hottest {worst[0]} {worst[1]['temp']} C  " + " ".join(f"{k[:6]}={v:5.1f}" for k, v in list(a.items())[:6]))
        time.sleep(0.5)

def walk(r, steps, step_mm, imu=None):
    w = StaticWalker(r.joints, steps=steps, step=step_mm); stab = Stabilizer(); dt = 1.0 / w.hz
    for c, feet, u in w.states():
        t = time.time()
        if imu:
            roll, pitch, rr, pr = imu.read(); dx, dy = stab.correction(roll, pitch, rr, pr); c = [c[0] + dx, c[1] + dy]
        try: r.send(w.angles(c, feet, u), max_step_deg=4.0)
        except ValueError as e:
            print("IK failed, holding:", e); break
        r.check_health()
        time.sleep(max(0.0, dt - (time.time() - t)))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["ping", "monitor", "stand", "crouch", "wave", "walk", "relax"])
    ap.add_argument("--dry-run", action="store_true"); ap.add_argument("--steps", type=int, default=4); ap.add_argument("--step-mm", type=float, default=40.0)
    ap.add_argument("--imu", action="store_true", help="use BNO085 tilt feedback while walking")
    a = ap.parse_args()
    r = Robot(dry_run=a.dry_run); ping_only = False
    zero = lambda: {n: 0.0 for n in r.names}
    try:
        if a.cmd == "ping": ping_only = True; return ping(r)
        if a.cmd == "relax": r.torque_off(); return 0
        r.configure(); r.soft_start(seconds=0.0 if a.dry_run else 2.0)
        if a.cmd == "monitor": monitor(r)
        elif a.cmd == "stand": r.move_to(zero(), 2.0)
        elif a.cmd == "crouch": r.move_to({**poses.crouch(18), **poses.ARMS_READY}, 2.5)
        elif a.cmd == "wave":
            r.move_to({**zero(), **poses.wave(0)}, 2.0); t0 = time.time()
            while time.time() - t0 < 6: r.send(poses.wave(time.time() - t0), max_step_deg=3.0); time.sleep(0.02)
        elif a.cmd == "walk":
            imu = None
            if a.imu:
                from humanoid.sensors import IMU; imu = IMU()
            walk(r, a.steps, a.step_mm, imu)
        r.move_to(zero(), 2.5)          # controlled stand-up before torque is released
    except (KeyboardInterrupt, SafetyTrip) as e:
        print("STOP:", repr(e)); r.emergency_stop()
    finally:
        if not ping_only: r.torque_off()
        r.close()

if __name__ == "__main__":
    sys.exit(main() or 0)
