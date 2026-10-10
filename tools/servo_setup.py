#!/usr/bin/env python3
"""Per-servo bring-up on a bench BEFORE assembly. Connect ONE servo to a Waveshare adapter.
   python tools/servo_setup.py --port /dev/ttyACM0 --set-id 7         assign ID (factory ID is 1)
   python tools/servo_setup.py --port /dev/ttyACM0 --id 7 --center    mark current position as tick 2048 (do this with the horn off)
   python tools/servo_setup.py --port /dev/ttyACM0 --id 7 --info"""
import argparse, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware"))
from humanoid.feetech import Bus, Servos, CENTER
ap = argparse.ArgumentParser(); ap.add_argument("--port", required=True); ap.add_argument("--id", type=int, default=1)
ap.add_argument("--set-id", type=int); ap.add_argument("--center", action="store_true"); ap.add_argument("--info", action="store_true")
a = ap.parse_args(); s = Servos(Bus(a.port))
if not s.ping(a.id): sys.exit(f"no servo answers at id {a.id} (1 Mbps). Check power (12 V) and cable.")
if a.set_id:
    s.set_id(a.id, a.set_id); print("ID ->", a.set_id, "ping:", s.ping(a.set_id))
elif a.center:
    s.write(a.id, "lock", 0); s.write(a.id, "torque_enable", 0); s.write(a.id, "torque_enable", 128)
    import time; time.sleep(0.1); print("present position now", s.read(a.id, "present_pos"), "(expect", CENTER, ")")
else:
    print({k: s.read(a.id, k) for k in ("present_pos", "present_volt", "present_temp", "present_load", "min_pos", "max_pos")})
