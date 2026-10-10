"""Hardware abstraction: 20 joints on two Feetech buses, with limits, ramps and safety."""
import json, os, time
import numpy as np
from .feetech import Servos, Bus, FakeBus, CENTER, TICKS, decode_sign_magnitude
from .joints import load_joints, angle_to_tick, tick_to_angle, body_limits, clamp

TEMP_WARN, TEMP_CUT = 60, 70          # deg C
VOLT_WARN, VOLT_CUT = 10.2, 9.6       # V (3S: 3.4 V / 3.2 V per cell)
DEFAULT_PORTS = {"A": "/dev/serial/by-id/usb-legs", "B": "/dev/serial/by-id/usb-upper"}
TORQUE_LIMIT_BOOT = 400               # of 1000 until the soft-start ramp finishes

class SafetyTrip(RuntimeError):
    pass

class Robot:
    def __init__(self, ports=None, dry_run=False, joints=None):
        self.joints = joints or load_joints()
        self.names = list(self.joints)
        self.buses = {}
        ids = {"A": [], "B": []}
        for n, j in self.joints.items(): ids[j["bus"]].append(j["id"])
        if dry_run:
            self.buses = {b: Servos(FakeBus(i)) for b, i in ids.items()}
        else:
            ports = ports or DEFAULT_PORTS
            self.buses = {b: Servos(Bus(ports[b])) for b in ids}
        self.ids = ids
        self.target = {n: 0.0 for n in self.names}
        self.estop = False

    # ---- state
    def read_ticks(self):
        out = {}
        for b, s in self.buses.items():
            got = s.sync_read("present_pos", self.ids[b])
            for n, j in self.joints.items():
                if j["bus"] == b and j["id"] in got: out[n] = got[j["id"]]
        return out

    def read_angles(self):
        t = self.read_ticks()
        return {n: tick_to_angle(self.joints[n], v) for n, v in t.items()}

    def read_health(self):
        h = {}
        for b, s in self.buses.items():
            v = s.sync_read("present_volt", self.ids[b]); t = s.sync_read("present_temp", self.ids[b])
            for n, j in self.joints.items():
                if j["bus"] == b and j["id"] in v: h[n] = dict(volt=v[j["id"]] / 10.0, temp=t.get(j["id"], 0))
        return h

    def check_health(self):
        h = self.read_health()
        for n, v in h.items():
            if v["temp"] >= TEMP_CUT or v["volt"] <= VOLT_CUT:
                self.torque_off(); raise SafetyTrip(f"{n}: temp {v['temp']} C volt {v['volt']} V")
        return h

    # ---- commands
    def torque_on(self):
        for b, s in self.buses.items(): s.torque(self.ids[b], True)
    def torque_off(self):
        for b, s in self.buses.items(): s.torque(self.ids[b], False)

    def configure(self):
        """Write protective EEPROM settings: position limits (from CAD), torque limit, temp/volt cut-offs, gentle P gain."""
        for n, j in self.joints.items():
            s = self.buses[j["bus"]]; i = j["id"]
            lo, hi = body_limits(j)
            t_lo, t_hi = sorted((angle_to_tick(j, lo - 3), angle_to_tick(j, hi + 3)))
            s.write(i, "lock", 0)
            s.write(i, "min_pos", t_lo); s.write(i, "max_pos", t_hi)
            s.write(i, "max_temp", TEMP_CUT); s.write(i, "max_volt", 140); s.write(i, "min_volt", int(VOLT_CUT * 10))
            s.write(i, "mode", 0)
            s.write(i, "p_gain", 16); s.write(i, "d_gain", 32); s.write(i, "i_gain", 0)
            s.write(i, "acceleration", 100)
            s.write(i, "lock", 1)

    def soft_start(self, seconds=2.0):
        """Hold the pose the robot is in, ramp torque up. Never snaps to a stored target."""
        cur = self.read_angles()
        self.target.update(cur)
        self.send(self.target)
        for b, s in self.buses.items():
            s.sync_write("torque_limit", {i: TORQUE_LIMIT_BOOT for i in self.ids[b]})
        self.torque_on()
        steps = 20
        for k in range(steps):
            lim = int(TORQUE_LIMIT_BOOT + (1000 - TORQUE_LIMIT_BOOT) * (k + 1) / steps)
            for b, s in self.buses.items(): s.sync_write("torque_limit", {i: lim for i in self.ids[b]})
            time.sleep(seconds / steps)

    def send(self, angles, max_step_deg=None):
        """Clamp to limits, optionally rate-limit vs. the current target, and sync-write goal positions."""
        if self.estop: return
        goals = {"A": {}, "B": {}}
        for n, a in angles.items():
            j = self.joints[n]
            a = clamp(j, a, margin=0.5)
            if max_step_deg is not None:
                a = self.target[n] + float(np.clip(a - self.target[n], -max_step_deg, max_step_deg))
            self.target[n] = a
            goals[j["bus"]][j["id"]] = angle_to_tick(j, a)
        for b, g in goals.items():
            if g: self.buses[b].sync_write("goal_pos", g)

    def move_to(self, angles, seconds=2.0, hz=50):
        """Smooth (min-jerk) move from the current target to `angles`; unspecified joints hold."""
        start = dict(self.target)
        goal = {n: angles.get(n, start[n]) for n in self.names}
        n_steps = max(1, int(seconds * hz))
        for k in range(1, n_steps + 1):
            u = k / n_steps; s = 10 * u**3 - 15 * u**4 + 6 * u**5
            self.send({n: start[n] + (goal[n] - start[n]) * s for n in self.names})
            time.sleep(1.0 / hz)

    def emergency_stop(self):
        self.estop = True; self.torque_off()

    def close(self):
        try: self.torque_off()
        finally:
            for s in self.buses.values(): s.bus.close()
