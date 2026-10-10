"""Electrical power / runtime estimate from the MuJoCo walking run (mean |torque| per joint) + holding torque for standing.
Servo electrical model: I = I_idle + |tau|/Kt, Kt = stall_torque/stall_current (output-referred, includes gear losses)."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))
import walk_test as W

SERVO = {"STS3215": dict(stall_nm=2.9, stall_a=2.7, idle_a=0.03), "STS3250": dict(stall_nm=4.9, stall_a=3.5, idle_a=0.04)}  # STS3250 stall current ASSUMED 3.5 A
VBAT = 11.1; CAP_WH = 11.1 * 3.0 * 0.85          # 3S 3000 mAh, 85 % usable
COMPUTE_W = 9.0                                    # Pi 5 under load + camera + IMU + sensors (incl. buck losses)

def main():
    ok, dist, peak = W.run(steps=8, step=40.0, quiet=True)
    mt = W.run.mean_torque
    model = json.load(open(os.path.join(os.path.dirname(__file__), "..", "out", "model.json")))
    amps = 0.0; per = {}
    for n, t in mt.items():
        s = SERVO[model["joints"][n]["servo"]]
        i = s["idle_a"] + t / (s["stall_nm"] / s["stall_a"]); per[n] = i; amps += i
    watts = amps * VBAT + COMPUTE_W
    pk_sum = sum(SERVO[model['joints'][n]['servo']]['idle_a'] + t / (SERVO[model['joints'][n]['servo']]['stall_nm'] / SERVO[model['joints'][n]['servo']]['stall_a']) for n, t in peak.items())
    out = dict(walked=ok, sum_of_joint_peaks_a=round(pk_sum, 1), stall_all_a=round(sum(SERVO[v['servo']]['stall_a'] for v in model['joints'].values()), 1), servo_current_a=round(amps, 2), total_w=round(watts, 1), runtime_min_walking=round(CAP_WH / watts * 60, 0),
               usable_wh=round(CAP_WH, 1), worst_joint_a={k: round(v, 2) for k, v in sorted(per.items(), key=lambda x: -x[1])[:4]})
    print(json.dumps(out, indent=1)); json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "out", "power_report.json"), "w"), indent=1)
    return out

if __name__ == "__main__":
    main()
