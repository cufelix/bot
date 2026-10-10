"""Design-invariant tests: budget, servo count vs CAD, torque margins, printability, clash-free CAD, simulated walking.
Pre-req:  make model config urdf export  (generates out/)"""
import csv, json, os, sys, collections
import pytest
ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in ("analysis", "bom", "sim", "firmware", "cad"):
    sys.path.insert(0, os.path.join(ROOT, p))

def load_model(): return json.load(open(os.path.join(ROOT, "out", "model.json")))

def test_bom_within_budget():
    import build_bom
    t = build_bom.total()
    assert t <= 2000.0, t
    assert t >= 1200.0, "suspiciously cheap - BOM incomplete?"

def test_bom_servo_counts_match_cad():
    import build_bom
    cad = collections.Counter(j["servo"] for j in load_model()["joints"].values())
    bom = {r["item"]: int(r["qty"]) for r in build_bom.load()}
    assert bom["Feetech STS3250 servo"] == cad["STS3250"]
    assert bom["Feetech STS3215 servo (ST-3215-C018)"] == cad["STS3215"] + 1      # +1 spare

def test_model_sane():
    m = load_model()
    assert len(m["joints"]) == 20
    mass = sum(p["mass"] for p in m["parts"]) / 1000
    assert 1.5 < mass < 3.5, mass
    for n, j in m["joints"].items():
        assert j["lo"] < 0 + 1e-9 <= j["hi"] + 1e-9 or j["lo"] <= 0 <= j["hi"], n

def test_torque_envelope_ok():
    import torque
    rep = torque.main(write=False)
    bad = [r for r in rep["envelope"] if not r["ok"]]
    assert not bad, bad
    # stance feasibility: CoM inside the foot in every balance case
    assert all(c.get("feasible", True) for c in rep["cases"].values())

def test_printed_links_strong_enough():
    import strength
    assert strength.main()

def test_print_plan_fits_bed_and_is_watertight():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "out", "print_plan.csv"))))
    assert len(rows) >= 25
    assert all(r["fits_bed"] == "True" and r["watertight"] == "True" for r in rows), [r["part"] for r in rows if r["fits_bed"] != "True" or r["watertight"] != "True"]

def test_joint_config_matches_cad():
    cfg = json.load(open(os.path.join(ROOT, "firmware", "config", "joints.json")))
    m = load_model()["joints"]
    assert set(cfg) == set(m)
    for n, c in cfg.items(): assert (c["lo"], c["hi"]) == (m[n]["lo"], m[n]["hi"]) and c["servo"] == m[n]["servo"]

@pytest.mark.slow
def test_cad_neutral_pose_has_no_clashes():
    import robot as Rb
    from clash import clashes
    parts, J = Rb.build()
    items = {n: Rb.posed(p, J, {}) for n, p in parts.items()}
    assert clashes(items) == []

@pytest.mark.slow
def test_simulated_robot_stands_and_walks():
    import stand_test, walk_test
    ok, _, _ = stand_test.run(3.0, 15.0, quiet=True)
    assert ok
    ok, dist, peak = walk_test.run(steps=6, step=40.0, quiet=True)
    assert ok and dist > 100

def test_body_frame_angles_are_mirror_symmetric():
    """Same body-frame pitch/yaw angles on both sides, mirrored roll, must give mirror-image link positions (catches sign errors)."""
    import numpy as np
    from kinematics import Robot
    R = Robot(); cfg = json.load(open(os.path.join(ROOT, "firmware", "config", "joints.json")))
    body = {"hip_pitch": -25, "knee": 50, "ankle_pitch": -25, "hip_roll": 7, "ankle_roll": -7, "hip_yaw": 9, "sh_pitch": -60, "sh_roll": 20, "elbow": -45}
    ang = {}
    for side, mirror in (("L_", 1), ("R_", -1)):
        for n, v in body.items():
            if n in ("hip_roll", "ankle_roll", "sh_roll", "hip_yaw"): v = v * mirror        # roll/yaw about X/Z flip under y-mirror
            ang[side + n] = cfg[side + n]["sign"] * v
    for p in R.parts:
        if not p["name"].startswith("link_L_"): continue
        q = next(x for x in R.parts if x["name"] == p["name"].replace("link_L_", "link_R_"))
        a = R.place(p["frame"], p["com"], ang); b = R.place(q["frame"], q["com"], ang)
        assert np.allclose(a * [1, -1, 1], b, atol=1.5), (p["name"], a, b)
