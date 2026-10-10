import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))
from humanoid import feetech as F
from humanoid.joints import load_joints, angle_to_tick, tick_to_angle, body_limits, clamp
from humanoid.robot import Robot

def test_ping_packet_matches_datasheet_example():
    assert F.packet(1, F.PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])

def test_write_goal_packet():
    # goal_pos=2048 on id 3: FF FF 03 05 03 2A 00 08 chk
    p = F.packet(3, F.WRITE, bytes([42]) + F.pack(2, 2048))
    assert p[:8] == bytes([0xFF, 0xFF, 3, 5, 3, 42, 0x00, 0x08]) and p[-1] == F.checksum(p[2:-1])

def test_status_roundtrip():
    st = F.packet(7, 0, b"\x34\x12")
    assert F.parse_status(b"\x00" + st) == (7, 0, b"\x34\x12")
    bad = bytearray(st); bad[-1] ^= 1
    assert F.parse_status(bytes(bad)) is None

def test_sign_magnitude():
    for v in (-1000, -1, 0, 5, 2000):
        assert F.decode_sign_magnitude(F.encode_sign_magnitude(v, 11), 11) == v

def test_joint_roundtrip_and_limits():
    J = load_joints(calib="/nonexistent")
    assert len(J) == 20 and len({j["id"] for j in J.values()}) == 20
    for n, j in J.items():
        lo, hi = body_limits(j)
        assert lo <= 0 <= hi, n
        for a in (lo, 0.0, hi):
            assert abs(tick_to_angle(j, angle_to_tick(j, a)) - a) < 360 / 4096
        assert clamp(j, 1e3) <= hi and clamp(j, -1e3) >= lo
        assert 0 < angle_to_tick(j, lo) < 4095 and 0 < angle_to_tick(j, hi) < 4095

def test_robot_dry_run_moves_and_clamps():
    r = Robot(dry_run=True)
    r.configure(); r.soft_start(seconds=0.0)
    r.send({"L_knee": 60.0, "R_knee": 500.0})        # second one must clamp
    a = r.read_angles()
    assert abs(a["L_knee"] - 60.0) < 0.2
    assert a["R_knee"] <= body_limits(r.joints["R_knee"])[1]
    h = r.check_health(); assert all(v["volt"] == 12.0 for v in h.values())
    r.emergency_stop(); r.send({"L_knee": 10.0}); assert abs(r.read_angles()["L_knee"] - 60.0) < 0.2
