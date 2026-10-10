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


class _LoopSerial:
    """Fake pyserial port backed by a FakeBus; honours the byte count Bus asks to read."""
    def __init__(self, fake): self.fake, self.rx = fake, b""
    def reset_input_buffer(self): self.rx = b""
    def flush(self): pass
    def write(self, pkt):
        sid, inst = pkt[2], pkt[4]
        self.rx = self.fake.transact(sid, inst, pkt[5:-1], 0)
    def read(self, n): out, self.rx = self.rx[:n], self.rx[n:]; return out

def test_real_bus_framing_sync_read_and_silent_servo():
    fake = F.FakeBus([1, 2, 3]); bus = F.Bus.__new__(F.Bus); bus.s = _LoopSerial(fake)
    s = F.Servos(bus)
    assert s.sync_read("present_pos", [1, 2, 3]) == {1: 2048, 2: 2048, 3: 2048}
    assert set(s.sync_read("present_pos", [1, 9, 3])) == {1, 3}      # silent servo must not corrupt the rest

def test_soft_start_refuses_missing_servo_and_estop_latches():
    import pytest
    from humanoid.robot import SafetyTrip
    r = Robot(dry_run=True)
    r.buses["A"].bus.mem.pop(r.ids["A"][0])
    with pytest.raises(SafetyTrip): r.soft_start(0)
    r2 = Robot(dry_run=True); r2.emergency_stop()
    with pytest.raises(SafetyTrip): r2.torque_on()

def test_tick_always_in_range():
    j = dict(load_joints()["L_knee"]); j["offset"] = -5000
    assert 0 <= angle_to_tick(j, 130) <= 4095
