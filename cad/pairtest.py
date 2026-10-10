"""Combined-pose collision tests (both legs / both arms), run after sweep.py."""
import robot as Rb
from clash import bb_overlap, vol
parts, J = Rb.build()
def test(name, ang):
    shapes = {n: Rb.posed(p, J, ang) for n, p in parts.items()}
    bad = []
    names = list(shapes)
    moving = {n for n, p in parts.items() if any(f in ang for f in Rb.chain(p.frame, J))}
    for m in moving:
        for n in names:
            if n in moving and n < m: continue
            if n == m: continue
            # only flag pairs whose relative pose changed: one in moving set and the other is outside OR both moving but from different joints
            if bb_overlap(shapes[m], shapes[n]) and vol(shapes[m], shapes[n]) > 1.0:
                bad.append((m, n))
    print(f"{name:34s}", "OK" if not bad else bad[:4])
# CoM over right foot: left leg abducts, right leg adducts (feet flat via ankle roll)
for r in (8, 12, 15):
    test(f"weight-shift right roll={r}", {"L_hip_roll": r, "L_ankle_roll": -r, "R_hip_roll": r, "R_ankle_roll": -r})
    test(f"weight-shift left roll={r}", {"L_hip_roll": -r, "L_ankle_roll": r, "R_hip_roll": -r, "R_ankle_roll": r})
test("crouch 30 + roll 10", {"L_hip_roll": 10, "R_hip_roll": 10, "L_hip_pitch": -30, "L_knee": 60, "L_ankle_pitch": -30, "R_hip_pitch": 30, "R_knee": -60, "R_ankle_pitch": 30, "L_ankle_roll": -10, "R_ankle_roll": -10})
test("step swing leg (L lifted, R stance)", {"L_hip_pitch": -35, "L_knee": 60, "L_ankle_pitch": -25})
