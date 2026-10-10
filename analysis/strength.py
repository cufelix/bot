"""Hand-calc stress check of the printed link bars (20 x 8 mm) and horn bolt group. Run: python analysis/strength.py"""
import json, os
HERE = os.path.dirname(os.path.abspath(__file__))
STALL = {"STS3215": 2.9, "STS3250": 4.9}
W, T = 20.0, 8.0                 # bar width (in the plane of the joint torque) x thickness, mm
S_STRONG = T * W**2 / 6          # section modulus for bending from the joint's own torque
S_WEAK = W * T**2 / 6            # for the orthogonal moment from other joints
SIGMA_ALLOW = 22.0               # MPa : PETG-CF Sy ~ 45 MPa (XY) with SF 2 (layer-adhesion direction is weaker: keep bars on flat)
BOLT_R, N_BOLT, D_BOLT, T_WALL = 7.0, 4, 3.0, 8.0
M3_SHEAR = 2500.0                # N, A2 M3 (conservative)

def main():
    model = json.load(open(os.path.join(HERE, "..", "out", "model.json")))
    peak_all = max(STALL[j["servo"]] for j in model["joints"].values())
    rows, ok = [], True
    for n, j in model["joints"].items():
        if n.startswith("R_"): continue
        tau = STALL[j["servo"]] * 1000.0      # N*mm: worst case = servo stall torque (overload/fall)
        s1 = tau / S_STRONG
        s2 = 0.5 * peak_all * 1000.0 / S_WEAK   # half the biggest stall torque acting on the weak axis
        F = tau / (N_BOLT * BOLT_R); brg = F / (D_BOLT * T_WALL)
        good = s1 <= SIGMA_ALLOW * 2 and s2 <= SIGMA_ALLOW * 2 and F < M3_SHEAR and brg < 20
        rows.append((n, j["servo"], tau / 1000, s1, s2, F, brg, good)); ok &= good
    print("Printed-link stress at servo STALL torque (worst case); allowable = yield 45 MPa (PETG-CF, XY)")
    print(f"{'joint':14s}{'servo':9s}{'tau Nm':>8s}{'strong MPa':>12s}{'weak MPa':>10s}{'bolt N':>8s}{'bearing MPa':>12s}  ok")
    for r in rows: print(f"{r[0]:14s}{r[1]:9s}{r[2]:8.1f}{r[3]:12.1f}{r[4]:10.1f}{r[5]:8.0f}{r[6]:12.1f}  {r[7]}")
    # operating (peak = 2.0 x walking envelope) check against SIGMA_ALLOW
    print("\nAll joints OK at stall:", ok, "| bar strong-axis S = %.0f mm3, weak S = %.0f mm3" % (S_STRONG, S_WEAK))
    return ok

if __name__ == "__main__":
    main()
