"""Single source of truth for the humanoid. All lengths mm, masses g, torque N*m.
Frame: X forward, Y left, Z up, origin on floor between the feet."""

# --- servo envelopes (Feetech serial-bus). VERIFY against datasheet drawing/caliper before printing. ---
# local servo frame: shaft axis = +Z (horn side), body spans z in [-T,0]; long axis = +X, shaft at x=0.
SERVOS = {
    "STS3215": dict(L=45.2, W=35.0, T=24.7, shaft_back=11.0, mass=55, stall=2.9, rated=1.0, price=35.0, kg_cm=30),
    "STS3250": dict(L=45.2, W=35.0, T=24.7, shaft_back=11.0, mass=75, stall=4.9, rated=1.6, price=75.0, kg_cm=50),
}
HORN_H = 3.0          # horn disc thickness above shaft plane
HORN_D = 20.0
FLANGE_T = 8.0        # printed flange thickness bolted on horn
HORN_BOLT_R = 7.0     # 4x M3 on this radius (placeholder pattern, check horn)
CRADLE_WALL = 3.0
CLEAR = 0.3           # print clearance around servo

# --- skeleton ---
HIP_HALF = 52.0       # hip joint y offset (hip spacing 104)
SHOULDER_HALF = 78.0
THIGH = 110.0
SHANK = 110.0
ANKLE_ROLL_Z = 40.0   # ankle roll axis height above floor
OFFSET = 34.0         # axis-to-axis stacking offset of cross joints
FOOT_L, FOOT_W = 150.0, 70.0
FOOT_X0 = -45.0       # heel x
UPPER_ARM = 85.0
FOREARM = 95.0
BED = (220.0, 220.0, 250.0)   # max printable part envelope
INFILL_DENSITY = 1.24e-3 * 0.40   # g/mm3: PETG 1.27 at ~40% effective
