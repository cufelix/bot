# Design notes

## Goals and constraints
- Total build ≤ **$2000** including everything needed to run it (battery, charger, compute, sensors, spares, shipping reserve).
- Fully 3D-printable structure on a 220 × 220 × 250 mm bed; no machining, no custom PCB.
- A real walking-capable biped, not a display model: 6-DOF legs, force sensing, IMU, calibrated serial-bus servos.
- Everything derived from one parametric source so a measurement correction regenerates CAD, joint tables, URDF, analysis and docs.

## Why these actuators
Hobby serial-bus servos are the only class that fits the budget with feedback: 20 joints × ~$40–75. Dynamixel-class actuators would be 5–10× the cost.

| | STS3215 (12 V) | STS3250 (12 V) |
|---|---|---|
| Stall torque | 2.9 N·m (30 kg·cm) | 4.9 N·m (50 kg·cm) |
| Rated torque | 1.0 N·m | 1.6 N·m |
| Mass | 55 g | 75 g (assumed) |
| Used for | hip yaw, ankle roll, all arm + neck joints (12) | hip roll, hip pitch, knee, ankle pitch (8) |
| Est. price | $35 | $75 |

Sizing came from the torque analysis, not the other way round: `analysis/torque.py` posed the CAD mass model through single-support squats with the CoM leaned over the stance foot, CoP shifts, swing-leg holds and arm reaches. The first version had STS3215 at hip roll and ankle pitch; both exceeded the 65 %-of-stall criterion (hip roll static 1.7 N·m because a 104 mm hip spacing forces ~11° of lean over one foot), so they were upgraded. MuJoCo walking then showed the ankle pitch reaching 59 % of stall at only 40 mm steps in a 3215, which is why ankle pitch is a 3250 too.

## Kinematics
Frame: X forward, Y left, Z up, origin on the floor between the feet. Joint axes (left leg, neutral):

| Joint | Axis | Position (x, y, z mm) |
|---|---|---|
| hip yaw | Z | 0, 52, 362 |
| hip roll | X | −30, 52, 328 |
| hip pitch | Y | 0, 52, 294 |
| knee | Y | 0, 51, 184 |
| ankle pitch | Y | 0, 51, 74 |
| ankle roll | X | −24, 52, 40 |

Thigh and shank are both 110 mm; the 34 mm offsets between cross-axis joints are the width of one servo body, the price of not having a spherical hip. The firmware uses numeric IK (`humanoid/kinematics.py`, damped least squares with joint bounds) so these offsets need no special casing; ~1 ms per leg.

Body-frame angle convention (firmware, viewer, gait): rotation about +X (roll), +Y (pitch), +Z (yaw), identical sign on both sides. `sign` in `firmware/config/joints.json` converts to the servo's out-axis angle; `dir` in `calibration.json` fixes physical servo direction during bring-up. A test (`test_body_frame_angles_are_mirror_symmetric`) guards this.

## Structure
Every link is generated, not drawn:
- **Cradle** — 3 mm shell that holds one servo (0.3 mm print clearance, open on the horn side, 4 back-wall screw holes).
- **Flange** — Ø28 × 8 mm disc bolted to the horn (4 × M3 on r = 7 mm + centre screw).
- **Bars** — 20 × 8 mm, thickness along the joint axis so the servo's own torque is carried on the strong axis; routed in waypoints so they never cross a neighbouring servo or the next flange (`cad/skeleton.py` routes).
- Custom: pelvis (2 hip-yaw cradles + deck), torso shell (battery/Pi bay, shoulder + neck cradles, rear cover), neck fork with a 623 bearing on the servo's rear shaft, head shell with camera window, foot (FSR pockets + TPU sole) and mitten hand.

Collision-free joint ranges came out of `cad/sweep.py` (each joint swept to its limit with all others at zero) plus `cad/pairtest.py` (weight-shift, crouch, swing). Final body-frame ranges are in [ASSEMBLY_TABLE](ASSEMBLY_TABLE.md). The firmware also writes them into each servo's EEPROM as hard position limits (±3°).

## Balance and gait
`firmware/humanoid/gait.py` is a **quasi-static** walker: shift the pelvis over the stance foot, swing the other foot (min-jerk, 14 mm lift), repeat. It is slow (~0.6 steps/s at 40 mm) but needs no dynamic model. `stabilizer.py` adds a small pelvis offset proportional to IMU roll/pitch and rates. In MuJoCo (PD servos, torque-limited to the servo stall values, 1 kHz) it walks 40–60 mm steps; 80 mm steps saturate the ankle. It is the starting point, not a finished controller: the foot FSRs are read by `sensors.py` but not yet used in the loop (touchdown detection and ZMP feedback are the obvious next step).

## Power
12 V-class servos run straight from the 3S pack (9.6–12.6 V, servo limit 14 V). A Pololu D24V50F5 supplies the Pi at 5 V / 5 A. A star distribution on two barrier strips feeds each limb separately so no limb draws through another limb's connectors. See [WIRING](WIRING.md).

## Assumptions that carry risk
1. **Servo mechanical interface** (see README table). All dependent geometry is in one dictionary.
2. **Printed-part strength** was hand-calculated (bar bending at servo stall, M3 bolt group, bearing stress), not FEA. PETG-CF yield 45 MPa (XY) assumed; layer-adhesion direction is weaker, hence the print orientations.
3. **Servo thermals**: single-leg standing loads hip roll and knee at ≈1× *rated* torque. Walking gives these joints ~50 % duty, which is typical for this servo class, but check temperatures (`cli.py monitor`) during the first sessions. The firmware cuts torque at 70 °C.
4. **Simulation fidelity**: ideal PD actuators with torque limit; no backlash, deadband, latency or voltage sag.

## Possible upgrades (not in budget)
Waist yaw (+$35), parallel dual servo at the knee, gripper hands (+2 × $35 + printed fingers), depth camera, second battery in the pelvis to lower the CoM.
