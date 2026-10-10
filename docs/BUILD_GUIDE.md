# Build guide

Time: ~55 h printing, ~12 h assembly + wiring, ~4 h bring-up. Skills: soldering, crimping, basic Linux.
Tools (not in the BOM): 3D printer ≥ 220 mm bed with a PETG-CF-capable nozzle (hardened), soldering iron, wire strippers/crimper, hex drivers (2.0/2.5), digital calipers, multimeter, hobby knife, 3.4 mm / 2.5 mm drills, heat gun, LiPo-safe charging spot. No printer: order the 27 parts from a print service (PETG/nylon-CF, ≈ $120–200; not in the BOM total).

## 0. Verify the servo interface (do this before printing in bulk)
1. When the servos arrive, measure with calipers: body length / width / thickness, shaft centre offset from the end, horn outer diameter, horn bolt circle and thread, and the 4 screw-hole positions on the servo bottom/sides.
2. Compare with `SERVOS`, `HORN_D`, `HORN_BOLT_R`, `CLEAR` and the cradle hole pattern in `cad/kin.py:cradle()` / `cad/params.py`. Edit; `make all` regenerates everything (STLs, URDF, analysis, joint config, docs) and re-runs the collision and torque checks.
3. Print `pelvis` (two cradles) and one `link_*_hip_yaw` flange. Fit a servo, check the horn seats flush, the pocket is not tight, the screws bite. Adjust `CLEAR`. Only then print the rest.

## 1. Print
See [PRINT_GUIDE](PRINT_GUIDE.md): 27 parts, materials, orientation and settings. Left/right parts are separate models (not mirrored copies in the slicer). Label parts with a paint pen (`link_L_knee` …).

## 2. Prepare every servo (bench, one at a time)
Do this **before** mounting anything.
```bash
# power: servo to the Waveshare adapter, adapter's DC jack to a 12 V supply or the battery via fuse
python tools/servo_setup.py --port /dev/ttyACM0 --set-id <N>      # N from docs/ASSEMBLY_TABLE.md (factory ID is 1)
python tools/servo_setup.py --port /dev/ttyACM0 --id <N> --info   # confirm 12 V reading and temperature
```
Then centre: turn the output shaft **by hand** to the mechanical middle of the joint it will serve (the printed link must sit at its neutral pose, see the viewer), and run
`python tools/servo_setup.py --port /dev/ttyACM0 --id <N> --center` — this declares tick 2048 = joint zero. Mount the horn at this position. This is why `offset` in calibration is normally ~0. Label ID on the servo and both ends of its cable.

## 3. Mechanical assembly — legs first
Order for **each leg** (left shown; right mirrors with the `R_` parts). Servo "sits in" / "horn drives" per joint is in [ASSEMBLY_TABLE](ASSEMBLY_TABLE.md).
1. Foot: glue TPU `sole_*_foot` to `link_*_ankle_roll` (contact cement), FSRs into the 4 pockets first (leads out through the edge notch), 0.5 mm Kapton over each.
2. Servo 6 (ankle roll) into the `link_*_ankle_pitch` cradle (4 × M3 through the back wall). Horn on, then bolt `link_*_ankle_roll` (the foot) to the horn: 4 × M3×8 on the flange + 1 centre M3×6 with threadlocker.
3. Servo 5 (ankle pitch) into `link_*_knee` cradle; bolt `link_*_ankle_pitch` to its horn.
4. Servo 4 (knee) into `link_*_hip_pitch`; bolt `link_*_knee` to its horn.
5. Servo 3 (hip pitch) into `link_*_hip_roll`; bolt `link_*_hip_pitch`.
6. Servo 2 (hip roll) into `link_*_hip_yaw`; bolt `link_*_hip_roll`.
7. Servo 1 (hip yaw) into the `pelvis` cradle (do both legs, pelvis upside down on foam); bolt `link_*_hip_yaw`.
At each step cycle the joint through its range **by hand with the servo unpowered**: it must move freely to the limits in the table without a clash. Servo gears are not back-drivable cheaply; do not force them.

## 4. Torso, head, arms
1. Insert M3 heat-set inserts in the 4 pelvis deck pockets. Screw `torso_shell` to the pelvis (4 × M3×12 from below).
2. Shoulder servos 13 and 16 into the side-wall cradles (horn pokes through the 23 mm hole); neck yaw servo 19 into the top cradle.
3. Arm: `link_*_sh_pitch` on the shoulder horn, servo 14/17 (shoulder roll) in its cradle, `link_*_sh_roll` on that horn, elbow servo 15/18 in the upper-arm cradle, `link_*_elbow` (forearm + mitten hand) on the elbow horn.
4. Neck: `neck_fork` on neck-yaw horn. Servo 20 (neck pitch) into the `head_shell` cradle; slide the head over the fork so the servo horn meets the fork's flange plate; press the 623 bearing into the fork's rear plate over the servo's rear shaft; bolt the horn flange. Camera into the front window (M2 × 4), eye LEDs optional, `head_cover` on 2 M3.
5. Rear: fit the kill switch through `torso_cover`, close with 4 × M3 self-tappers.

## 5. Electronics (torso)
Layout is modelled — see the coloured boxes in the viewer or `cad/parts.py: electronics()`:
- Floor: battery (centre, Velcro strap), 2 bus adapters (rear-left/right), IMU, ADS1115 pair (front).
- Battery: slide in from the rear opening, Velcro strap through the 4 pelvis-deck slots.
- Layer 2: `pi_tray` rests on the 4 torso ledges above the battery; Pi 5 on 4 × M2.5×6 standoffs on its 58 × 49 pattern; power distribution strips + buck on the left (foam tape / zip ties).
Wire per [WIRING](WIRING.md). Routing: all servo cables go through the pelvis slot to the legs, up the side to arms and neck; use spiral wrap and a stationary tie-down on every link, free loop at each joint.

## 6. First power
Do **not** connect the battery yet. Follow [BRINGUP](BRINGUP.md) (bench supply with current limit, then IDs, then calibration, then torque-limited tests). Read [SAFETY](SAFETY.md).

## Fastener schedule (estimate; verify counts when printing)
| Fastener | Use | Qty |
|---|---|--:|
| M3×6 | servo → cradle (4/servo), centre horn screw | ~100 |
| M3×8 | horn → flange (4/joint) | ~84 |
| M3×12 | pelvis ↔ torso, covers, head | ~14 |
| M3 heat-set insert | pelvis deck, accessories | 8 |
| M3 nylock + washers | bearing keeper, camera, hub | ~12 |
| M2.5 × 6 + standoff | Pi 5, boards | ~20 |
