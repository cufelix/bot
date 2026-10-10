# Joint / part assembly table (auto-generated)

Each joint is one servo. The **servo body** sits in a printed cradle; the servo **horn** carries the next printed link, bolted with 4×M3 + 1 centre M3.
Servo IDs are fixed here - label cables and servos before assembly.

| # | Joint | Servo | Bus | Axis | Range (body°) | Servo body sits in | Horn drives |
|--:|---|---|---|---|---|---|---|
| 1 | L_hip_yaw | STS3215 | A | Z | -20 … 30 | `pelvis` | `link_L_hip_yaw` |
| 2 | L_hip_roll | STS3250 | A | X | -15 … 25 | `link_L_hip_yaw` | `link_L_hip_roll` |
| 3 | L_hip_pitch | STS3250 | A | Y | -90 … 40 | `link_L_hip_roll` | `link_L_hip_pitch` |
| 4 | L_knee | STS3250 | A | Y | 0 … 130 | `link_L_hip_pitch` | `link_L_knee` |
| 5 | L_ankle_pitch | STS3250 | A | Y | -50 … 50 | `link_L_knee` | `link_L_ankle_pitch` |
| 6 | L_ankle_roll | STS3215 | A | X | -20 … 20 | `link_L_ankle_pitch` | `link_L_ankle_roll` |
| 7 | R_hip_yaw | STS3215 | A | Z | -30 … 20 | `pelvis` | `link_R_hip_yaw` |
| 8 | R_hip_roll | STS3250 | A | X | -25 … 15 | `link_R_hip_yaw` | `link_R_hip_roll` |
| 9 | R_hip_pitch | STS3250 | A | Y | -90 … 40 | `link_R_hip_roll` | `link_R_hip_pitch` |
| 10 | R_knee | STS3250 | A | Y | 0 … 130 | `link_R_hip_pitch` | `link_R_knee` |
| 11 | R_ankle_pitch | STS3250 | A | Y | -50 … 50 | `link_R_knee` | `link_R_ankle_pitch` |
| 12 | R_ankle_roll | STS3215 | A | X | -20 … 20 | `link_R_ankle_pitch` | `link_R_ankle_roll` |
| 13 | L_sh_pitch | STS3215 | B | Y | -170 … 90 | `torso_shell` (side wall) | `link_L_sh_pitch` |
| 14 | L_sh_roll | STS3215 | B | X | -5 … 100 | `link_L_sh_pitch` | `link_L_sh_roll` |
| 15 | L_elbow | STS3215 | B | Y | -130 … 5 | `link_L_sh_roll` | `link_L_elbow` |
| 16 | R_sh_pitch | STS3215 | B | Y | -170 … 90 | `torso_shell` (side wall) | `link_R_sh_pitch` |
| 17 | R_sh_roll | STS3215 | B | X | -100 … 5 | `link_R_sh_pitch` | `link_R_sh_roll` |
| 18 | R_elbow | STS3215 | B | Y | -130 … 5 | `link_R_sh_roll` | `link_R_elbow` |
| 19 | neck_yaw | STS3215 | B | Z | -80 … 80 | `torso_shell` (top plate) | `neck_fork` |
| 20 | neck_pitch | STS3215 | B | Y | -15 … 15 | `head_shell` (servo body moves with the head) | `neck_fork` (head turns on it) |

Servo count: 12× STS3215, 8× STS3250 (+1 spare STS3215 in the BOM).

