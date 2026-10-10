# Print guide

27 printed parts (including the 2 TPU soles). Net model mass **559 g**, estimated **55 h** of printing (very rough: mass / 12 g/h + 0.3 h per part).
Ready-to-slice STLs (already rotated to a stable face-down pose, origin at the bed corner): `out/stl_print/`. STEP: `out/step/`.

## Material and settings

| Group | Material | Nozzle / layer | Walls | Infill | Notes |
|---|---|---|--:|--:|---|
| Links, pelvis, torso (PETG-CF) | PETG-CF (or PA-CF) | 0.4 / 0.2 | 4 | 40 % gyroid | Dry the filament. Print bars flat (as oriented) so layers run along the bar. Do not use PLA: servo cradles get warm. |
| Head, covers, Pi tray (PETG) | PETG | 0.4 / 0.2 | 3 | 20 % | |
| Foot soles (TPU 95A) | TPU 95A | 0.4 / 0.2 | 3 | 100 % | 20 mm/s, no retraction tuning needed on direct drive |

After printing: ream every servo pocket and 3.4 mm hole with the matching drill by hand; the pockets are 0.3 mm oversize (see `CLEAR` in `cad/params.py`). Tune `CLEAR` for your printer, re-run `make export`.

## Part list

| Part | Material | Qty | Mass g | Size mm | Est. h |
|---|---|--:|--:|---|--:|
| link_L_hip_yaw | PETG-CF | 1 | 14.3 | 72x68x41 | 1.5 |
| link_L_hip_roll | PETG-CF | 1 | 10.9 | 48x62x42 | 1.2 |
| link_L_hip_pitch | PETG-CF | 1 | 16.8 | 41x143x44 | 1.7 |
| link_L_knee | PETG-CF | 1 | 16.8 | 41x143x43 | 1.7 |
| link_L_ankle_pitch | PETG-CF | 1 | 13.3 | 66x68x51 | 1.4 |
| link_L_ankle_roll | PETG-CF | 1 | 34.5 | 150x70x52 | 3.2 |
| link_L_sh_pitch | PETG-CF | 1 | 13.4 | 60x50x53 | 1.4 |
| link_L_sh_roll | PETG-CF | 1 | 13.8 | 41x113x34 | 1.4 |
| link_L_elbow | PETG-CF | 1 | 22.6 | 34x169x18 | 2.2 |
| link_R_hip_yaw | PETG-CF | 1 | 14.3 | 72x68x41 | 1.5 |
| link_R_hip_roll | PETG-CF | 1 | 10.9 | 48x62x42 | 1.2 |
| link_R_hip_pitch | PETG-CF | 1 | 16.8 | 41x143x44 | 1.7 |
| link_R_knee | PETG-CF | 1 | 16.8 | 41x143x43 | 1.7 |
| link_R_ankle_pitch | PETG-CF | 1 | 13.3 | 66x68x51 | 1.4 |
| link_R_ankle_roll | PETG-CF | 1 | 34.5 | 150x70x52 | 3.2 |
| link_R_sh_pitch | PETG-CF | 1 | 13.4 | 60x50x53 | 1.4 |
| link_R_sh_roll | PETG-CF | 1 | 13.8 | 41x113x34 | 1.4 |
| link_R_elbow | PETG-CF | 1 | 22.6 | 34x169x18 | 2.2 |
| sole_L_foot | TPU 95A | 1 | 7.9 | 150x70x2 | 1.0 |
| sole_R_foot | TPU 95A | 1 | 7.9 | 150x70x2 | 1.0 |
| pelvis | PETG-CF | 1 | 34.7 | 110x152x28 | 3.2 |
| torso_shell | PETG-CF | 1 | 101.9 | 130x120x106 | 8.8 |
| torso_cover | PETG | 1 | 18.2 | 126x118x3 | 1.8 |
| pi_tray | PETG | 1 | 10.0 | 63x120x3 | 1.1 |
| neck_fork | PETG-CF | 1 | 10.9 | 44x51x28 | 1.2 |
| head_shell | PETG | 1 | 48.9 | 72x80x75 | 4.4 |
| head_cover | PETG | 1 | 5.4 | 63x58x3 | 0.8 |
