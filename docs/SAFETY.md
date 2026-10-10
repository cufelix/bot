# Safety

- **LiPo**: charge in a LiPo bag, supervised, balance charger at 1 C, store at 3.8 V/cell. Never leave a charging pack unattended. Stop using a puffed or dropped pack.
- **Fuse and kill switch**: the 20 A fuse and latching 30 A kill switch are mandatory. The switch is on the robot's back where you can reach it; practise before first power.
- **Pinch points**: servos have ≥ 2.9 N·m; the hip, knee and shoulder bars close like scissors. Keep fingers out when torque is enabled; `python firmware/cli.py relax` drops torque.
- **Falls**: a 2.5 kg robot at 0.6 m can break printed links and horns; start on a mat, tethered. Switch off before inspecting.
- **Servo protection** (set by `Robot.configure()`): hard position limits, 70 °C thermal cut, 9.6 V under-voltage cut, torque ramp at start, software e-stop. These are in servo EEPROM, so they survive a crashed Pi.
- **Electrical**: no loose strands (ferrules on the strips), heat-shrink every splice, no wires crossing a joint axis without a loop, insulation on the barrier strip.
- **Thermal**: never run the legs at full stand for > 10 min on the first day; check temperatures every few minutes.
- **Children / pets**: small parts and pinch hazards; not a toy.
