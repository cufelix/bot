# Bring-up and calibration

Order matters. Keep the robot **on a stand (feet off the floor) until step 7**.

## 1. Pi setup
```bash
sudo apt install python3-venv python3-pip i2c-tools
python3 -m venv ~/venv && . ~/venv/bin/activate && pip install pyserial numpy scipy adafruit-circuitpython-bno08x adafruit-circuitpython-ads1x15 adafruit-circuitpython-ina219
sudo raspi-config nonint do_i2c 0
sudo rpi-eeprom-config --edit        # add the line PSU_MAX_CURRENT=5000  (Pi 5 powered from GPIO 5 V pins, not USB-C PD)
echo 'usb_max_current_enable=1' | sudo tee -a /boot/firmware/config.txt
```
Create stable names for the two adapters (find serials with `udevadm info /dev/ttyACM0`), `/etc/udev/rules.d/99-humanoid.rules`:
```
SUBSYSTEM=="tty", ATTRS{serial}=="<legs adapter serial>",  SYMLINK+="humanoid_legs"
SUBSYSTEM=="tty", ATTRS{serial}=="<upper adapter serial>", SYMLINK+="humanoid_upper"
```
(these are the defaults; override with `HUMANOID_PORT_LEGS` / `HUMANOID_PORT_UPPER`).
`i2cdetect -y 1` must show 0x40, 0x48, 0x49, 0x4A.

## 2. Power-on check on a current-limited bench supply
12 V, limit 3 A, no servos moving. Read idle draw: 20 servos ≈ 0.6–1.0 A. Buck output 5.0 V ± 0.1. Then pack voltage via INA219.

## 3. Bus check
```bash
python firmware/cli.py ping          # all 20 must answer; a missing one = ID or cable
python firmware/cli.py monitor       # voltage ~ pack voltage, temperatures ambient
```

## 4. Direction calibration (`firmware/config/calibration.json`)
For each joint in turn, with the robot held in the air, command +10° (`humanoid.joints.angle_to_tick`) and observe: if the physical rotation follows the right-hand rule about the axis in [ASSEMBLY_TABLE](ASSEMBLY_TABLE.md) (+X roll: the left side goes up; +Y pitch: points above the axis move forward, so a hanging leg swings *backward*; +Z yaw: counter-clockwise seen from above), keep `dir: 1`, otherwise set `-1`. Quick script:
```python
from humanoid.robot import Robot
r = Robot(); r.configure(); r.soft_start()
r.move_to({"L_knee": 20}, 1.5)   # knee positive = shank swings backward
```
Knee positive = shank backwards, hip pitch positive = leg backwards, elbow negative = forearm forward, shoulder roll positive (left) = arm out sideways, neck pitch positive = look down.

## 5. Zero offsets
Set the robot in the neutral pose using a flat plate/square (legs straight, feet flat and parallel, arms down). Read `r.read_ticks()`; for each joint `offset = tick - 2048` goes in `calibration.json`. If servos were centred at assembly (BUILD_GUIDE §2) these are within ±40 ticks (±3.5°); larger means a horn was mounted off-centre.

## 6. Limit check
The firmware writes ±3° margin hard limits into the servo EEPROM (`Robot.configure()`). Verify one joint at a time by commanding beyond the limit: the servo must stop at the limit. A joint that hits a printed part first has a calibration offset error.

## 7. Stand
Torque ramps in over 2 s from the *current* pose (`soft_start`), so a robot sagging on the stand does not snap. Then:
```bash
python firmware/cli.py stand      # lower onto a mat while holding the shoulders, release once the legs bear weight
python firmware/cli.py crouch
```
Watch temperatures: `cli.py monitor`. Hip roll and knee run warmest.

## 8. Walk (tethered first)
```bash
python firmware/cli.py walk --steps 4 --step-mm 30            # open loop
python firmware/cli.py walk --steps 4 --step-mm 40 --imu      # with tilt feedback
```
Hang a light line from the torso top (cable tie loop) to a boom; keep slack. If it rocks laterally, reduce `shift_frac`/raise `t_shift` or lower `kp_y` in `stabilizer.py`; if the swing foot scuffs, raise `lift`. Expect to spend an afternoon tuning; the simulation reference values are in `sim/walk_test.py`.

## 9. Record-keeping
Commit your `calibration.json` with the serial numbers of the servos; swapped servos need re-centring (`servo_setup.py --center`).
