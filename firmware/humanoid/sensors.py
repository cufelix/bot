"""Optional hardware sensors (import lazily so the rest runs on a laptop).
IMU BNO085 @0x4A (I2C), 2x ADS1115 @0x48/0x49 (FSR foot sensors), INA219 @0x40 (battery V/I)."""
import math

class IMU:
    def __init__(self):
        import board, busio
        from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
        from adafruit_bno08x.i2c import BNO08X_I2C
        self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA, frequency=400_000), address=0x4A)
        self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR); self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
    def read(self):
        """-> roll, pitch (deg, torso frame: X forward, Y left, Z up), roll_rate, pitch_rate (deg/s).
        MOUNT the board with its X axis pointing forward and Z up (see docs/wiring.md)."""
        qi, qj, qk, qr = self.bno.game_quaternion
        roll = math.degrees(math.atan2(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (qr * qj - qk * qi)))))
        gx, gy, _ = self.bno.gyro
        return roll, pitch, math.degrees(gx), math.degrees(gy)

class FootSensors:
    """8 FSRs in dividers (10k to GND): L foot on ADS1115 0x48 ch0-3 (heel-L, heel-R, toe-L, toe-R), R foot on 0x49."""
    def __init__(self):
        import board, busio, adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
        self.ch = {s: [AnalogIn(ADS.ADS1115(i2c, address=a), p) for p in (ADS.P0, ADS.P1, ADS.P2, ADS.P3)] for s, a in (("L", 0x48), ("R", 0x49))}
    def read(self):
        return {s: [c.voltage for c in v] for s, v in self.ch.items()}
    def contact(self, thresh=0.25):
        r = self.read(); return {s: sum(x > thresh for x in v) >= 2 for s, v in r.items()}

class Battery:
    def __init__(self):
        import board, busio
        from adafruit_ina219 import INA219
        self.ina = INA219(busio.I2C(board.SCL, board.SDA), addr=0x40)
    def read(self):
        return dict(volts=self.ina.bus_voltage, amps=self.ina.current / 1000.0)
