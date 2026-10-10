"""Torso-tilt feedback: shifts the commanded pelvis position against measured roll/pitch (IMU).
roll/pitch in degrees (roll about +X: left side down = negative), rates in deg/s."""
class Stabilizer:
    def __init__(self, kp_y=1.2, kd_y=0.12, kp_x=1.0, kd_x=0.1, limit=20.0):
        self.kp_y, self.kd_y, self.kp_x, self.kd_x, self.limit = kp_y, kd_y, kp_x, kd_x, limit
    def correction(self, roll, pitch, roll_rate, pitch_rate):
        """returns (dx, dy) mm to ADD to the commanded pelvis position"""
        dy = self.kp_y * roll + self.kd_y * roll_rate        # left-down (roll<0) -> move pelvis right (-y)
        dx = -(self.kp_x * pitch + self.kd_x * pitch_rate)    # nose-down (pitch>0) -> move pelvis back (-x)
        c = lambda v: max(-self.limit, min(self.limit, v))
        return c(dx), c(dy)
