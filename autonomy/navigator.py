"""Behaviour layer: turns camera detections into velocity commands for the walking policy.
SEARCH (turn on the spot until the target is seen) -> APPROACH (steer by bearing, speed by apparent size) -> ARRIVED (stand).
Lost target for > LOST_S seconds returns to SEARCH. The walking policy and its safety limits are unchanged below this layer."""
import numpy as np

VX_MAX, WZ_MAX = 0.14, 0.5
ARRIVE_SIZE = 0.33          # blob height as a fraction of image height at the stop distance
LOST_S = 1.0

class Navigator:
    def __init__(self, dt=0.1):
        self.state, self.dt, self.lost, self.search_dir = "SEARCH", dt, 0.0, 1.0
        self.last_bearing, self.last_size, self.last_elev = 0.0, 0.0, 0.0

    def step(self, det):
        """det: output of vision.detect or None. Returns (cmd[vx,vy,wz], state)."""
        if det is None:
            self.lost += self.dt
            if self.state == "ARRIVED": return np.zeros(3), self.state
            if self.state == "APPROACH" and (self.last_size > 0.2 or self.last_elev < -0.3) and self.lost >= 0.3:    # ball slipped under the camera's view: it is at our feet
                self.state = "ARRIVED"; return np.zeros(3), self.state
            if self.state == "APPROACH" and self.lost < LOST_S:      # coast toward last bearing
                return np.array([0.04, 0.0, np.clip(1.2 * self.last_bearing, -WZ_MAX, WZ_MAX)]), self.state
            self.state = "SEARCH"
            self.search_dir = np.sign(self.last_bearing) or self.search_dir
            return np.array([0.0, 0.0, 0.4 * self.search_dir]), self.state
        self.lost = 0.0; self.last_bearing = det["bearing"]; self.last_size = det["size"]; self.last_elev = det["elevation"]
        if det["size"] >= ARRIVE_SIZE: self.state = "ARRIVED"; return np.zeros(3), self.state
        self.state = "APPROACH"
        wz = float(np.clip(1.5 * det["bearing"], -WZ_MAX, WZ_MAX))
        align = max(0.0, 1.0 - abs(det["bearing"]) / 0.6)                    # slow down while the target is off-axis
        vx = VX_MAX * align * float(np.clip((ARRIVE_SIZE - det["size"]) / 0.15, 0.3, 1.0))
        return np.array([vx, 0.0, wz]), self.state
