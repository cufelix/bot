"""Policy observation layout shared by training (rl/env.py) and the robot runtime (policy.py). Change both sides together."""
import numpy as np

OBS_DIM = 3 + 3 + 3 + 12 + 12 + 12 + 2           # gyro, gravity, command, q-qref, qd, last action, gait clock
ACT_SCALE = 0.15                                  # rad of residual per unit action
CMD_SCALE = np.array([2.0, 2.0, 0.5])

def build_obs(gyro, gravity_body, cmd, q_leg, qref, qd_leg, last_a, phase):
    ph = 2 * np.pi * phase
    return np.concatenate([np.asarray(gyro) * 0.25, gravity_body, np.asarray(cmd) * CMD_SCALE, np.asarray(q_leg) - np.asarray(qref),
                           np.asarray(qd_leg) * 0.05, last_a, [np.sin(ph), np.cos(ph)]]).astype(np.float32)
