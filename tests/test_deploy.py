"""End-to-end: the real firmware (Robot, tick conversion, WalkController) drives the MuJoCo robot through an emulated Feetech bus."""
import os, sys
import numpy as np, pytest
ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "firmware"))

def _walk(cmd, seconds):
    from rl.simbus import make
    from humanoid.policy import WalkController
    r, imu, w = make(); r.configure(); r.soft_start(seconds=0.0)
    c = WalkController(r, lambda o: np.zeros(12), imu); c.set_command(*cmd)
    x0 = w.d.qpos[:3].copy(); yaw0 = 0.0
    for _ in range(int(seconds * 50)): c.step()
    return w, x0

@pytest.mark.slow
def test_firmware_walks_forward_in_sim():
    w, x0 = _walk((0.1, 0, 0), 8)
    assert w.d.subtree_com[1][2] > 0.25                    # still upright
    assert w.d.qpos[0] - x0[0] > 0.3                       # made real forward progress

@pytest.mark.slow
def test_firmware_estops_on_fall():
    from rl.simbus import make
    from humanoid.policy import WalkController
    from humanoid.robot import SafetyTrip
    r, imu, w = make(); r.configure(); r.soft_start(seconds=0.0)
    c = WalkController(r, lambda o: np.zeros(12), imu)
    w.d.qpos[3:7] = [0.7, 0.7, 0, 0]                       # tipped 90 deg about x
    with pytest.raises(SafetyTrip): c.step()
    assert r.estop

def test_obs_layout_shared_with_training():
    from humanoid.obs import OBS_DIM
    from rl.env import WalkEnv
    assert WalkEnv(randomize=False).observation_space.shape == (OBS_DIM,)
