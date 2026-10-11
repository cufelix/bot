# Walking & autonomy (simulation)

Pipeline: `cad/params.py` → URDF → MuJoCo (`rl/model.py`) → PPO (`rl/train.py`) → ONNX → `firmware/humanoid/policy.py` (50 Hz) → Feetech bus.

- Policy = residual around a hand-built reference gait (`rl/gait_ref.py`); controls 12 leg joints, arms/neck hold pose.
- Trained 30M steps on CPU (SB3 PPO, 32 envs), with domain randomisation (gains, mass, friction, latency, noise, pushes).
- Shipped checkpoint: 16M steps (best on benchmark; later ones overfit). `python -m rl.benchmark <ckpt>`.

| command | falls / 10 (randomised + push) |
|---|---|
| fwd 0.10 m/s | 0 |
| fwd 0.18 m/s | 0 |
| sideways 0.06 | 2 |
| turn 0.5 rad/s | 0 |
| stand | 1 |
| back -0.06 | 1 |

Autonomy: head camera → colour-blob detector → SEARCH/APPROACH/ARRIVED navigator → velocity command to the same policy (`python -m autonomy.sim_demo`). See `docs/img/autonomy_demo.gif`.

## Not proven
Sim only. No hardware test, sim-to-real gap unknown. Servo interfaces/masses are assumptions. Tether the robot for first power-up.
