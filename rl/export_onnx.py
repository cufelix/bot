"""Export a trained SB3 PPO policy to ONNX (deterministic mean action, obs 47 -> act 12). Usage: python -m rl.export_onnx out/rl/run3/final.zip firmware/config/walk_policy.onnx"""
import sys, torch, numpy as np
from stable_baselines3 import PPO

class Net(torch.nn.Module):
    def __init__(self, p): super().__init__(); self.p = p
    def forward(self, obs):
        feats = self.p.mlp_extractor.forward_actor(self.p.extract_features(obs, self.p.pi_features_extractor))
        return torch.clamp(self.p.action_net(feats), -1.0, 1.0)

def export(ckpt, out):
    m = PPO.load(ckpt, device="cpu"); net = Net(m.policy).eval(); x = torch.zeros(1, m.observation_space.shape[0])
    torch.onnx.export(net, x, out, input_names=["obs"], output_names=["action"], opset_version=17, dynamo=False)
    import onnxruntime as ort
    s = ort.InferenceSession(out); y = s.run(None, {"obs": x.numpy()})[0]
    obs = np.random.randn(1, x.shape[1]).astype(np.float32)
    ref = m.predict(obs[0], deterministic=True)[0]; got = s.run(None, {"obs": obs})[0][0]
    assert np.allclose(np.clip(ref, -1, 1), got, atol=1e-4), (ref, got)
    return out

if __name__ == "__main__":
    print("exported", export(sys.argv[1], sys.argv[2]))
