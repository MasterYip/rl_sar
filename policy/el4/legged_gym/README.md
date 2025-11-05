# El4 Robot Policy Files

This directory should contain your trained policy files from legged_gym (IsaacGym).

## Required Files

Place your trained policy files in this directory:

1. **policy.pt** - PyTorch JIT model for libtorch inference
2. **policy.onnx** - ONNX model for onnxruntime inference (optional)

## Exporting Policy from legged_gym

If you trained your policy using legged_gym/IsaacGym, you need to export it to JIT format:

```python
import torch
from legged_gym import LEGGED_GYM_ROOT_DIR

# Load your trained model
model_path = "path/to/your/model.pt"
actor = torch.load(model_path)

# Export to JIT
traced_script_module = torch.jit.script(actor)
traced_script_module.save(f"{LEGGED_GYM_ROOT_DIR}/../rl_sar/policy/el4/legged_gym/policy.pt")
```

## Configuration

Make sure the settings in `config.yaml` match your training configuration:

- `num_obs`: Should match your observation space size (66 without height measurements, 253 with)
- `num_actions`: Should be 18 (6 legs × 3 joints)
- `action_scale`: Should match the action scale used during training
- `control_dt`: Should match the control frequency (decimation × sim_dt)

## Joint Order

The joint order follows the physical robot configuration defined in `../base.yaml`:

- RF (Right Front): HAA, HFE, KFE
- RM (Right Middle): HAA, HFE, KFE
- RB (Right Back): HAA, HFE, KFE
- LF (Left Front): HAA, HFE, KFE
- LM (Left Middle): HAA, HFE, KFE
- LB (Left Back): HAA, HFE, KFE

Total: 18 DOFs
