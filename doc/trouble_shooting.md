# Troubleshooting

### 20260618 PhysX error: GPU updateFrictionPatches fail to launch kernel

**RootCause**: 
The G1 29dof velocity task config at velocity_env_cfg.py:382 sets:

```python
self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15  # 327,680 — double the default
```

Combined with running PhysX GPU dynamics on an RTX 4070 Laptop GPU (which also drives the display), the GPU compute kernels for collision detection time out, causing the GPU * fail to launch kernel!! errors.

#### Option 1: Use CPU physics (recommended for play.py)

python scripts/rsl_rl/play.py --task Unitree-G1-29dof-Velocity --num_envs 1 --device cpu
This is safe for inference. It runs all physics on the CPU, which won't timeout. For playing back a trained policy at 1 environment, CPU is perfectly adequate.

#### Option 2: Reduce GPU PhysX buffer sizes

If you need GPU physics (e.g., for training), add to RobotPlayEnvCfg.__post_init__ in velocity_env_cfg.py:

```python
@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # ... existing code ...
        
        # Reduce GPU buffer sizes for laptop GPUs
        self.sim.physx.gpu_max_rigid_patch_count = 2 * 2**15  # was 10 * 2**15
        self.sim.physx.gpu_max_rigid_contact_count = 2**21     # was 2**23
        self.sim.physx.gpu_collision_stack_size = 2**24        # was 2**26
```

#### Option 3: Increase Linux GPU timeout limit

As a one-time fix for the current session:

```bash
# Check current timeout
cat /sys/module/nvidia_drm/parameters/nvidia_drm_modeset

# Set higher timeout (requires sudo)
echo 10000 | sudo tee /sys/module/nvidia_drm/parameters/nvidia_drm_modeset
```
