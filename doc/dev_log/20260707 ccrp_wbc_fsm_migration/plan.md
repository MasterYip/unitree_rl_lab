# Plan: Integrate AMP Locomotion Models into unitree_rl_lab

## Context

The wbc_fsm project (at `/home/user/CodeSpace/Sim2Real/wbc_fsm`, symlinked to `./tmp`) contains AMP locomotion models for the Unitree G1 humanoid robot. These models need to be integrated into the unitree_rl_lab deploy system so they can be used on the real robot alongside the existing Velocity policy.

Three ONNX models are available in `tmp/wbc_fsm/model/loco/`:
- `loco_0731.onnx` — LSTM-based velocity-tracking locomotion (3 inputs: obs/h_in/c_in, 3 outputs: actions/h_out/c_out)
- `amp_0309_1.onnx` — Feed-forward AMP locomotion with 4-frame observation history (single input/output)
- `Unitree-G1-AMP-Flat_model_30000.onnx` — Feed-forward AMP for Mujoco with sequential joint mapping (simulation-only, lower priority)

## Approach

### AMP feed-forward model (`amp_0309_1.onnx`)
Works with the **existing** `State_RLBase` + `OrtRunner` infrastructure. Just needs a correctly configured `deploy.yaml` with `use_gym_history: true` and `history_length: 4` for observation stacking.

### Loco LSTM model (`loco_0731.onnx`)
Requires new `OrtRunnerLSTM` class that manages hidden states (h_in/c_in → h_out/c_out) across timesteps. The `State_RLBase` constructor is modified to auto-detect LSTM models and create the appropriate runner.

## Files to Modify

### 1. `deploy/include/isaaclab/algorithms/algorithms.h`
- Add `virtual void reset() {}` to the `Algorithms` base class
- Add new `OrtRunnerLSTM` class:
  - Takes all input names and all output names (not just first)
  - Stores `h_state_` and `c_state_` vectors internally
  - `reset()` zeroes hidden states
  - `act()` adds h_in/c_in to input tensors before inference, extracts h_out/c_out after, updates stored states

### 2. `deploy/include/isaaclab/envs/manager_based_rl_env.h`
- Add `alg->reset()` call in `ManagerBasedRLEnv::reset()` method (after observation_manager->reset())

### 3. `deploy/robots/g1_29dof/src/State_RLBase.cpp`
- After loading the ONNX model, check if input names contain "h_in"
- If yes: create `OrtRunnerLSTM` instead of `OrtRunner`
- If no: use existing `OrtRunner` (preserves backward compatibility)

### 4. `deploy/robots/g1_29dof/config/policy/amp/v0/` (NEW)
- `exported/policy.onnx` — copy of `tmp/wbc_fsm/model/loco/amp_0309_1.onnx`
- `params/deploy.yaml` — observation/action config using `use_gym_history: true, history_length: 4`

### 5. `deploy/robots/g1_29dof/config/policy/loco/v0/` (NEW)
- `exported/policy.onnx` — copy of `tmp/wbc_fsm/model/loco/loco_0731.onnx`
- `params/deploy.yaml` — observation/action config with LSTM-compatible setup (no history stacking, history_length: 1)

### 6. `deploy/robots/g1_29dof/config/config.yaml`
- Add `Loco_AMP` state (id: 4, type: RLBase) — uses amp_0309_1.onnx, walk/run with speed modes
- Add `Loco_LSTM` state (id: 5, type: RLBase) — uses loco_0731.onnx, velocity-tracking
- Add transitions between new states and existing states

## Key params derived from wbc_fsm source

### AMP model params (from `State_Amp.h/cpp` and `amp.json`)
- **Observation**: 384 dims = 4 frames × 96 (ang_vel[3] + projected_gravity[3] + commands[3] + dof_pos_rel[29] + dof_vel_rel[29] + last_action[29])
- **Action**: 29 dims, JointPositionAction, scale=0.25, offset=default_dof_pos
- **default_dof_pos** (joint_ids_map order): `[-0.312, -0.312, 0, 0, 0, 0, 0, 0, 0, 0.669, 0.669, 0.2, 0.2, -0.363, -0.363, 0.2, -0.2, 0, 0, 0, 0, 0.6, 0.6, 0, 0, 0, 0, 0, 0]`
- **Command ranges**: lin_vel_x=[-1.5, 3.0], lin_vel_y=0, ang_vel_z=[-1.57, 1.57]

### Loco model params (from `State_Loco.h/cpp` and `loco.json`)
- **Observation**: 96 dims (ang_vel[3] + projected_gravity[3] + commands[3] + dof_pos_rel[29] + dof_vel_rel[29] + last_action[29])
- **Action**: 29 dims, JointPositionAction, scale=0.25, offset=default_dof_pos
- **default_dof_pos** (joint_ids_map order): `[-0.2, -0.2, 0, 0, 0, 0, 0, 0, 0, 0.42, 0.42, 0.35, 0.35, -0.23, -0.23, 0.18, -0.18, 0, 0, 0, 0, 0.87, 0.87, 0, 0, 0, 0, 0, 0]`
- **Command ranges**: lin_vel_x=[-0.6, 0.85], lin_vel_y=[-0.4, 0.4], ang_vel_z=[-1.0, 1.0]

## Verification

1. Verify ONNX models were copied correctly by checking file hashes match
2. Verify deploy.yaml files are valid YAML
3. Check that config.yaml changes reference the correct policy directories and state IDs
4. Verify the State_RLBase change compiles (build the g1_29dof deploy target)
