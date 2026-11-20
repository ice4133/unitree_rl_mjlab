# Unitree RL Mjlab — Go2W

This project uses [mjlab](https://github.com/mujocolab/mjlab) and MuJoCo to train
and deploy velocity policies for Unitree Go2W. Bodydemo03 robot assets,
velocity training templates and Go2W-style Flat deployment are also available.

[中文说明](README_zh.md) · [Installation](doc/setup_en.md)

See the [Bodydemo03 model guide](src/assets/robots/bodydemo03/README.md) for its
URDF, meshes and converted MuJoCo XML.

## Training

Run commands from the repository root after activating the Python environment
and installing the project with `pip install -e .`.

```bash
python scripts/list_envs.py --keyword Go2W
python scripts/train.py Unitree-Go2W-Flat --env.scene.num-envs=1024
python scripts/train.py Unitree-Go2W-Rough --env.scene.num-envs=512
```

The Go2W tasks are `Unitree-Go2W-Flat` and `Unitree-Go2W-Rough`.
Reduce the environment count if GPU memory is limited. Multi-GPU training uses
`--gpu-ids 0 1`. The installed mjlab package may also register its own tasks;
those are separate from this repository's Go2W configurations.

Training saves checkpoints and ONNX exports under
`logs/rsl_rl/go2w_velocity/<time>/` (Flat) or
`logs/rsl_rl/go2w_velocity_rough/<time>/` (Rough).
See the [Go2W task guide](src/tasks/velocity/config/go2w/README.md) for rewards,
observations, action ordering and export details.

Bodydemo03 adds `Bodydemo03-Flat` and `Bodydemo03-Rough` templates. See the
[Bodydemo03 guide](src/assets/robots/bodydemo03/README.md#velocity-训练与回放模板)
for provisional parameters and replay commands:

```bash
python scripts/train.py Bodydemo03-Flat --env.scene.num-envs=512
```

## Policy replay

```bash
python scripts/play.py Unitree-Go2W-Flat \
  --checkpoint-file=logs/rsl_rl/go2w_velocity/<time>/model_<iteration>.pt \
  --num-envs=1
```

For Rough, use `Unitree-Go2W-Rough` and its matching checkpoint directory.
Use `python scripts/visualize_terrain.py` to inspect terrains with the Go2W model.

## C++ simulation and deployment

The controller uses `deploy/robots/go2w`, shared code in `deploy/include`, and
ONNX Runtime in `deploy/thirdparty`. Go2W uses the Go2 DDS message format from
Unitree SDK2; keep that SDK dependency.

After installing Unitree SDK2 and the dependencies in the installation guide,
build with ROS 2 Humble enabled:

```bash
source /opt/ros/humble/setup.bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
cmake -S deploy/robots/go2w -B deploy/robots/go2w/build
cmake --build deploy/robots/go2w/build -j2
```

In separate terminals, launch the simulator and controller:

```bash
./simulate/build/unitree_mujoco --robot go2w \
  --scene src/assets/robots/unitree_go2w/xmls/scene_go2w.xml
./deploy/robots/go2w/build/go2w_ctrl --network=lo
```

`simulate/config.yaml` currently selects Bodydemo03's deployment scene; the command
above explicitly selects Go2W. Configure manual or automatic velocity input in
`deploy/robots/go2w/config/config.yaml`. Go2W's ROS camera/lidar configuration is
`ros2_sensors/config/sensors.yaml`; set `ros2_sensors_config` to it for navigation.
See the [deployment guide](deploy/robots/go2w/README.md) for state controls,
ROS commands, motor ordering and build options without ROS.

To replace the deployed Flat policy:

```bash
cp logs/rsl_rl/go2w_velocity/<time>/policy.onnx \
  deploy/robots/go2w/config/policy/velocity/v0/exported/policy.onnx
```

The current C++ configuration expects Flat's 55 observations and 16 actions.
Rough uses 242 observations and requires a matching height scan on the deployment
side before it can replace the Flat policy.

For a physical robot, use its Ethernet interface instead of `lo` after validating
the policy in simulation and following the robot's deployment procedure.

Bodydemo03 uses the same deployment structure, DDS slots and FSM. Its own gains,
zero standing pose and torque scene match the current Flat training template:

```bash
cmake -S deploy/robots/bodydemo03 -B deploy/robots/bodydemo03/build
cmake --build deploy/robots/bodydemo03/build -j2
./simulate/build/unitree_mujoco
./deploy/robots/bodydemo03/build/bodydemo03_ctrl --network=lo
```

Run the last two commands in separate terminals. The controller defaults to
keyboard input: `1` to stand, wait 2 seconds, `2` to run the policy, `0` for damping.
See the [Bodydemo03 deployment guide](deploy/robots/bodydemo03/README.md) for ONNX
installation, tests, ROS commands and differences from Go2W.

## Mapping and navigation

See the [autonomy guide](autonomy/README.md) for ROS 2 mapping, Nav2 obstacle
avoidance and startup order. Navigation sends `/cmd_vel` to the controller with
`control.mode: auto`.

## Acknowledgements

- [mjlab](https://github.com/mujocolab/mjlab): training and environment framework.
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl): reinforcement learning.
- [mujoco_warp](https://github.com/google-deepmind/mujoco_warp): GPU simulation.
- [MuJoCo](https://github.com/google-deepmind/mujoco): physics engine.
