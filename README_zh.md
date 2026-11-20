# Unitree RL Mjlab — Go2W

本工程基于 [mjlab](https://github.com/mujocolab/mjlab) 和 MuJoCo，
用于 Unitree Go2W 的速度策略训练、回放和部署。
提供 Go2W 训练与部署配置，以及实验室 Bodydemo03 的模型、velocity 训练模板和 Flat 部署。

[English](README.md) · [安装配置](doc/setup_zh.md)

自研机器人的 URDF 和网格可按 [URDF 转 MJCF](doc/urdf_to_mjcf_zh.md) 转换为 MuJoCo XML。
已导入的 [Bodydemo03 模型](src/assets/robots/bodydemo03/README.md) 包含 URDF、网格及转换后的 XML。

## 🛠️ 使用指南

Go2W 的 ROS 2 建图与自动避障导航见 [autonomy 使用说明](autonomy/README.md)。
导航通过 `/cmd_vel` 连接部署端 `control.mode: auto`，复用已有速度跟踪策略。

### 1. 速度跟踪训练

激活 Python 环境并执行 `pip install -e .` 后，在项目根目录运行：

```bash
python scripts/list_envs.py --keyword Go2W
python scripts/train.py Unitree-Go2W-Flat --env.scene.num-envs=1024
python scripts/train.py Unitree-Go2W-Rough --env.scene.num-envs=512
```

Go2W 的任务为 `Unitree-Go2W-Flat` 和 `Unitree-Go2W-Rough`。
显存不足时降低环境数；多 GPU 训练可添加 `--gpu-ids 0 1`。
安装的 mjlab 库还可能注册自身的任务，它们与本仓库的 Go2W 配置相互独立。

训练结果分别保存在 `logs/rsl_rl/go2w_velocity/<时间>/`（Flat）和
`logs/rsl_rl/go2w_velocity_rough/<时间>/`（Rough），包含 checkpoint 和 ONNX。
奖励、观测、动作顺序和导出约定见
[Go2W 任务说明](src/tasks/velocity/config/go2w/README.md)。

Bodydemo03 提供 `Bodydemo03-Flat`、`Bodydemo03-Rough` 模板，使用方法和
待调整参数见 [Bodydemo03 说明](src/assets/robots/bodydemo03/README.md#velocity-训练与回放模板)：

```bash
python scripts/train.py Bodydemo03-Flat --env.scene.num-envs=512
```

### 2. 策略回放

```bash
python scripts/play.py Unitree-Go2W-Flat \
  --checkpoint-file=logs/rsl_rl/go2w_velocity/<时间>/model_<迭代>.pt \
  --num-envs=1
```

回放 Rough 时，使用 `Unitree-Go2W-Rough` 和对应目录下的 checkpoint。
地形可视化可运行 `python scripts/visualize_terrain.py`，支持显示 Go2W 模型。

### 3. C++ 仿真与部署

控制器位于 `deploy/robots/go2w`，依赖 `deploy/include` 的公共代码和
`deploy/thirdparty` 中的 ONNX Runtime。
Go2W 使用 Unitree SDK2 的 Go2 DDS 消息格式，这部分 SDK 依赖需要保留。

安装 Unitree SDK2 和安装说明中的依赖后，启用 ROS 2 Humble 编译：

```bash
source /opt/ros/humble/setup.bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
cmake -S deploy/robots/go2w -B deploy/robots/go2w/build
cmake --build deploy/robots/go2w/build -j2
```

在两个终端分别启动仿真器和控制器：

```bash
./simulate/build/unitree_mujoco --robot go2w \
  --scene src/assets/robots/unitree_go2w/xmls/scene_go2w.xml
./deploy/robots/go2w/build/go2w_ctrl --network=lo
```

`simulate/config.yaml` 当前默认选择 Bodydemo03 部署场景，上面的命令显式加载 Go2W。
Go2W 建图／导航还需将 `ros2_sensors_config` 设为 `ros2_sensors/config/sensors.yaml`。
手动／自动速度来源在 `deploy/robots/go2w/config/config.yaml` 中设置。
状态切换、ROS 指令、电机顺序和不启用 ROS 的构建方式见
[部署说明](deploy/robots/go2w/README.md)。

替换部署端的 Flat 策略：

```bash
cp logs/rsl_rl/go2w_velocity/<时间>/policy.onnx \
  deploy/robots/go2w/config/policy/velocity/v0/exported/policy.onnx
```

当前 C++ 部署配置对应 Flat 的 55 维观测和 16 维动作。
Rough 使用 242 维观测，部署端提供相同定义的高度扫描后才能替换。
实机部署前先完成仿真验证，并按机器人部署流程将 `lo` 改为实机网卡。

Bodydemo03 沿用 Go2W 的部署结构、DDS 槽位和状态机，使用自身的零位站姿、
PD 参数及力矩场景：

```bash
cmake -S deploy/robots/bodydemo03 -B deploy/robots/bodydemo03/build
cmake --build deploy/robots/bodydemo03/build -j2
./simulate/build/unitree_mujoco
./deploy/robots/bodydemo03/build/bodydemo03_ctrl --network=lo
```

最后两条命令在不同终端运行。默认键盘控制，`1` 站立，等待 2 秒后按 `2`
进入策略，`0` 阻尼。权重安装、测试、ROS 输入及与 Go2W 的差异见
[Bodydemo03 部署说明](deploy/robots/bodydemo03/README.md)。

### 4. 建图与导航

ROS 2 建图、Nav2 避障和完整启动顺序见 [autonomy 使用说明](autonomy/README.md)。
导航通过 `/cmd_vel` 向 `control.mode: auto` 的控制器发送速度指令。

## 致谢

- [mjlab](https://github.com/mujocolab/mjlab)：训练和环境框架。
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl)：强化学习算法。
- [mujoco_warp](https://github.com/google-deepmind/mujoco_warp)：GPU 仿真。
- [MuJoCo](https://github.com/google-deepmind/mujoco)：物理引擎。
