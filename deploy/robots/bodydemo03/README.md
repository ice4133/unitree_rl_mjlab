# Bodydemo03 部署

本目录按 `deploy/robots/go2w` 接入 `Bodydemo03-Flat`。入口、公共 FSM、
ONNX 推理、键盘／手柄输入、ROS 2 `/cmd_vel` 和电机槽位约定沿用 Go2W。
仿真通信同样使用 Unitree SDK2 的 Go2 DDS 消息，前 16 个槽位有效。

## 编译与启动

在项目根目录运行：

```bash
source /opt/ros/humble/setup.bash
cmake -S deploy/robots/bodydemo03 -B deploy/robots/bodydemo03/build
cmake --build deploy/robots/bodydemo03/build -j2
ctest --test-dir deploy/robots/bodydemo03/build --output-on-failure

cmake --build simulate/build -j2
```

`simulate/config.yaml` 默认选择 `scene_bodydemo03_deploy.xml`。在两个终端启动：

```bash
./simulate/build/unitree_mujoco
./deploy/robots/bodydemo03/build/bodydemo03_ctrl --network=lo
```

首次配置 `simulate/build` 时使用 `cmake -S simulate -B simulate/build`。
部署控制器需收到仿真器的 `rt/lowstate` 后才会继续初始化。
Go2W 和 Bodydemo03 控制器共用 domain 0 的 `rt/lowcmd`，启动前请先在旧控制器
终端按 `Ctrl+C`，确保只运行一个控制器。Bodydemo03 启动时若检测到已有命令发布，
会报错并退出；它不会自动结束本机的 `go2w_ctrl` 或其他控制器进程。
默认 `control.mode: keyboard`，键盘 `1` 进入 `FixStand`，等待 2 秒完成站立，
再按 `2` 进入 `Velocity`；`0` 返回阻尼。W/S 前后，Q/E 转向，横向速度固定为零。

`config/config.yaml` 也支持 Go2W 相同的 `joystick` 和 `auto` 模式。
手柄需同时在 `simulate/config.yaml` 中开启 `use_joystick` 并选择正确型号。
L2 + 十字键上进入站立，R2 + A 进入策略，L2 + B 返回阻尼。
自动档订阅 domain 10 的 `/cmd_vel`，保留手柄／终端状态按键；
Unitree 电机通信使用 domain 0，ROS 指令超时 0.5 秒归零。

```bash
ROS_DOMAIN_ID=10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

更改控制模式后重启控制器。不需要 ROS 输入时，可以用
`-DENABLE_ROS2_COMMANDS=OFF` 编译，继续使用键盘或手柄。
仿真器的 `ros2_sensors/config/bodydemo03.yaml` 默认关闭相机／雷达等 ROS 传感器；
策略所需 IMU 由 XML 传感器经 `rt/lowstate` 提供，不受该开关影响。

## 使用训练结果

从正在训练的 Flat 日志目录复制完整导出的 ONNX：

```bash
cp logs/rsl_rl/bodydemo03_velocity/<时间>/policy.onnx \
  deploy/robots/bodydemo03/config/policy/velocity/v0/exported/policy.onnx
```

重启控制器加载新权重。ONNX 本身已包含 Actor 的运行均值归一化。
首次接入已复制 `2026-11-20_14-55-29` 目录当时的导出，属于训练初期权重；
训练完成后按上面的命令更新。权重按本机训练产物管理，不提交 Git。
新 checkout 中需要先复制对应策略，再启动完整 FSM。

本配置接收 `obs: [1, 55]`，输出 `actions: [1, 16]`。
Rough 的 242 维输入需要匹配的地形扫描，此处暂时只接入当前训练的 Flat。
公共 ONNX runner 会在输入维度不匹配时明确报错。

## 与 Go2W 保持一致的部分

- `Passive → FixStand → Velocity` 状态机、50 Hz 策略及 1 kHz FSM。
- 55 维观测顺序：角速度 3、重力投影 3、速度命令 3、相位 2、
  腿位置 12、全部关节速度 16、上次原始动作 16。
- 动作前 12 维控制 FL/FR/RL/RR 的 hip/thigh/calf 位置增量，
  hip 尺度 0.125，thigh/calf 尺度 0.25；后 4 维轮速尺度 5 rad/s。
- XML 关节顺序为 FL/FR/RL/RR，每腿 hip/thigh/calf/foot；
  SDK 槽位为 FR 腿 0–2、FL 3–5、RR 6–8、RL 9–11，
  FR/FL/RR/RL 轮子 12/13/14/15，使用相同的 `joint_ids_map`。
- 腿部命令写 `q`，轮子命令写 `dq` 且 `kp=0`；
  仿真桥接计算 `tau + kp*(q_target-q) + kd*(dq_target-dq)`，XML 电机接收力矩。

## Bodydemo03 专属差异

| 项目 | Bodydemo03 配置 |
| --- | --- |
| 关节名称 | 小写 `fl/fr/rl/rr`，轮轴为 `*_foot_joint` |
| 默认角度与站立 | 全零，2 秒从当前命令姿态过渡到 CAD 零位站姿 |
| 腿 kp / kd / 力矩上限 | 400 / 12 / 50 Nm |
| 轮 kp / kd / 力矩上限 | 0 / 2 / 8 Nm |
| 腿位置范围 | hip ±0.5、thigh ±1.0、calf ±1.5 rad |
| 关节方向 | 保留训练的 URDF 坐标方向，左右轮前进时角速度符号相反 |
| 仿真场景 | `scene_bodydemo03_deploy.xml`，0.002 秒物理步长 |

独立部署场景按 Go2W 使用纯力矩电机，保留原始模型的质量、惯量及关节轴，
同步训练的关节限位、armature、轮地接触和自碰撞设置，并补齐桥接所需 IMU。
原来的 `scene_bodydemo03.xml` 继续用作内置位置／速度伺服的模型预览。
训练继续使用原来的 Python 配置。上述 PD 和力矩限制是当前训练模板参数。
实际 Bodydemo03 硬件通信协议尚未接入；此处 DDS 约定用于 C++ 仿真。

在项目 Python 环境中可额外验证场景映射、模型参数、力矩限幅及零位显式 PD：

```bash
conda activate unitree_rl_mjlab
python deploy/robots/bodydemo03/tests/test_scene.py -v
```
