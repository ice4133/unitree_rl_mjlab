# Go2W 部署

本目录复用 Go2 的 DDS 消息格式，控制 12 个腿部电机和 4 个轮电机。
`main.cpp` 保留现有入口，阻尼、站立和策略状态统一复用 `deploy/include/FSM`。
机器人专属代码为 `src/State_RLBase.cpp`，动作映射的声明和实现分别位于
`include/Go2WPolicy.h` 和 `src/Go2WPolicy.cpp`。ONNX 推理复用公共 `OrtRunner`。

```bash
cmake -S deploy/robots/go2w -B deploy/robots/go2w/build
cmake --build deploy/robots/go2w/build -j2
ctest --test-dir deploy/robots/go2w/build --output-on-failure
./deploy/robots/go2w/build/go2w_ctrl --network=lo
```

Linux 构建会将 Unitree SDK2 的动态库路径写入可执行文件的 RPATH。
即使当前终端已加载 ROS 环境，也无需手动设置 `LD_LIBRARY_PATH`。

控制器需收到 `rt/lowstate` 后才会继续启动。仿真器使用根目录的
`simulate/config.yaml`，目前已选择 `scene_go2w.xml`。

键盘 `1` 进入站立，`2` 进入策略，`0` 进入阻尼；W/S 控制前后，Q/E 控制转向。
当前平地滚动任务的横向速度固定为 0，A/D 不产生横移命令。
在 `config/config.yaml` 中设置 `control.mode: keyboard` 或 `control.mode: joystick`，
状态切换按入口提示操作。`control.mode` 是唯一的档位选择：手动档同时选择
速度和状态输入；自动档通过 ROS 接收速度，同时保留手柄状态按钮和终端状态按键。
没有 `control` 节点时默认为手柄。
仿真手柄还需在 `simulate/config.yaml` 设置 `use_joystick: 1` 和匹配的
`joystick_type`。按住 L2（LT）并按十字方向键上进入站立，状态切换组合键
支持任一按下顺序；修改这些配置后需重启对应程序。

## 自动速度输入

`config/config.yaml` 的 `control.mode: auto` 使用 ROS 2 `/cmd_vel`
（`geometry_msgs/msg/Twist`）。`linear.x`、`linear.y` 是机体坐标系前进、横移
速度（m/s），`angular.z` 是绕机体 Z 轴的转向速度（rad/s）。接收节点默认
使用独立的 domain 10，Unitree 电机通信仍使用 domain 0；无需修改 shell 的
`ROS_DOMAIN_ID` 来启动控制器。默认 Fast DDS，启动时自动选择
`rmw_fastrtps_cpp`，与仿真传感器的中间件约定一致。

自动档只切换速度来源，仍需先进入 `FixStand`，完成站立后再进入 `Velocity`。
状态按钮继续使用手柄；在交互终端运行时，也可以直接用 `1`、`2`、`0`
控制站立、策略和阻尼，无需额外配置。终端状态按键按下期间优先于手柄按钮，
松开后恢复读取手柄。不带终端运行时不创建键盘输入线程。
自动档中键盘移动键和手柄摇杆不提供速度，状态切换和姿态保护继续生效。

```bash
ROS_DOMAIN_ID=10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

尚未收到指令、收到非有限值或超过 `control.ros.timeout`（默认 0.5 秒）未收到新指令时，
目标速度归零，保持策略运行。超时按单调时钟计算，不依赖 `/clock`。
所有来源统一沿用策略 `params/deploy.yaml` 的速度范围限幅；当前策略横移范围为 0。
未来 Nav2 可以直接发布到同一个话题。

公共 `ManagerBasedRLEnv` 的第三个可选参数是 `isaaclab::VelocityCommandSource`，
返回 `{vx, vy, wz}`。默认仍读取手柄（键盘沿用已有的手柄模拟路径），
每个策略周期读取一次，速度观测和步态相位共用该快照。公共头文件不依赖 ROS；
订阅器在 Go2W 内部实现，并由环境中的回调持有。其他机器人无需修改。
只需要手动控制且不希望链接 ROS 时，可以使用 `-DENABLE_ROS2_COMMANDS=OFF` 构建。

手柄直接提供摇杆值，键盘沿用模拟摇杆的路径；只有自动档通过 ROS 订阅 `/cmd_vel`。
三种来源最后都转换成 `{vx, vy, wz}`，统一限幅后进入策略的速度观测。
策略接收数值观测，不直接订阅 ROS。电机指令仍通过 Unitree SDK2 的 `rt/lowcmd` 发布。

## 电机和动作顺序

| 含义 | 顺序 |
| --- | --- |
| XML 的 16 个关节 | FL、FR、RL、RR，每条腿依次 hip/thigh/calf/wheel |
| SDK 腿电机 | FR 0–2、FL 3–5、RR 6–8、RL 9–11 |
| SDK 轮电机 | FR 12、FL 13、RR 14、RL 15 |
| 策略的前 12 个动作 | FL、FR、RL、RR 的 hip/thigh/calf 位置增量 |
| 策略的后 4 个动作 | FL、FR、RL、RR 的轮角速度 |

`params/deploy.yaml` 中，`joint_ids_map`、`default_joint_pos` 使用 XML 顺序；
`stiffness`、`damping` 使用 SDK 顺序。每个动作项的 `joint_ids` 索引 XML
关节列表，程序按动作项顺序拆分策略输出，再映射到 SDK 电机。
腿部写 `q`，轮子写 `dq`，轮子的 `kp` 始终为 0。
轮速单位为 rad/s，线速度换算可使用 XML 中的轮半径 0.086 m。

## 接入训练后的策略

当前仓库没有训练好的 Go2W 策略。本目录没有复制 Go2 的权重。
把 Go2W 的 ONNX 放到：

```
config/policy/velocity/v0/exported/policy.onnx
```

公共 `State_RLBase` 在控制器初始化时加载策略，完整配置需要先放入权重。
单独测试阻尼和站立时，可暂时从 `config/config.yaml` 的 `FSM._` 中移除
`Velocity` 项，并移除 `FixStand.transitions` 中进入 `Velocity` 的配置。
部署参数与 `src/assets/robots/unitree_go2w/go2w_constants.py` 的姿态和增益，
以及 `src/tasks/velocity/config/go2w/env_cfgs.py` 的动作和观测一致。
当前对应任务为 `Unitree-Go2W-Flat`；修改训练配置后，需同步核对以下参数：

- `step_dt`、两个动作项的顺序、关节选择、scale 和 offset。
- 观测顺序、归一化、scale、history_length、速度命令范围和步态周期。
- 模板观测为角速度 3、重力投影 3、速度命令 3、步态相位 2、腿关节位置 12、
  所有关节速度 16、上次动作 16，共 55 维，ONNX 输入名为 `obs`。
- 模板输出为 16 维；输入和输出维度需与部署配置一致。
- 髋关节位置动作尺度为 0.125，其余腿关节为 0.25；轮速动作尺度为 5.0。
- 角速度观测乘 0.25，全部关节速度观测乘 0.05。策略训练时的运行均值
  归一化包含在导出的 ONNX 中，部署端不再额外归一化。

训练若采用不同的观测或动作约定，应相应修改此处的 `params/deploy.yaml`。
该模板用于仿真接入，完整策略行为需在训练完成后进行仿真验证。
