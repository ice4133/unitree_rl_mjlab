# Autonomy

ROS 2 Humble 工作空间，实现雷达预处理、二维扫描、SLAM 建图与 Go2W 自动避障导航。
导航使用 Nav2，底层继续使用现有速度跟踪 ONNX 策略，不需要另外训练导航网络。

## 职责和目录

| 包 | 职责 | 当前状态 |
| --- | --- | --- |
| `autonomy_perception` | 点云过滤、水平坐标系、scan 转换；后续深度图处理 | 雷达链路已实现 |
| `autonomy_localization` | SLAM、地图保存；后续已知地图定位 | 官方 SLAM Toolbox 已接入 |
| `autonomy_navigation` | Nav2 代价地图、规划、路径跟踪、碰撞监控、速度守护 | 已实现 |
| `autonomy_bringup` | 组合启动、domain、仿真时间、RViz、建图/导航检查 | 已实现 |
| `third_party/pointcloud_to_laserscan` | 官方点云到二维扫描转换 | 固定 Humble 源码版本 |

`simulate` 负责传感器、真值里程计和原有 TF；`deploy` 负责策略和电机。
建图不要求 Nav2，也不发送运动指令。相机原始话题仍由 simulate 发布。

```text
/lidar/points (10 Hz)
  -> lidar_filter_node
  -> /perception/lidar/points_filtered (三维点云，保留其他字段)
  -> 官方 pointcloud_to_laserscan
  -> /scan (约 10 Hz)
  -> 官方 slam_toolbox
  -> /map (二维 OccupancyGrid，默认 1 秒刷新) + map -> odom TF
```

时间全部来自现有 `/clock`。过滤和投影保留输入采样时间；TF 按点云时刻查询。
`/odom` 继续是仿真真值；SLAM Toolbox 通过 TF 获取里程计位姿。
扫描频率由点云输入决定，`scan_time` 仅描述扫描周期，不控制发布频率。

## 构建

```bash
./autonomy/run.sh build
```

脚本自动加载 `/opt/ros/humble/setup.bash`，构建后运行也会加载本工作空间。
无需手动 export domain 或中间件。运行节点默认 domain 10、Fast DDS、
`use_sim_time:=true`；SDK2 电机通信仍为 domain 0。

依赖：ROS 2 Humble 开发环境、colcon、slam_toolbox、navigation2、nav2_bringup、
nav2_collision_monitor、rviz2、tf2、message_filters、ament_cmake_gtest、ament_cmake_pytest。
当前机器已有这些依赖。
点云转换包以源码随工作空间构建，来源、版本和许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
其他机器可先用 `rosdep install --from-paths autonomy/src --ignore-src -r -y` 安装缺失依赖。

## 运行建图

在仓库根目录，分别打开终端：

```bash
# 1. 仿真。专用房间复用原机器人、传感器和前方箱子，额外增加墙体和柱子。
./simulate/build/unitree_mujoco --scene=src/assets/robots/unitree_go2w/xmls/scene_go2w_mapping.xml

# 2. 建图和 RViz。
./autonomy/run.sh mapping

# 3. 部署控制器。
./deploy/robots/go2w/build/go2w_ctrl --network=lo
```

为了手动带机器人建图，在 `deploy/robots/go2w/config/config.yaml` 中选择
`control.mode: keyboard` 或 `joystick`，重启控制器后先站立、再进入策略。
键盘档用 `1` 站立，完成后用 `2` 进入策略，W/S 前后、Q/E 转弯，`0` 阻尼。
手柄档沿用现有组合键，且需要 `simulate/config.yaml` 开启手柄输入并连接设备。
建图模式不会发送速度或避障。当前部署配置已设为 `auto`，手动建图时请先切换到手动档。

无图形界面时可以只关闭 RViz（MuJoCo 自身仍需要当前 GUI/显示环境）：

```bash
./autonomy/run.sh mapping use_rviz:=false
```

只调试感知层：`./autonomy/run.sh perception`。
官方转换器按需订阅，只有 RViz、SLAM 或其他订阅者订阅 `/scan` 时才开始转换。
直接启动子包的 launch 文件时，domain/环境由调用者负责；通常使用统一入口即可。

本机 RViz 在首次加载栅格地图时会打印一次 `active samplers ... same texture image unit`
的 GLSL 提示；已实际确认地图、扫描和三维点云仍能正常显示，地图保存也正常。
该提示来自已安装的 RViz/OGRE 渲染组件，不代表 SLAM 数据链路失败。

## 检查和保存

```bash
# 只读检查，默认观察 10 秒；检查频率、扫描时间戳、地图内容和完整 TF。
./autonomy/run.sh check

# 保存 Nav2 可用的二维栅格地图，输出 YAML + PGM。
./autonomy/run.sh save-map "$PWD/autonomy/maps/room"

# 可选：另外保存 SLAM 位姿图，以便后续继续建图。
ROS_DOMAIN_ID=10 ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$PWD/autonomy/maps/room'}"
```

`maps/` 中生成的地图默认不进入 Git。保存的栅格地图不是完整的三维环境，
后续 Nav2 仍需要实时点云处理新出现的障碍物。

## 自动绕过前方障碍

分别在三个终端运行（建图模式和导航模式二选一，不要重复启动）：

```bash
# 1. 启动包含前方箱子和墙体的仿真。
./simulate/build/unitree_mujoco --scene=src/assets/robots/unitree_go2w/xmls/scene_go2w_mapping.xml

# 2. 启动部署控制器。control.mode 已设为 auto。
./deploy/robots/go2w/build/go2w_ctrl --network=lo
# 在控制器终端按 1 站立，等至少 2 秒，再按 2 进入 Velocity。

# 3. 在线 SLAM + Nav2 + RViz；建议站稳后启动，避免趴卧时扫描污染初始地图。
./autonomy/run.sh navigation
```

等待节点激活后，在 RViz 工具栏用 **Nav2 Goal** 设置目标，或在第四个终端运行：

```bash
# 默认箱子中心 (2, 0)，半长 0.2、半宽 0.4 m；目标设在箱子后方。
./autonomy/run.sh goal --x 4 --y 0 --yaw 0 --timeout 120

# 只读检查；有目标执行时可要求观测到路径和非零速度。
./autonomy/run.sh check-navigation --require-motion
./autonomy/run.sh check
```

目标坐标属于 `map`，上述例子适用于从初始位置启动的专用场景；不是持续向前巡航。
机器人会规划绕行路径，转向、绕过箱子，在目标附近停下。没有目标时保持零速度。
CLI 到点返回 0，拒绝、失败或超时返回非零；Ctrl-C / 超时会请求取消当前目标。
控制器终端随时可以按 `0` 进入阻尼。auto 只接管速度，不自动触发站立和策略状态切换。
不要同时使用手动 `ros2 topic pub /cmd_vel`，以免绕过守护并与导航输出争用。

```text
/scan + /map + TF + /odom
  -> Navfn A* 全局规划（1 Hz 重规划，实时障碍层）
  -> Regulated Pure Pursuit（跟踪绕行路径、转弯/近障减速、局部碰撞检查）
  -> /navigation/cmd_vel_raw
  -> velocity_smoother -> /navigation/cmd_vel_smoothed
  -> collision_monitor -> /navigation/cmd_vel_checked
  -> velocity_guard -> /cmd_vel -> deploy auto -> ONNX -> rt/lowcmd
```

默认前进不超过 0.35 m/s、转向不超过 0.6 rad/s，横移固定为 0。
碰撞监控检查机器人四周的停止矩形；最终守护以单调时钟检查激光和速度新鲜度，
并检查扫描时间戳与 TF。激光超过 0.5 秒、速度超过 0.3 秒或 TF 不可用时输出零速度。
守护进程停止后，部署端原有 0.5 秒超时仍会归零。
Humble 的碰撞监控会忽略过期数据源，所以不能单靠其 `source_timeout` 保证失联停车，
这里用独立守护补上，并由集成测试验证。

当前是平地仿真方案，定位使用 SLAM 和仿真真值里程计；已知地图 AMCL、复杂地形、
视觉障碍和实机定位仍未接入。在线规划允许通过 SLAM 未知区域（例如箱子后的阴影），
实时扫描层与局部碰撞检查继续有效；若无可行路径则停车/恢复或报告失败。

## 坐标系和适用范围

```text
map                          SLAM Toolbox 发布 map -> odom
└── odom
    ├── base_link            simulate 发布，保留完整 3D 位姿
    │   ├── mid360_frame     simulate 发布静态安装关系
    │   └── front_camera_optical_frame
    ├── base_footprint       感知层发布机体 XY/yaw 的地面投影
    └── lidar_scan_frame     感知层发布雷达原点位置和水平 yaw
```

两个新增帧均为独立的 odom 子帧，不重新发布或修改 base_link 的父帧。
SLAM 使用 `base_footprint`，scan 使用 `lidar_scan_frame`，机身 roll/pitch
不会直接旋转二维扫描平面。三维点云仍保留高度。

当前高度筛选基于平地假设：地面为 odom 的 z=0，默认保留离地 0.08–2.0 m 的点。
这不是复杂地形分割；低于 8 cm 的障碍、坡地、台阶、相机融合和实机漂移需要后续处理。
实机接入时需要调整 ground_height、机器人包围盒及里程计来源。
重置仿真时间或更换场景后，应重启建图会话，避免旧地图与新 odom 混用。

## 参数和后续扩展

- `autonomy_perception/config/lidar.yaml`：高度、距离、自身包围盒、帧和点云话题。
- `autonomy_perception/config/scan.yaml`：二维扫描角度、分辨率、范围。
- `autonomy_localization/config/slam.yaml`：匹配、回环、地图分辨率与更新策略。
- 顶层 launch 参数：`ros_domain_id`、`use_sim_time`、`use_rviz`，以及 `lidar_params`、`scan_params`、`slam_params`、`nav_params`。
- 相机深度处理加入 perception；AMCL/map_server 加入 localization；Nav2 参数在 navigation。
  对外沿用标准 ROS 消息，不在统一启动包中堆算法。

导航的控制器、footprint、速度限制和超时配置见 [导航包说明](src/autonomy_navigation/README.md)。

## 测试

```bash
source autonomy/install/setup.bash
cd autonomy
colcon test --packages-select autonomy_perception autonomy_navigation --event-handlers console_direct+
colcon test-result --verbose
```

感知单元测试覆盖过滤、姿态补偿、时间戳、额外字段、组织化点云行填充、
大端字段和无效配置。完整运行验证使用上面的 `run.sh check`。

导航测试使用独立 domain 197，启动实际 Humble 碰撞监控和速度守护，验证通行、近障停车、
激光中断停车、速度中断停车及恢复，不向 domain 10 的机器人发指令。
完整 MuJoCo 绕障轨迹和测量结果见 [自动导航验证记录](docs/navigation_validation.md)。
