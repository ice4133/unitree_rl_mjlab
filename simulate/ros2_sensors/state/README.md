# Floating-base ground-truth odometry

位姿与速度提取方式移植自 `mujoco_ros2_control/src/mujoco_system_interface.cpp`
的 floating-base odometry 发布逻辑，来源与许可证见上级 `NOTICE` / `LICENSE`。
发布器直接编入 `unitree_mujoco`，使用 SensorManager 的 ROS context，固定在
domain 10，通过 Fast DDS 发布；不需要额外启动状态节点。

## 输出

| 话题 | 类型 | 默认频率 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 每秒仿真时间 50 帧 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 同步发布 `odom -> base_link` |

默认 `header.frame_id=odom`，`child_frame_id` 从选定 free joint 所属 body 的
MJCF 名称得到（Go2W 是 `base_link`）。里程计和 TF 使用完全相同的位姿与
仿真时间戳，与相机、雷达和 `/clock` 共用时间基准。

位置和完整三维姿态直接读取 free joint 的 `qpos`，参考系是 MuJoCo 世界
坐标系，在 ROS 中命名为 `odom`，没有将初始机身位置平移为零。
包含机身高度、横滚和俯仰，不把 IMU 的安装点或质心误当成机身原点。
MuJoCo 四元数为 w/x/y/z，发布时对应 ROS 的同名字段。

按照 ROS Odometry 约定，`pose` 在 `header.frame_id` 中表达，`twist` 在
`child_frame_id` 中表达。MuJoCo free joint 的平移速度是世界轴方向，发布前
通过当前姿态旋转到机身轴；角速度已在机身轴中，直接使用。单位为米、秒、
米/秒和弧度/秒。数据由积分后的 `qpos/qvel` 获取，避免 `mj_step` 后派生位姿
可能落后一个积分步的问题。

这是仿真真值里程计，没有积分轮速、漂移或噪声模型；协方差为零。
实机状态估计器需要提供自身估计及协方差。本模块只发布 `odom -> base_link`，
后续地图定位/SLAM 的 `map -> odom` 应由其定位模块提供。

## 配置

`../config/sensors.yaml`：

```yaml
odometry:
  enabled: true
  free_joint: floating_base_joint
  topic: /odom
  frame_id: odom
  publish_rate: 50.0
  publish_tf: true
```

配置在启动时读取。没有 `odometry` 配置或 `enabled: false` 时不启动该发布器；
找不到选定关节时提示并跳过，选择非 free joint、无效频率或父子 frame 同名时
报错。默认 `/odom` 使用 Reliable、Volatile、depth 10，以便常规里程计订阅者
直接接入。`publish_tf: false` 可以在保留里程计的同时关闭动态 TF。

仿真线程仅复制小型位姿/速度消息，独立工作线程执行 ROS 发布，不读取共享
`mjData`，待处理消息仅保留最新一帧。暂停后不再采样，已提交的一帧可以完成
发布。重置或时间回退后立即重新采样，丢弃尚未发布的旧代状态；模型重载与
退出时停止线程，再释放模型。

## 验证

```bash
cmake --build simulate/build -j2
ctest --test-dir simulate/build --output-on-failure
./simulate/build/unitree_mujoco
```

另开 ROS 2 Humble 终端，使用 domain 10：

```bash
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic info /odom --verbose
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic echo /odom --once
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic hz /odom
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run tf2_ros tf2_echo odom base_link
```

RViz Fixed Frame 可设为 `odom`，再添加 Odometry、TF 和 PointCloud2。查看工具
需要按需要启用 `use_sim_time`。实际墙钟频率受仿真速度和负载影响，慢仿真或
高倍加速下不保证墙钟 50 Hz。
