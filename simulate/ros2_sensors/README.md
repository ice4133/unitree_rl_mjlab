# MuJoCo ROS 2 sensors

本模块直接编入 `unitree_mujoco` 进程，读取当前仿真状态并发布相机和 3D 雷达数据。
相机和雷达实现移植自 `mujoco_ros2_control`，来源与改动见
[NOTICE](NOTICE)，原许可证见 [LICENSE](LICENSE)。不需要另外运行该项目。

## 目录与职责

- `camera/`：MJCF 相机参数、RGB/深度渲染、CameraInfo、ROS 图像发布。
- `common/`：ROS 节点、仿真时间、TF、传感器生命周期、EGL 上下文。
- `lidar/`：批量射线检测、XYZ 点云发布、独立工作线程；详见 [雷达说明](lidar/README.md)。
- `config/sensors.yaml`：开关、ROS 域、相机与雷达参数和话题。

相机位置、朝向、视场角和目标分辨率读取 MJCF；默认使用 Go2W 的
`front_camera`（640×480）。渲染使用独立 EGL 上下文，不随 GUI 观察视角改变。
采样在仿真线程进行，双缓冲快照交给独立渲染线程；慢帧覆盖待处理旧帧，
不积累图像队列。重置时重新计时，卸载模型时先停止相机再释放模型。
EGL display 在传感器管理器生命周期中复用；全部渲染线程结束后才最终关闭，
避免驱动在模型重载时反复卸载、重新初始化。
退出时先停止物理、宇树桥接和相机线程，再释放图形上下文及 MuJoCo 模型。

## 两个通信域

| 通道 | 实现 | Domain ID |
| --- | --- | --- |
| 宇树 `rt/lowcmd`、`rt/lowstate` | 原有 SDK2 / Cyclone DDS | **0** |
| ROS 相机、雷达、时钟、TF | `rclcpp` / Fast DDS | **10** |

ROS Context 通过 API 显式设置域 10，即使启动终端里 `ROS_DOMAIN_ID=0`，
相机也不会发布到域 0。宇树域仍由 `simulate/config.yaml` 的 `domain_id: 0`
指定；启用传感器时会检查两个域分别为 0 和 10。
ROS 侧使用 `rmw_fastrtps_cpp`，避免让 ROS RMW 再引入另一套 Cyclone DDS。
SDK2 使用其安装目录中的配套 `libddsc` / `libddscxx`。Linux 构建为仿真器
设置 ELF RPATH 优先级，避免 ROS 环境中的同名 Cyclone C 库覆盖 SDK 版本；
可用 `ldd simulate/build/unitree_mujoco` 查看当前加载路径。

## 编译与启动

从项目根目录执行，使用系统 ROS 2 Humble 环境：

```bash
source /opt/ros/humble/setup.bash
cmake -S simulate -B simulate/build \
  -DENABLE_ROS2_SENSORS=ON -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build simulate/build -j2
./simulate/build/unitree_mujoco
```

编译完成后，日常启动直接运行上面的可执行文件即可；也可以进入
`simulate/build/` 后运行 `./unitree_mujoco`。程序在未指定 RMW 时自动选择
Fast DDS，ROS Context 固定为域 10，宇树 SDK 固定使用配置中的域 0。
无需每次手动向 `LD_LIBRARY_PATH` 添加 `/opt/unitree_robotics/lib`：SDK 动态库
搜索路径已经写入可执行文件的 ELF RPATH。不必为此修改全局 shell 配置。
若终端明确设置了其他 `RMW_IMPLEMENTATION`，程序会提示使用
`rmw_fastrtps_cpp`，以保持本模块已验证的通信配置。

默认启用 ROS 传感器。`config/sensors.yaml` 中 `enabled: false` 可在运行时
关闭；`-DENABLE_ROS2_SENSORS=OFF` 可构建不依赖 ROS 的原有仿真器。
配置路径由 `simulate/config.yaml` 的 `ros2_sensors_config` 指定，
相对路径以 `simulate/` 为基准。找不到配置中的相机或雷达 site 时会提示并跳过，其他机器人
可以使用各自的传感器配置。

## 验收命令

另开一个终端（所有查看工具、RViz 和感知节点都使用域 10）：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10

ros2 topic list -t --no-daemon
ros2 node info /unitree_sim_sensors
ros2 topic info /camera/depth/image_raw --verbose
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /camera/depth/image_raw --once --no-arr
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo base_link front_camera_optical_frame
ros2 topic hz /lidar/points
ros2 topic echo /lidar/points --once --no-arr
ros2 run tf2_ros tf2_echo base_link mid360_frame
```

预期可见：

```text
/camera/color/image_raw   sensor_msgs/msg/Image       rgb8
/camera/depth/image_raw   sensor_msgs/msg/Image       32FC1，米
/camera/camera_info       sensor_msgs/msg/CameraInfo
/lidar/points             sensor_msgs/msg/PointCloud2  XYZ，米
/clock                   rosgraph_msgs/msg/Clock
/tf_static               tf2_msgs/msg/TFMessage
```

ROS 还可能显示 `/rosout` 等标准话题。彩色图、深度图、内参共享同一仿真采样
时间戳和 `front_camera_optical_frame`。深度为光轴方向距离，远裁剪面背景为 NaN。
图像与内参使用 SensorDataQoS（best effort、volatile、深度 1）；RViz 的
Image/Camera 显示应选 Best Effort。运行 `rviz2`，Fixed Frame 可先设
`base_link`，添加 Image 显示并选择彩色或深度话题。

相机目标频率为每秒仿真时间 15 帧，实际墙钟频率受仿真速度和渲染性能影响。
暂停仿真时停止常规采样，恢复或重置后继续。默认场景初始姿态下，前方箱体
正面到相机约 1.45 m；机器人自由落下或移动后，距离会随姿态改变。

检查域隔离（不会改动宇树 SDK 的域）：

```bash
ROS_DOMAIN_ID=0 ros2 topic list --no-daemon --spin-time 2
ROS_DOMAIN_ID=10 ros2 topic list --no-daemon --spin-time 2
```

域 0 不应出现 `/camera/*` 或 `/lidar/points`，域 10 应出现上述 ROS 话题。

## 3D 雷达

Go2W 场景默认启用 10 Hz、1000×20 的 XYZ 点云，视场为水平 360°、
垂直 -7°～52°，量程 0.1～40 m。采用上游规则射线网格和单时刻快照，
仅采用 MID-360 的视场参数，不模拟其非重复扫描轨迹或 Livox 协议。
雷达无需 EGL，使用私有仿真快照和独立计算线程，与相机一起由 SensorManager
管理。点云、TF、参数、生命周期和测试详见 [lidar/README.md](lidar/README.md)。
障碍提取和避障决策留给后续数据处理。
