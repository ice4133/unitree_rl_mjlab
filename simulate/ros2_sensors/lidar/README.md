# 3D lidar

射线生成和距离检测移植自 `mujoco_ros2_control` 的 `mujoco_3d_lidar`，
XYZ 点云组织方式移植自 `Mujoco3dLidarPlugin`。来源、版本和许可证见
上级目录的 `NOTICE` / `LICENSE`。实现直接链接到 `unitree_mujoco`，
与相机共享 `SensorManager` 和 ROS domain 10，无需单独启动雷达节点。

## 默认输出

- `/lidar/points`：`sensor_msgs/msg/PointCloud2`。
- `frame_id: mid360_frame`，静态 TF 为 `base_link -> mid360_frame`。
- `x/y/z` 为 FLOAT32，单位米，在雷达局部坐标系中，+X 前、+Y 左、+Z 上。
- 有组织点云：width=1000、height=20，先遍历水平方向，再遍历垂直方向。
- 每点 12 字节，每帧数据区 240,000 字节；不命中或超出量程的点保留为 NaN，
  `is_dense=false`。不填入虚构的 intensity、ring 或逐点时间字段。
- SensorDataQoS：best effort、volatile、深度 1。RViz PointCloud2 选择 Best Effort，
  Fixed Frame 可先设为 `base_link`。
- 目标频率为每秒仿真时间 10 帧，每帧最多 20,000 个有效返回点。
  时间戳对应整个扫描快照，与 `/clock` 使用同一仿真时间。

默认视场为 MID-360 的水平 360°、垂直 -7°～52°，量程 0.1～40 m。
沿用上游规则角度网格，不模拟 MID-360 的非重复扫描轨迹、逐点采样、
运动畸变、反射率、测距噪声或内置 IMU；不输出 Livox CustomMsg。
该输出适合后续自行处理 XYZ 点云，不表示完整复现真实 MID-360。

## 配置

在 `../config/sensors.yaml` 的 `lidars` 列表中配置，每次启动时读取：

| 字段 | 含义 |
| --- | --- |
| `name` / `site` | 传感器名称 / MJCF 安装 site；缺少 site 时提示并跳过 |
| `enabled` | 是否启用该雷达 |
| `frame_id` / `topic` | 雷达坐标系 / 点云话题 |
| `publish_rate` | 按仿真时间计算的扫描频率，必须为正数 |
| `resolution` | `[水平采样数, 垂直采样数]`，正整数，总数不超过 1,000,000 |
| `azimuth_range` | 水平角范围，单位弧度，位于 `[-pi, pi]` |
| `elevation_range` | 仰角范围，单位弧度，位于 `[-pi/2, pi/2]` |
| `range_min` / `range_max` | 最小 / 最大距离，单位米，有效区间为 `[min, max)` |
| `geom_groups` | 参与射线检测的 geom group 列表，空列表表示所有 group |

角度区间在有多个采样点时必须递增；完整水平 360° 不重复采样首尾方向。
默认检测 group 0（地面、前方箱体）和 group 3（机器人碰撞体），忽略
group 2 的外观网格。与上游一样，排除 site 所属 body 的几何体，其他 body
（例如腿和轮子）仍可能遮挡雷达。碰撞体轮廓与相机中的外观网格可能不同。

安装点只添加在 `scene_go2w.xml`，位于 `base_link` 上方 0.15 m；不增加质量、
关节、电机或原生 sensor 数据，不影响电机传感器数组布局和训练用 `go2w.xml`。

## 线程与生命周期

仿真线程按扫描频率复制状态。独立工作线程对私有双缓冲快照计算几何位姿和
BVH、执行 MuJoCo 3.3.6 的 `mj_multiRay`、转换并发布点云。正在计算的快照不会
被仿真线程写入，慢帧只覆盖待处理旧帧，不积压队列。重置时清理待处理帧、
重新计时并丢弃旧代扫描结果；模型释放前停止工作线程。
暂停后不再采样（已提交的扫描可能完成一次发布），恢复或重置后继续。

## 验证

```bash
cmake --build simulate/build -j2
ctest --test-dir simulate/build --output-on-failure
./simulate/build/unitree_mujoco
```

另开 ROS 2 终端：

```bash
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic info /lidar/points --verbose
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic hz /lidar/points
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic echo /lidar/points --once --no-arr
ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run tf2_ros tf2_echo base_link mid360_frame
```

实际墙钟频率受仿真速度、CPU、场景复杂度及其他传感器负载影响。
