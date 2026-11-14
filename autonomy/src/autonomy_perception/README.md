# 数据处理层

数据链路：`/lidar/points` -> `lidar_filter_node` ->
`/perception/lidar/points_filtered` -> 官方 `pointcloud_to_laserscan` -> `/scan`。

Go2W 适配节点等待扫描时刻的 TF，用 odom 的 Z 轴进行高度筛选，用 base_link
包围盒过滤自身点，保留所有有效点的其他字段，只改写 XYZ 并保持原始采样时间。
它创建两个独立的 odom 子坐标系：

- `base_footprint`：机体的地面 XY 投影，保留 yaw。
- `lidar_scan_frame`：原雷达位置，保留机体 yaw，去掉 roll/pitch。

输出云保留三维高度，后续也可用于代价地图；仅 `/scan` 是二维投影。
原有 `odom -> base_link -> mid360_frame` 不被重发或改父坐标系。

第一版针对平地：`ground_height` 是 odom 中地面高度，默认 MuJoCo 的 z=0。
这不是坡地分割器，也不处理动态目标或点级运动畸变。实机/坡地需要地面估计及
相应参数；高度低于 8 cm 的物体会被当前配置过滤，不能据此验证低矮障碍物避障。

相机深度预处理后续也归本包；当前相机数据继续由 simulate 原样发布。
