# Go2W 自动避障导航

本包将 Nav2 接到部署控制器的 auto 速度接口。全局规划使用 Navfn A*，
局部控制使用 Regulated Pure Pursuit，适配不能横移、能原地转向的 Go2W。
感知属于 `autonomy_perception`，在线 SLAM 属于 `autonomy_localization`，
组合启动属于 `autonomy_bringup`，ONNX 与电机控制仍在 `deploy`。

从仓库根目录运行 `./autonomy/run.sh navigation` 会一起启动感知、SLAM、Nav2、
速度守护和导航 RViz。先将部署控制器置于 auto，并完成站立和进入 Velocity。
完整步骤见 [工作空间说明](../../README.md)。

```bash
./autonomy/run.sh goal --x 4 --y 0 --yaw 0 --timeout 120
./autonomy/run.sh check-navigation --require-motion
```

`goal` 使用 NavigateToPose action，目标在 map 坐标系；超时使用墙钟，退出前请求取消。
也可使用 RViz 的 Nav2 Goal。启动导航本身不自动发送目标。

`config/nav2.yaml` 是 Humble 参数，包含：

- 全局 static + scan obstacle + inflation 代价地图；局部 6×6 m 滚动 scan 代价地图。
- 1.0×0.7 m 矩形 footprint，额外 0.03 m padding，0.9 m 膨胀半径。
- 在线 SLAM 未知区域允许规划；实时障碍层、跟踪器碰撞检查与停止矩形均生效。
- RPP 期望前进 0.30 m/s，转向对齐 0.45 rad/s；平滑后限制 vx ∈ [-0.15, 0.35]、
  vy = 0、wz ∈ [-0.6, 0.6]，低于现有策略范围。
- 跟踪器的转向加速度参数 7.0（≥15 Hz × 0.45 rad/s）允许从静止给出完整转向目标，
  避免策略在微小指令下停滞；
  下游平滑器仍限制指令加速度为 0.8 rad/s²、前进 0.4 m/s²。
- Nav2 标准行为树定期重规划；恢复包含清图、原地转向、等待和低速后退。
  后退也经过相同的全周碰撞监控和守护。

速度链路只有一个最终发布者：

```text
controller_server / behavior_server -> /navigation/cmd_vel_raw
velocity_smoother                  -> /navigation/cmd_vel_smoothed
collision_monitor                  -> /navigation/cmd_vel_checked
velocity_guard                     -> /cmd_vel
```

Humble 碰撞监控的过期数据源会被忽略，而非自动停车。
`velocity_guard` 用 steady timer 检查扫描接收间隔、严格递增的采样时间戳、
ROS 时间下的采样年龄、扫描到基座 TF、最新 odom TF 和指令接收间隔。
输入未就绪、全无效扫描、非有限速度、数据超时或 TF 不可用时发布零速度，
恢复后自动放行；仿真重置时间后需重启整套导航。停止矩形覆盖前后左右，单点侵入即停车。

单独启动本包仅启动导航，不提供感知或定位；调用者必须已有 `/scan`、`/map`、`/odom`、
`map -> odom -> base_footprint` 和雷达 TF，且负责相同 ROS domain 与中间件。
统一入口默认 domain 10 / Fast DDS / 仿真时间；SDK2 保持 domain 0。

实现依据：[Humble RPP](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html)、
[Humble scan 超时处理源码](https://api.nav2.org/nav2-humble/html/scan_8cpp_source.html)。
