# 导航参数

`nav2.yaml` 供 ROS 2 Humble 的 Go2W 平地导航使用，涵盖 Navfn、RPP、
全局/局部代价地图、行为恢复、速度平滑、碰撞监控和最终速度守护。
顶层通过 `nav_params:=/absolute/path/nav2.yaml` 替换。

接口为 `map` / `odom` / `base_footprint`，输入 `/map`、`/odom`、`/scan` 与 TF，
最终输出 `geometry_msgs/msg/Twist` 到 `/cmd_vel`。内部速度使用 `/navigation/*`。

修改机器人外形时同步更新两张代价地图的 footprint 和 collision_monitor 的停止矩形；
修改速度时同步更新控制器、恢复、平滑器、守护配置，保持在部署策略范围内。
超时使用秒；守护接收超时使用单调时钟，采样年龄和 TF 使用 ROS 时钟。
