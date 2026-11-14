# Upstream components

- `src/third_party/pointcloud_to_laserscan` is an unchanged source snapshot of
  https://github.com/ros-perception/pointcloud_to_laserscan, branch `humble`,
  commit `59bf996fb3ee7db0026a5cd3ce0d2a39d2e602ea` (BSD-3-Clause).
  Its LICENSE and copyright headers are retained. `colcon.meta` disables only
  upstream lint tests; our own perception tests remain enabled.
- SLAM uses the installed ROS 2 Humble `slam_toolbox` package. The mapper
  configuration follows its `mapper_params_online_async.yaml`; this workspace
  does not copy or modify its mapping, matching or optimization algorithms.
- Navigation uses the installed ROS 2 Humble Nav2 servers and plugins. The
  `bt_navigator` plugin list follows `nav2_bringup/params/nav2_params.yaml`
  (Apache-2.0); the default behavior trees are loaded from the installed Nav2
  package. Planning, path following and collision monitoring algorithms are
  not vendored here. Go2W-specific limits, launches and the final velocity
  watchdog are maintained in `autonomy_navigation`.
- The existing simulated lidar/odometry follow `mujoco_ros2_control`. That
  repository provides simulator sensors, but does not contain a SLAM Toolbox
  or Nav2 bringup to transplant. Here we consume its adapted ROS interfaces.

The custom perception adapter handles the Go2W body mask, ground-height band
and level scan frames. Scan projection is performed by the upstream package.
