# Lidar extension point

No lidar is enabled or simulated yet. Add the lidar implementation here when
the MJCF lidar model and scan pattern have been chosen. Implement the shared
`common/sensor.hpp` lifecycle (`sample`, `reset`, `stop`) and register it in
`SensorManager::load_model`. Reuse the manager's ROS node in domain 10 and
publish standard `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2` messages.
Lidar remains independent of the Unitree SDK2 motor channels in domain 0.
