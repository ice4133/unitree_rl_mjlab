# Resolve imported ROS targets in the parent scope as they are also linked into
# unitree_mujoco (some Humble exports use target-property generator expressions).
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
