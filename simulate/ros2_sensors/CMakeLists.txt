add_library(ros2_sensors STATIC
  common/sensor_manager.cpp
  common/offscreen_context.cpp
  camera/camera_sensor.cpp
)
target_compile_features(ros2_sensors PUBLIC cxx_std_17)
target_include_directories(ros2_sensors PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco/include
)
ament_target_dependencies(ros2_sensors
  rclcpp sensor_msgs geometry_msgs rosgraph_msgs tf2_ros
)
target_link_libraries(ros2_sensors
  mujoco yaml-cpp OpenGL::OpenGL OpenGL::EGL pthread
)
