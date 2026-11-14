add_library(ros2_sensors STATIC
  common/sensor_manager.cpp
  common/offscreen_context.cpp
  camera/camera_sensor.cpp
  lidar/lidar_sensor.cpp
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

if(BUILD_TESTING)
  add_executable(lidar_sensor_test tests/lidar_sensor_test.cpp)
  target_link_libraries(lidar_sensor_test ros2_sensors)
  if(UNIX AND NOT APPLE)
    set_property(TARGET lidar_sensor_test APPEND PROPERTY BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}")
    target_link_options(lidar_sensor_test PRIVATE "LINKER:--disable-new-dtags")
  endif()
  add_test(NAME lidar_sensor COMMAND lidar_sensor_test)
  set_tests_properties(lidar_sensor PROPERTIES TIMEOUT 30
    ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
endif()
