cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)
include(CTest)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

# SDK2's Cyclone C++ library must use its matching Cyclone C library. Sourcing
# ROS adds a different libddsc.so.0 to LD_LIBRARY_PATH; ELF RPATH takes priority.
if(UNIX AND NOT APPLE)
  get_target_property(UNITREE_DDS_LIBRARY ddsc IMPORTED_LOCATION)
  get_filename_component(UNITREE_DDS_LIBRARY_DIR "${UNITREE_DDS_LIBRARY}" DIRECTORY)
  set_property(TARGET unitree_mujoco APPEND PROPERTY BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}")
  target_link_options(unitree_mujoco PRIVATE "LINKER:--disable-new-dtags")
endif()

option(ENABLE_ROS2_SENSORS "Publish simulated sensors over ROS 2 domain 10" ON)
if(ENABLE_ROS2_SENSORS)
  include(ros2_sensors/common/dependencies.cmake)
  add_subdirectory(ros2_sensors)
  target_compile_definitions(unitree_mujoco PRIVATE UNITREE_WITH_ROS2_SENSORS=1)
  target_link_libraries(unitree_mujoco ros2_sensors)
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
