cmake_minimum_required(VERSION 3.12)
project(go2w_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

option(ENABLE_ROS2_COMMANDS "Receive automatic velocity commands over ROS 2" ON)
if(ENABLE_ROS2_COMMANDS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rmw_fastrtps_cpp REQUIRED)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
if(ENABLE_ROS2_COMMANDS)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE UNITREE_WITH_ROS2_COMMANDS=1)
  ament_target_dependencies(${PROJECT_NAME}_lib rclcpp geometry_msgs)
endif()
link_libraries(${PROJECT_NAME}_lib)

add_executable(go2w_ctrl main.cpp)

include(CTest)
if(BUILD_TESTING AND EXISTS "${PROJECT_SOURCE_DIR}/tests/velocity_command_test.cpp")
  add_executable(velocity_command_test tests/velocity_command_test.cpp)
  if(ENABLE_ROS2_COMMANDS)
    target_compile_definitions(velocity_command_test PRIVATE UNITREE_WITH_ROS2_COMMANDS=1)
    ament_target_dependencies(velocity_command_test rclcpp geometry_msgs)
  endif()
  add_test(NAME velocity_command COMMAND velocity_command_test
    "${PROJECT_SOURCE_DIR}/config/policy/velocity/v0/params/deploy.yaml")
  set_tests_properties(velocity_command PROPERTIES TIMEOUT 20
    ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=0")
endif()

# Keep SDK2's Cyclone C and C++ libraries matched even when ROS is sourced.
# ELF RPATH takes priority over ROS library paths in LD_LIBRARY_PATH.
if(UNIX AND NOT APPLE)
  get_target_property(UNITREE_DDS_LIBRARY ddsc IMPORTED_LOCATION)
  get_filename_component(UNITREE_DDS_LIBRARY_DIR "${UNITREE_DDS_LIBRARY}" DIRECTORY)
  foreach(runtime_target go2w_ctrl velocity_command_test)
    if(TARGET ${runtime_target})
      set_property(TARGET ${runtime_target} APPEND PROPERTY BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}")
      # LINK_FLAGS preserves compatibility with the CMake 3.12 minimum above.
      set_property(TARGET ${runtime_target} APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--disable-new-dtags")
    endif()
  endforeach()
endif()
