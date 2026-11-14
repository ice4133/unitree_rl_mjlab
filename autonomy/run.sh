#!/usr/bin/env bash
set -e
autonomy_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
action="${1:-mapping}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$action" == build ]]; then
  cd "$autonomy_dir"
  exec /usr/bin/colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
fi
if [[ ! -f "$autonomy_dir/install/setup.bash" ]]; then
  echo "Build the workspace first: $autonomy_dir/run.sh build" >&2
  exit 1
fi
source "$autonomy_dir/install/setup.bash"
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
case "$action" in
  mapping|perception|navigation)
    exec ros2 launch autonomy_bringup bringup.launch.py "task:=$action" "$@" ;;
  check)
    exec ros2 run autonomy_bringup check_mapping "$@" ;;
  goal)
    exec ros2 run autonomy_navigation send_goal "$@" ;;
  check-navigation)
    exec ros2 run autonomy_bringup check_navigation "$@" ;;
  save-map)
    if [[ $# -eq 0 ]]; then
      echo "Usage: $0 save-map /absolute/path/to/map_prefix" >&2
      exit 1
    fi
    exec ros2 run nav2_map_server map_saver_cli -f "$1" --ros-args -p use_sim_time:=true ;;
  *)
    echo "Usage: $0 {build|mapping|perception|check|save-map|navigation|goal|check-navigation} [arguments]" >&2
    exit 1 ;;
esac
