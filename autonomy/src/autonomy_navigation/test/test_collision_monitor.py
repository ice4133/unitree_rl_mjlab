"""Exercise the installed Humble monitor with real DDS, TF and lifecycle transitions."""
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_collision_stop_and_sensor_timeout():
    # Keep test velocities away from the simulator/controller domain 10.
    domain = 197
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    config = Path(get_package_share_directory('autonomy_navigation')) / 'config/nav2.yaml'
    processes = []
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('test_collision_input', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        for package, executable in [('nav2_collision_monitor', 'collision_monitor'),
                                    ('autonomy_navigation', 'velocity_guard')]:
            processes.append(subprocess.Popen([
                'ros2', 'run', package, executable, '--ros-args',
                '--params-file', str(config), '-p', 'use_sim_time:=false'], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True))
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        scan_pub = node.create_publisher(LaserScan, '/scan', 10)
        cmd_pub = node.create_publisher(Twist, '/navigation/cmd_vel_smoothed', 10)
        received = []
        node.create_subscription(Twist, '/cmd_vel', lambda msg: received.append(msg.linear.x), 10)
        change = node.create_client(ChangeState, '/collision_monitor/change_state')
        assert change.wait_for_service(timeout_sec=10.0), 'Collision monitor did not start'
        for transition in (1, 3):  # configure, activate
            request = ChangeState.Request()
            request.transition.id = transition
            future = change.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            assert future.done() and future.result().success, 'Lifecycle transition failed'

        def drive(duration, distance, send_command=True, frame='base_footprint', replay=False, speed=0.2):
            received.clear()
            deadline = time.monotonic() + duration
            stamp = node.get_clock().now().to_msg()
            while time.monotonic() < deadline:
                if distance is not None:
                    scan = LaserScan()
                    scan.header.stamp = stamp if replay else node.get_clock().now().to_msg()
                    scan.header.frame_id = frame
                    scan.angle_min = -0.01
                    scan.angle_max = 0.01
                    scan.angle_increment = 0.01
                    scan.range_min = 0.1
                    scan.range_max = 20.0
                    scan.ranges = [distance] * 3
                    scan_pub.publish(scan)
                command = Twist()
                command.linear.x = speed
                if send_command:
                    cmd_pub.publish(command)
                executor.spin_once(timeout_sec=0.025)
                # A ready subscription can return immediately; bound the publication rate.
                time.sleep(0.025)
            assert received, 'Monitor produced no output'
            return received[-1]

        assert drive(1.0, 10.0) > 0.19, 'Clear scan should pass forward command'
        assert drive(0.4, 0.3) == 0.0, 'Front obstacle must stop forward motion'
        assert drive(0.4, 10.0) > 0.19, 'Fresh clear scan should release stop'
        assert drive(0.8, None) == 0.0, 'Missing scan must stop within source_timeout'
        assert drive(0.4, 10.0) > 0.19, 'Fresh scan should restore motion after timeout'
        assert drive(0.6, 10.0, send_command=False) == 0.0, 'Missing command must stop'
        assert drive(0.4, 10.0) > 0.19, 'Fresh command should restore motion'
        assert drive(0.4, float('nan')) == 0.0, 'Invalid scan must stop'
        assert drive(0.4, 10.0, frame='missing_scan_frame') == 0.0, 'Missing scan TF must stop'
        assert drive(0.4, 10.0) > 0.19, 'Valid TF should restore motion'
        assert drive(0.8, 10.0, replay=True) == 0.0, 'Replayed scans cannot keep motion alive'
        assert drive(0.4, 10.0, speed=float('nan')) == 0.0, 'Non-finite command must stop'
    finally:
        import signal
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
