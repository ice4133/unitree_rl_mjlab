#!/usr/bin/python3
"""Read-only bounded check of active Nav2, guarded velocity and path output."""
import argparse
import math
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Path
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--domain', type=int, default=10)
    parser.add_argument('--require-motion', action='store_true', help='Require a plan and nonzero velocity during observation')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    rclpy.init(args=[], domain_id=args.domain)
    node = rclpy.create_node('check_autonomy_navigation')
    velocities, plans, scans = [], [], []
    node.create_subscription(Twist, '/cmd_vel', lambda msg: velocities.append(
        (msg.linear.x, msg.linear.y, msg.angular.z)), 10)
    node.create_subscription(Path, '/plan', lambda msg: plans.append(len(msg.poses)), 10)
    node.create_subscription(LaserScan, '/scan', lambda msg: scans.append(time.monotonic()), qos_profile_sensor_data)
    failures = []
    try:
        servers = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                   'velocity_smoother', 'collision_monitor')
        clients = [node.create_client(GetState, f'/{server}/get_state') for server in servers]
        # Allow bounded DDS discovery before the requested observation window.
        startup_deadline = time.monotonic() + 5.0
        while (not velocities or not scans) and time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for server, client in zip(servers, clients):
            state = None
            # Service matching can precede reply-endpoint discovery with Fast DDS.
            # Retry a lost initial read; each read remains bounded.
            if client.wait_for_service(timeout_sec=0.5):
                for _ in range(2):
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                    if future.done() and future.result() is not None:
                        state = future.result().current_state
                        break
                    future.cancel()
            if state is None or state.id != 3:
                failures.append(f'{server}: {state.label if state else "no lifecycle response"}')
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        if len(publishers) != 1 or publishers[0].node_name != 'velocity_guard':
            failures.append('/cmd_vel must have exactly one publisher: velocity_guard')
        if not scans or time.monotonic() - scans[-1] > 0.5:
            failures.append('No fresh /scan')
        if not velocities:
            failures.append('No guarded /cmd_vel')
        if any(not all(math.isfinite(v) for v in values) or not (-0.1501 <= values[0] <= 0.3501)
               or values[1] != 0.0 or abs(values[2]) > 0.6001 for values in velocities):
            failures.append('Non-finite velocity or Go2W navigation limits exceeded')
        moving = sum(abs(v[0]) + abs(v[2]) > 0.01 for v in velocities)
        if args.require_motion and (not any(plans) or not moving):
            failures.append('Expected an active goal with a path and motion output')
        print(f'scan={len(scans)}, cmd_vel={len(velocities)}, moving={moving}, plans={len(plans)}')
        for failure in failures:
            print('FAIL:', failure)
        if not failures:
            print('PASS: Nav2 active, single guarded auto velocity publisher, fresh scan and valid speed limits')
        return bool(failures)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
