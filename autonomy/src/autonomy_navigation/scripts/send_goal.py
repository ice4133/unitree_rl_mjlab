#!/usr/bin/python3
"""Send a bounded Nav2 goal in map coordinates; cancel on timeout or Ctrl-C."""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions


def wait(node, future, deadline):
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError('Navigation deadline exceeded')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=4.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0, help='radians')
    parser.add_argument('--timeout', type=float, default=120.0, help='wall-clock seconds, including startup')
    parser.add_argument('--domain', type=int, default=10)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw, args.timeout)) or args.timeout <= 0:
        parser.error('Coordinates must be finite and timeout must be positive')
    # Keep the ROS context alive during KeyboardInterrupt so cancellation can be sent.
    rclpy.init(args=[], domain_id=args.domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('autonomy_send_goal', parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    handle = None
    pending = None
    exit_code = 1
    try:
        deadline = time.monotonic() + args.timeout
        print('Waiting for active Nav2; stand and enter Velocity in the controller terminal.', flush=True)
        while not client.wait_for_server(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                raise TimeoutError('Nav2 action server unavailable')
        # A nonzero /clock is necessary for goal/TF timestamps.
        while node.get_clock().now().nanoseconds == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError('No simulation clock')
            rclpy.spin_once(node, timeout_sec=0.1)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
        pending = client.send_goal_async(goal)
        handle = wait(node, pending, deadline)
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        print(f'Goal accepted: map ({args.x:.2f}, {args.y:.2f}), yaw={args.yaw:.2f}', flush=True)
        result = wait(node, handle.get_result_async(), deadline)
        exit_code = 0 if result.status == GoalStatus.STATUS_SUCCEEDED else 1
        print(f'Navigation {"SUCCEEDED" if exit_code == 0 else "FAILED"}: status={result.status}', flush=True)
    except (TimeoutError, RuntimeError, KeyboardInterrupt) as error:
        reason = 'interrupted by user' if isinstance(error, KeyboardInterrupt) else str(error)
        print(f'Navigation stopped: {reason}', flush=True)
    finally:
        # Resolve an in-flight send before exiting, so it cannot leave an accepted orphan goal.
        if handle is None and pending is not None:
            try:
                handle = wait(node, pending, time.monotonic() + 3.0)
            except TimeoutError:
                print('Goal acknowledgement unavailable; check Nav2 before sending another goal.', flush=True)
        if handle is not None and handle.accepted and exit_code:
            try:
                wait(node, handle.cancel_goal_async(), time.monotonic() + 3.0)
                print('Goal cancellation requested.', flush=True)
            except TimeoutError:
                print('Cancellation timed out; use controller key 0 to enter damping.', flush=True)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
