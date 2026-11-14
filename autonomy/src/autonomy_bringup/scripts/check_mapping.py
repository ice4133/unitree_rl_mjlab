#!/usr/bin/python3
"""Read-only, bounded checks for the running domain-10 mapping pipeline."""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--domain', type=int, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive and finite')
    rclpy.init(args=[], domain_id=args.domain)
    node = rclpy.create_node('check_autonomy_mapping', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    data = {key: [] for key in ('raw', 'filtered', 'scan', 'odom', 'clock', 'map')}

    def record(key):
        def callback(message):
            # Keep only small metadata for clouds, whose payload is not needed here.
            if key in ('raw', 'filtered'):
                data[key].append((time.monotonic(), message.header.stamp, message.width * message.height))
            else:
                data[key].append((time.monotonic(), message))
        return callback

    node.create_subscription(PointCloud2, '/lidar/points', record('raw'), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/perception/lidar/points_filtered', record('filtered'), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', record('scan'), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', record('odom'), 10)
    node.create_subscription(Clock, '/clock', record('clock'), qos_profile_sensor_data)
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', record('map'), map_qos)
    failures = []
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for key, values in data.items():
            if not values:
                failures.append(f'No {key} data in domain {args.domain}')
            elif key != 'map':
                rate = (len(values) - 1) / (values[-1][0] - values[0][0]) if len(values) > 1 else 0
                print(f'{key}: {len(values)} messages, {rate:.2f} Hz')
        if data['scan']:
            scan = data['scan'][-1][1]
            if scan.header.frame_id != 'lidar_scan_frame' or not any(math.isfinite(r) for r in scan.ranges):
                failures.append('Scan frame is wrong or all ranges are empty')
            raw_stamps = {(s.sec, s.nanosec) for _, s, _ in data['raw']}
            matching = sum((m.header.stamp.sec, m.header.stamp.nanosec) in raw_stamps for _, m in data['scan'])
            if raw_stamps and matching < max(1, len(data['scan']) - 5):
                failures.append('Scan timestamps do not follow raw lidar timestamps')
            print(f'scan: {len(scan.ranges)} bins, {matching} matching raw timestamps')
        if data['map']:
            grid = data['map'][-1][1]
            occupied = sum(v >= 65 for v in grid.data)
            free = sum(0 <= v <= 25 for v in grid.data)
            print(f'map: {grid.info.width} x {grid.info.height}, {grid.info.resolution:.3f} m/cell, '
                  f'occupied={occupied}, free={free}')
            if grid.header.frame_id != 'map' or not occupied or not free:
                failures.append('Map frame/content is invalid')
        for frame in ('base_link', 'mid360_frame', 'base_footprint', 'lidar_scan_frame'):
            try:
                buffer.lookup_transform('map', frame, Time())
                print(f'TF map -> {frame}: OK')
            except Exception as error:
                failures.append(f'TF map -> {frame}: {error}')
        for failure in failures:
            print('FAIL:', failure)
        if not failures:
            print(f'PASS: lidar -> filtered cloud -> scan -> SLAM map, domain {args.domain}')
        return bool(failures)
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
