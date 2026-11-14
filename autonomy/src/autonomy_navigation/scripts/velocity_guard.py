#!/usr/bin/python3
"""Final auto velocity output with steady-clock scan/command watchdogs.

Humble collision_monitor ignores expired sources. This node closes that gap and
also prevents replayed scans, missing transforms and invalid commands passing.
"""
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        defaults = {
            'input_topic': '/navigation/cmd_vel_checked', 'output_topic': '/cmd_vel',
            'scan_topic': '/scan', 'base_frame': 'base_footprint', 'odom_frame': 'odom',
            'scan_timeout': 0.5, 'command_timeout': 0.3, 'transform_timeout': 0.3,
            'publish_rate': 20.0, 'max_forward': 0.35, 'max_reverse': 0.15, 'max_yaw': 0.6,
        }
        self.cfg = {name: self.declare_parameter(name, default).value for name, default in defaults.items()}
        for name, value in self.cfg.items():
            if isinstance(defaults[name], float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
            if isinstance(defaults[name], str) and not value:
                raise ValueError(f'{name} must not be empty')
        if self.cfg['input_topic'] == self.cfg['output_topic']:
            raise ValueError('Velocity guard input/output must differ')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.command = Twist()
        self.command_received = -math.inf
        self.scan_received = -math.inf
        self.scan_stamp = None
        self.scan_frame = ''
        self.publisher = self.create_publisher(Twist, self.cfg['output_topic'], 1)
        self.create_subscription(Twist, self.cfg['input_topic'], self.on_command, 1)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, qos_profile_sensor_data)
        # Continue expiring inputs when /clock pauses or no clock has arrived.
        self.timer = self.create_timer(1.0 / self.cfg['publish_rate'], self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, message):
        if not all(math.isfinite(v) for v in (message.linear.x, message.linear.y, message.angular.z)):
            self.command_received = -math.inf
            return
        self.command = Twist()
        self.command.linear.x = max(-self.cfg['max_reverse'], min(self.cfg['max_forward'], message.linear.x))
        self.command.angular.z = max(-self.cfg['max_yaw'], min(self.cfg['max_yaw'], message.angular.z))
        self.command_received = time.monotonic()

    def on_scan(self, message):
        stamp = Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        valid = (message.header.frame_id and len(message.ranges) > 0
                 and math.isfinite(message.angle_increment) and message.angle_increment > 0
                 and math.isfinite(message.range_min) and math.isfinite(message.range_max)
                 and 0 <= message.range_min < message.range_max
                 and any((math.isfinite(r) and message.range_min <= r <= message.range_max)
                         or r == math.inf for r in message.ranges)
                 and -0.1 <= age <= self.cfg['scan_timeout'])
        if not valid:
            self.scan_received = -math.inf
            return
        # A repeated old sample cannot keep the watchdog alive.
        if self.scan_stamp is not None and stamp <= self.scan_stamp:
            return
        self.scan_stamp = stamp
        self.scan_frame = message.header.frame_id
        self.scan_received = time.monotonic()

    def ready(self):
        now = time.monotonic()
        if (now - self.command_received >= self.cfg['command_timeout']
                or now - self.scan_received >= self.cfg['scan_timeout']):
            return False
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        if not -0.1 <= age <= self.cfg['scan_timeout']:
            return False
        try:
            self.buffer.lookup_transform(self.cfg['base_frame'], self.scan_frame, self.scan_stamp,
                                         timeout=Duration())
            transform = self.buffer.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'], Time(),
                                                      timeout=Duration())
            # Static transforms have stamp zero and are valid indefinitely (also useful for tests).
            stamp = Time.from_msg(transform.header.stamp)
            if stamp.nanoseconds:
                age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
                if not -0.1 <= age <= self.cfg['transform_timeout']:
                    return False
        except TransformException:
            return False
        return True

    def tick(self):
        self.publisher.publish(self.command if self.ready() else Twist())


def main():
    rclpy.init()
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
