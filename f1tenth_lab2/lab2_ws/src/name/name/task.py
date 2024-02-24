#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):

        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        ittc_values = []
        for i in range(len(scan_msg.ranges)):
            current_range = scan_msg.ranges[i]
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            range_rate = self.speed * np.cos(angle)

            if range_rate >= 0.0:
                ittc = float('inf')
            else:
                ittc = current_range / max(-range_rate, 0.0)
            ittc_values.append(ittc)

        print("Time to collision",min(ittc_values))
        if any(ittc <0.8 for ittc in ittc_values):
            print("Brake Applied .......................................................................\n\n\n\n")
            brake_cmd = AckermannDriveStamped()
            brake_cmd.header.stamp = self.get_clock().now().to_msg()
            brake_cmd.drive.steering_angle = 0.0
            brake_cmd.drive.speed = 0.0
            brake_cmd.drive.acceleration = 0.0
            brake_cmd.drive.jerk = 0.0
            brake_cmd.drive.steering_angle_velocity = 0.0
            self.drive_pub.publish(brake_cmd)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
