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

        # Create ROS subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        nearest_distance = min(scan_msg.ranges)
        relative_speed = abs(self.speed)
        time_to_collision = nearest_distance / relative_speed if relative_speed != 0.0 else float('inf')

        print("Time to collision is ",time_to_collision)

        if time_to_collision < 1: 
            print("Brake Applied .......................................................................\n")
            brake_cmd = AckermannDriveStamped()
            brake_cmd.header.stamp = self.get_clock().now().to_msg()
            brake_cmd.drive.steering_angle = 0.0  # Keep steering angle unchanged
            brake_cmd.drive.speed = 0.0  # Set linear speed to 0 for braking
            brake_cmd.drive.acceleration = 0.0  # Set acceleration to 0
            brake_cmd.drive.jerk = 0.0  # Set jerk to 0
            brake_cmd.drive.steering_angle_velocity = 0.0  # Set steering angle velocity to 0
            self.drive_pub.publish(brake_cmd)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

