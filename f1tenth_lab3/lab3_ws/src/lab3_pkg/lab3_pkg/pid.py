import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Set PID gains
        self.kp = 0.75
        self.kd = 0.70
        self.ki = 0.01

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # Lookahead distance
        self.lookahead_distance = 0.0001 # You can adjust this value

    def get_alpha(self, a, b, theta):
        """
        Calculate the angle alpha between the car's x-axis and the right wall using laser scan distances a and b.

        Args:
            a: distance to the wall at angle 0 degrees
            b: distance to the wall at angle theta
            theta: angle between the two laser scans

        Returns:
            alpha: calculated angle in radians
        """
        alpha = np.arctan2((a * np.cos(theta) - b), (a * np.sin(theta)))
        return alpha

    def get_current_distance(self, b, alpha):
        """
        Calculate the current distance between the car and the right wall.

        Args:
            b: distance to the wall at angle theta
            alpha: calculated angle in radians

        Returns:
            current_distance: current distance to the wall
        """
        current_distance = b * np.cos(alpha)
        return current_distance

    def get_future_distance(self, current_distance, alpha):
        """
        Estimate the future distance between the car and the right wall.

        Args:
            current_distance: current distance to the wall
            alpha: calculated angle in radians

        Returns:
            future_distance: estimated future distance to the wall
        """
        future_distance = current_distance + self.lookahead_distance * np.sin(alpha)
        return future_distance

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = np.arctan2(self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error), velocity)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def speed_control(self, steering_angle):
        """
        Compute the driving speed based on the steering angle.

        Args:
            steering_angle: calculated steering angle

        Returns:
            speed: calculated driving speed
        """
        if 0 <= np.abs(np.degrees(steering_angle)) <= 10:
            speed = 1.50
        elif 10 < np.abs(np.degrees(steering_angle)) <= 20:
            speed = 1.00
        else:
            speed = 0.70
        return speed

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # Obtain two laser scans (distances) a and b
        a = msg.ranges[0]  # Assuming distance at 0 degrees
        theta = np.radians(75)  # Set the angle between beams a and b (adjust as needed)
        b = msg.ranges[int(theta / msg.angle_increment)]

        # Calculate the angle alpha
        alpha = self.get_alpha(a, b, theta)

        # Calculate the current distance to the wall
        current_distance = self.get_current_distance(b, alpha)

        # Estimate the future distance to the wall
        future_distance = self.get_future_distance(current_distance, alpha)

        # Calculate the error (difference between desired and future distance)
        self.error = 1.0 - future_distance  # Set desired distance to 1.0 meters

        # Update integral term
        self.integral += self.error

        # Run D(t+1) through the PID algorithm to get a steering angle
        steering_angle = np.arctan2(self.kp * self.error + self.ki * self.integral + self.kd * (self.error - self.prev_error), 1.0)

        # Use the steering angle to compute a safe driving speed
        speed = self.speed_control(steering_angle)

        # Publish the steering angle and driving speed
        self.pid_control(self.error, speed)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
