import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)

        # Publish to drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Replace this with your own preprocessing logic
        window_size = 5
        proc_ranges = np.convolve(ranges, np.ones(window_size) / window_size, mode='same')
        proc_ranges[ranges > 3.0] = 0.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # Find the start and end indices of the largest gap
        gap_start, gap_end = 0, 0
        max_gap_length = 0
        current_gap_start = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] == 0:
                # Current point is not part of the gap
                gap_length = i - current_gap_start
                if gap_length > max_gap_length:
                    max_gap_length = gap_length
                    gap_start, gap_end = current_gap_start, i - 1

                current_gap_start = i + 1

        return gap_start, gap_end
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of the best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # Find the index of the furthest point within the gap
        best_point_index = start_i
        max_range = 0.0

        for i in range(start_i, end_i + 1):
            if ranges[i] > max_range:
                max_range = ranges[i]
                best_point_index = i

        return best_point_index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Find closest point to LiDAR
        closest_point_index = np.argmin(proc_ranges)

        # Draw a safety bubble around the closest point and set all points inside this bubble to 0
        bubble_radius = 1  # Set your own bubble radius
        proc_ranges[max(0, closest_point_index - bubble_radius):closest_point_index + bubble_radius + 1] = 0.0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best goal point in this gap (using the naive approach)
        best_point_index = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header
        drive_msg.drive.steering_angle = data.angle_min + best_point_index * data.angle_increment
        drive_msg.drive.speed = 1.0  # Set your desired speed
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

