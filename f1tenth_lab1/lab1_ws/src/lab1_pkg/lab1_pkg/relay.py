import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10
        )

    def drive_callback(self, msg):
        # Multiply speed and steering_angle by 3
        msg.drive.speed *= 3
        msg.drive.steering_angle *= 3

        # Print modified values
        self.get_logger().info('RelayNode - Modified Speed: %f, Modified Steering Angle: %f' %
                               (msg.drive.speed, msg.drive.steering_angle))

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
