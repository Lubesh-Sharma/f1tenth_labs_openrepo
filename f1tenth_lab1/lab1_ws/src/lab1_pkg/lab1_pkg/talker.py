import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self, speed, angle):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = speed
        self.angle = angle

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.angle * 3.14/180  # Convert angle from degrees to radians
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "Speed: %f, Steering Angle: %f"' % (msg.drive.speed, msg.drive.steering_angle))

def main(args=None):
    rclpy.init(args=args)

    v = float(input("Enter the speed: "))  # Prompt user for speed input
    d = float(input("Enter the steering angle in degrees: "))  # Prompt user for angle input

    node = MinimalPublisher(v, d)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
