import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

# Node class
class Talker(Node):

    # Constructor
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'drive',10)
        
        # Time interval in seconds for publish rate
        timer_period = 0.5

        self.timer = self.create_timer(timer_period,self.timer_callback)

        # Declare parameters
        self.declare_parameter('d',0.0)
        self.declare_parameter('v',0.0)


    # Publisher callback
    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = (self.get_parameter('d')).value
        msg.drive.speed = (self.get_parameter('v')).value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing steering angle and speed')


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
