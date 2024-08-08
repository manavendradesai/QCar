import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

# Node class
class Relay(Node):

    # Constructor
    def __init__(self):
        super().__init__('relay')

        # Declare subscriber
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.relay_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Declare publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'drive_relay',10)
        

    def relay_callback(self,msg):
            # Receive message
            msg.drive.steering_angle = 3*msg.drive.steering_angle
            msg.drive.speed = 3*msg.drive.speed

            # Publish
            self.get_logger().info('Publishing mutliplied steering angle and speed')
            self.publisher_.publish(msg)       

def main(args=None):
    rclpy.init(args=args)
    relay = Relay()
    rclpy.spin(relay)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
