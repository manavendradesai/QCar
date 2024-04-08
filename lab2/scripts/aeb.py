import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Node class
class AEB(Node):

    # Constructor
    def __init__(self):
        super().__init__('aeb')

        # Declare variables
        self.flag_laser = False
        self.laserrange = 0.0
        self.laserangle_min = 0.0
        self.laserangle_inc = 0.0
        self.TTC_cr = 0.0

        # Declare subscriber for laserscan msgs
        self.subscription = self.create_subscription(LaserScan,'scan',self.laserscan_callback,10
        )        

        # Declare subscriber for odom msgs
        self.subscription = self.create_subscription(Odometry,'ego_racecar/odom',self.aeb_callback,10
        )     

        # Declare publisher for ackermann msgs
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'drive',10)


    # Callback for laserscan message
    def laserscan_callback(self,msg):

        # Receive scan
        self.laserrange = msg.ranges

        # Filter scan for inf
        

        # Filter scan for nan


        # set flag to true

    # Callback for laserscan message
    def aeb_callback(self,msg):

        # Collect vx


        # Collect critical time-to-collison threshold


        # Execute only if filtered laser scan is received


            # Calculate instantaneous time-to-collision


            # Determine if AEB is necessary


def main(args=None):
    rclpy.init(args=args)
    relay = AEB()
    rclpy.spin(relay)
    rclpy.shutdown()

if __name__ == "__main__":
    main()