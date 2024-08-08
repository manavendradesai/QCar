#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

# Node class
class GapFollow(Node):


    # Constructor
    def __init__(self):
        super().__init__('wall_follow')

        # Declare variables
        self.flag_laser = False

        self.laserrange = 0.0
        self.laserangle_min = 0.0
        self.laserangle_inc = 0.0

        # Declare subscriber for laserscan msgs
        self.subscription = self.create_subscription(LaserScan,'scan',self.laserscan_callback,10
        )    

        # Declare publisher for ackermann msgs
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'drive',10)


    # Callback for laserscan message
    def laserscan_callback(self,msg):

        # Receive scan
        self.laserrange = np.array(msg.ranges)
        self.laserangle_min = msg.angle_min
        self.laserangle_inc = msg.angle_increment

        # Filter scan for inf and nan
        np.nan_to_num(self.laserrange,copy=False,nan=msg.range_max,posinf=msg.range_max,neginf=msg.range_max)

        # Set flag to true
        self.flag_laser = True

        # Set bubble


    # Bubble
    def make_bubble():


    # Find max gap
    def max_gap():


    # Find best point
    def best_point(): 

    



def main(args=None):
    rclpy.init(args=args)
    wall_follow = GapFollow()
    rclpy.spin(wall_follow)
    rclpy.shutdown()

if __name__ == "__main__":
    main()