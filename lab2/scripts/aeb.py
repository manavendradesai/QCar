#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import numpy as np

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

        # Declare parameters
        self.declare_parameter('TTC_cr',1.5)

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
        self.laserrange = np.array(msg.ranges)
        self.laserangle_min = msg.angle_min
        self.laserangle_inc = msg.angle_increment

        # Filter scan for inf and nan
        np.nan_to_num(self.laserrange,copy=False,nan=msg.range_max,posinf=msg.range_max,neginf=msg.range_max)

        # set flag to true
        self.flag_laser = True

    # Callback for odometry message
    def aeb_callback(self,msg):

        # Collect vx
        vx = msg.twist.twist.linear.x

        # Collect critical time-to-collison threshold
        self.TTC_cr = (self.get_parameter('TTC_cr')).value

        # Execute only if filtered laser scan is received
        if self.flag_laser:

            min_iTTc = 100

            # Calculate instantaneous time-to-collision
            for i in range(0,len(self.laserrange)):

                iTTc = self.laserrange[i]/(max(0.001,vx*np.cos(self.laserangle_min+i*self.laserangle_inc)))

                min_iTTc = min(min_iTTc,iTTc)

            # Determine if AEB is necessary
            if min_iTTc<self.TTC_cr:

                aeb_msg = AckermannDriveStamped()
                aeb_msg.drive.speed = 0.0

                # Apply AEB
                self.get_logger().info('AEB applied!!')
                self.publisher_.publish(aeb_msg)   


            # Reset flag
            self.flag_laser = False


def main(args=None):
    rclpy.init(args=args)
    aeb = AEB()
    rclpy.spin(aeb)
    rclpy.shutdown()

if __name__ == "__main__":
    main()