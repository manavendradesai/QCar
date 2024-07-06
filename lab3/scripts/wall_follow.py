#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

import numpy as np

# Node class
class WallFollow(Node):

    # Constructor
    def __init__(self):
        super().__init__('wall_follow')

        # Declare variables
        self.flag_laser = False
        self.flag_error = False

        self.laserrange = 0.0
        self.laserangle_min = 0.0
        self.laserangle_inc = 0.0

        self.TTC_cr = 0.0

        # Zero of LiDAR is along x of base-link
        self.rnghead_low = 30*(np.pi/180)
        self.rnghead_high = 90*(np.pi/180)
        self.theta = self.rnghead_high - self.rnghead_low
        self.L = 1.5

        # PID gains
        self.kp = 0.5
        self.kd = 5
        self.ki = 0

        # Error history
        # self.integral =
        self.pres_error = 0
        self.prev_error = 0

        # Desired distance to the wall in meters
        self.dist_to_wall = 0.8

        # Declare parameters
        self.declare_parameter('TTC_cr',1.5)

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

        # Get tracking error
        self.get_error()

        # Publish commands for PID steering, and velocity
        if self.flag_error:
            self.pid_control()


    # Helper functin to get laserscan range for given angle
    def get_range(self,laserrange,angle):

        # Run only if laserscan is received and filtered
        if self.flag_laser:
            
            # Index of heading. Neglect decimal.
            i = int( (angle-self.laserangle_min)//self.laserangle_inc )

            # print('Index: ', i)

            return laserrange[i]
        

    # Get cross-track error from left wall
    def get_error(self):

        r_high = self.get_range(self.laserrange,self.rnghead_high)

        r_low = self.get_range(self.laserrange,self.rnghead_low)

        alpha = np.arctan2( (r_high - r_low*np.cos(self.theta)) , (r_low*np.sin(self.theta)) )

        r = r_high*np.cos(alpha)

        self.pres_error = self.dist_to_wall - (r - self.L*np.sin(alpha))

        # Set flag to true
        self.flag_error = True

        # print('Cross-track error (in m): ', self.pres_error)
        # print('Heading (in rad): ', alpha)
        # print('r_high (in m): ', r_high)
        # print('r_low (in m): ', r_low)


    # Set PID control
    def pid_control(self):

        # PID steering angle (in radians)
        pid_steer = -self.kp*self.pres_error - self.kd*(self.pres_error - self.prev_error)

        # print('PID steering (in rad): ', pid_steer)

        # Publish PID commands for steering angle, and velocity
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = pid_steer

        msg.drive.speed = 1.5 - 0.5*( (np.abs(pid_steer)*180/np.pi) // 10) 

        print('Speed (in m/s): ', msg.drive.speed)

        self.publisher_.publish(msg)

        self.get_logger().info('Publishing steering angle and speed')

        # Update past error
        self.prev_error = self.pres_error


def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    rclpy.shutdown()

if __name__ == "__main__":
    main()