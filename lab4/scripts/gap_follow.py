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
        super().__init__('gap_follow')

        # Declare variables
        self.flag_laser = False
        self.flag_bub = False
        self.flag_gap = False

        self.laserrange = 0.0
        self.disprange = 0.0
        self.laserangle_min = 0.0
        self.laserangle_inc = 0.0

        self.ineg = 0
        self.ipos = 0
        self.nbub = 0

        self.p1 = 0

        # Declare parameters
        self.declare_parameter('L_disp',1.5)
        self.declare_parameter('L_look',1.0)
        self.declare_parameter('deltaf_max',25*np.pi/180)

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

        # Set value holder
        self.p1 = int(np.pi//self.laserangle_inc)

        # print('self.pi: ',self.p1)

        # Filter scan for inf and nan
        np.nan_to_num(self.laserrange,copy=False,nan=msg.range_max,posinf=msg.range_max,neginf=msg.range_max)

        # print('Len of laserrange: ',len(self.laserrange))

        # Index of -90 deg scan
        self.ineg = int(len(self.laserrange)//2 - self.p1//2)
        # print('self.ineg: ',self.ineg)

        # Index of +90 deg scan
        self.ipos = int(len(self.laserrange)//2 + self.p1//2)
        # print('self.ipos: ',self.ipos)

        # Collect lookahead distance parameter
        self.L_look = (self.get_parameter('L_look')).value

        # Collect disparity threshold
        self.L_disp = (self.get_parameter('L_disp')).value

        # Collect max steering
        self.deltaf_max = (self.get_parameter('deltaf_max')).value

        # Filter laserscans below lookahead to zero
        self.disprange = np.copy(self.laserrange)

        # Set number of laserscan indices to zero out in the bubble
        self.nbub = int((0.2)//(2*self.L_look*self.laserangle_inc))
        # print('self.nbub: ',self.nbub)

        # Set flag to true
        self.flag_laser = True

        # Set bubble
        self.make_bubble()


    # Bubble based on disparity
    def make_bubble(self):

        # Draw bubbles
        for i in range(self.ineg,self.ipos+1):
            
            # Check if disparity threshold is crossed
            if abs(self.laserrange[i]-self.laserrange[i+1])>self.L_disp:
                
                # Set laserscans within bubble to zero
                self.disprange[i:i+self.nbub+1] = 0
                self.disprange[i-self.nbub:i+1] = 0

                # Skip next nbub counters
                i = i+self.nbub

        # Set flag
        self.flag_bub = True

        self.set_steer()


    # Set steering
    def set_steer(self):    

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.5

        # Candidate steering angles
        # print(self.disprange[self.ineg:self.ipos+1])

        self.disprange[self.laserrange<self.L_look] = 0.0

        temp = np.nonzero(self.disprange)
        # print('temp: ', temp)
        
        temp2 = np.array(temp)[0]
        # print('temp: ', temp2)
        
        temp3 = temp2 - int(len(temp2)/2)
        # print('temp: ', temp3)

        temp4 = temp2[np.argmin(np.abs(temp3))]
        # print('temp4: ', temp4)

        temp5 = (temp4 - int(len(temp2)/2))*self.deltaf_max/(int(len(temp2)/2))
        print('Steering angle (in deg): ', temp5*180/np.pi)

        # poss_nonzero = np.array(np.nonzero(self.disprange[self.ineg:self.ipos+1]))[0] - int

        # print(poss_nonzero)

        # # Check of poss_nonzero is empty
        # if not np.any(poss_nonzero):
        #     poss_nonzero = np.array([0])

        # poss_steer = self.deltaf_max - (2*self.deltaf_max/(self.p1))*poss_nonzero 

        # print(poss_steer)

        # msg.drive.steering_angle = poss_steer[np.argmin(abs(poss_steer))]

        # self.get_logger().info('Steering set!!')

        # print('Steering angle: ', msg.drive.steering_angle)

        # self.publisher_.publish(msg)      


def main(args=None):
    rclpy.init(args=args)
    wall_follow = GapFollow()
    rclpy.spin(wall_follow)
    rclpy.shutdown()

if __name__ == "__main__":
    main()