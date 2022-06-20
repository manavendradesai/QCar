#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from lab1.msg import scan_range

#Replace NaNs and Infs with this
minusnone = -1;

def callback(ranges):
	#Filter NaNs and Inf
	mapping = {np.nan:0,np.inf:0} 
	filteredranges = [mapping.get(x,x) for x in ranges.ranges]	
	
	#Publish max min range over two separate topics
	pub_close = rospy.Publisher('closest_point',Float64,queue_size=1)
	pub_far = rospy.Publisher('farthest_point',Float64,queue_size=1)
	pub_again = rospy.Publisher('scan_range',scan_range,queue_size=1)

	minpoint = sorted(filteredranges)[0] if minusnone not in filteredranges else sorted(filteredranges)[1] 
	maxpoint = max(filteredranges)
	print('Furthest point: ', maxpoint, 'Closest point: ', minpoint)

	pub_close.publish(minpoint)
	pub_far.publish(maxpoint)

	#Publish laserscan header, min and maxpoint through a message on another topic
	pub_again.publish(ranges.header,minpoint,maxpoint)

def listener():
	rospy.init_node('listenlaserscan',anonymous=True)
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
