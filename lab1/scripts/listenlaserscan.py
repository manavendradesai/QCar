#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

#Replace NaNs and Infs with this
minusnone = -1;

def callback(ranges):
	#Filter NaNs and Inf
	mapping = {np.nan:0,np.inf:0} 
	filteredranges = [mapping.get(x,x) for x in ranges.ranges]	
	#rospy.loginfo(rospy.get_caller_id() + "Furthest point %s",max(filteredranges))
	
	#Publish max min range over two separate topics
	pub_close = rospy.Publisher('closest_point',Float64,queue_size=10)
	pub_far = rospy.Publisher('farthest_point',Float64,queue_size=1)

	minpoint = sorted(filteredranges)[0] if minusnone not in filteredranges else sorted(filteredranges)[1] 
	maxpoint = max(filteredranges) 
 	#rospy.loginfo("Closest point: %s",minpoint)
	#rospy.loginfo("Furthest point: %s", maxpoint)	
	print('Furthest point: ', maxpoint, 'Closest point: ', minpoint)
	pub_close.publish(minpoint)
	pub_far.publish(maxpoint)


def listener():
	rospy.init_node('listenlaserscan',anonymous=True)
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
