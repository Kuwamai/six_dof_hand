#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class Pose_sub:
	def __init__(self):
		self._sub = rospy.Subscriber("tracker_a", PoseStamped, self.pose_callback)

	def pose_callback(self, message):
		print(message.pose.position.x)

if __name__ == '__main__':
	rospy.init_node('pose_subscriber')
	pose_sub = Pose_sub()
	rospy.spin()
	
