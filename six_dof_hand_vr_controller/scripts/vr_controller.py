#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

class Pose_pub:
	def __init__(self):
		self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
		self._sub_tri = rospy.Subscriber("trigger_r", Float64, self.trigger_callback)

		self._pub_lx = rospy.Publisher("six_dof_hand/x_linear_actuator_joint_position_controller/command", Float64, queue_size=10)
		self._pub_ly = rospy.Publisher("six_dof_hand/y_linear_actuator_joint_position_controller/command", Float64, queue_size=10)
		self._pub_lz = rospy.Publisher("six_dof_hand/z_linear_actuator_joint_position_controller/command", Float64, queue_size=10)

		self._pub_rx = rospy.Publisher("six_dof_hand/x_rotary_actuator_joint_position_controller/command", Float64, queue_size=10)
		self._pub_ry = rospy.Publisher("six_dof_hand/y_rotary_actuator_joint_position_controller/command", Float64, queue_size=10)
		self._pub_rz = rospy.Publisher("six_dof_hand/z_rotary_actuator_joint_position_controller/command", Float64, queue_size=10)

		self._pub_fr = rospy.Publisher("six_dof_hand/r_finger_joint_position_controller/command", Float64, queue_size=10)
		self._pub_fl = rospy.Publisher("six_dof_hand/l_finger_joint_position_controller/command", Float64, queue_size=10)

	def pose_callback(self, message):
		print(message.pose.position)
		quaternion = [message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w]
		e = tf.transformations.euler_from_quaternion(quaternion, axes='rxyz')
		self._pub_lx.publish(message.pose.position.x)
		self._pub_ly.publish(message.pose.position.y)
		self._pub_lz.publish(message.pose.position.z)
		self._pub_rx.publish(e[0])
		self._pub_ry.publish(e[1])
		self._pub_rz.publish(e[2])

	def trigger_callback(self, message):
		self._pub_fr.publish(-message.data)
		self._pub_fl.publish(-message.data)
		
if __name__ == '__main__':
	rospy.init_node('vr_controller')
	pose_pub = Pose_pub()
	rospy.spin()
