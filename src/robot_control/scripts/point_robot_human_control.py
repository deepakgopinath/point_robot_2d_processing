#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Header
from std_msgs.msg import String

from point_robot_human_control.msg import CartVelCmd

import rospy
import math
import random
import numpy as np

class PointRobotHumanControl(RosProcessingComm):
	def __init__(self):
		print "In constructor"
		rospy.Subscriber('joy', Joy, self.joyCB)
		rospy.Publisher('user_vel', Float32MultiArray, queue_size=1)
		self.dim = 2
		if rospy.has_param('max_cart_vel'):
			self._max_cart_vel = np.array(rospy.get_param('max_cart_vel'))
		else:
			self._max_cart_vel = np.ones(self.dim)
			rospy.logwarn('No rosparam for max_cart_vel found...Defaulting to max linear velocity of 50 cm/s and max rotational velocity of 50 degrees/s')


		self._cart_vel = np.zeros(self.dim)
		self.user_vel = CartVelCmd()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = 2
		_dim[0].stride = 2
		self.user_vel.velocity.layout.dim = _dim
		self.user_vel.velocity.data = np.zeros_like(self._cart_vel)
		self.user_vel.header.stamp = rospy.Time.now()
		self.user_vel.header.frame_id = 'joy1axis'
	
	def joyCB(self, msg):
		_axes = np(msg.axes)
		self.user_vel.velocity.data[0] = _axes[0] * self._max_cart_vel[0]
		self.user_vel.velocity.data[1] = _axes[1] * self._max_cart_vel[1]
		self.user_vel.header.stamp = rospy.Time.now()

if __name__ == '__main__':
	rospy.init_node('point_robot_human_control')
	try:
		point_robot_human_control = PointRobotHumanControl()

	except rospy.ROSInterruptException:
		pass



