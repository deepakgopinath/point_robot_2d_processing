#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Header
from std_msgs.msg import String
from robot_control.srv import GoalPoses, GoalPosesResponse, GoalPosesRequest

from robot_control.msg import CartVelCmd

import rospy
import math
import random
import numpy as np
import threading

npa = np.array

class PointRobotHumanControl(RosProcessingComm):
	def __init__(self, dim=2, udp_ip='127.0.0.1', udp_recv_port=8025, udp_send_port=6001):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port)
		if rospy.has_param('framerate'):
			self.frame_rate = rospy.get_param('framerate')
		else:
			self.frame_rate = 60.0
		
		self.period = rospy.Duration(1.0/self.frame_rate)
		self.lock = threading.Lock()
		rospy.Subscriber('joy', Joy, self.joyCB)
		self.human_control_pub = rospy.Publisher('user_vel', CartVelCmd, queue_size=1)
		self.dim = dim
		if rospy.has_param('max_cart_vel'):
			self._max_cart_vel = np.array(rospy.get_param('max_cart_vel'))
		else:
			self._max_cart_vel = np.ones(self.dim)
			rospy.logwarn('No rosparam for max_cart_vel found...Defaulting to max linear velocity of 50 cm/s and max rotational velocity of 50 degrees/s')

		if rospy.has_param('width'):
			self.width = rospy.get_param('width')
		else:
			self.width = 1200

		if rospy.has_param('height'):
			self.height = rospy.get_param('height')
		else:
			self.height = 900 

		self.user_vel = CartVelCmd()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = 2
		_dim[0].stride = 2
		self.user_vel.velocity.layout.dim = _dim
		self.user_vel.velocity.data = np.zeros(self.dim)
		self.user_vel.header.stamp = rospy.Time.now()
		self.user_vel.header.frame_id = 'human_control'

		self.data = CartVelCmd()
		self._msg_dim = [MultiArrayDimension()]
		self._msg_dim[0].label = 'cartesian_velocity'
		self._msg_dim[0].size = 2
		self._msg_dim[0].stride = 2
		self.data.velocity.layout.dim = _dim
		self.data.velocity.data = np.zeros(self.dim)
		self.data.header.stamp = rospy.Time.now()
		self.data.header.frame_id = 'human_control'


		rospy.loginfo("Waiting for set_goals_node - set goals node ")
		rospy.wait_for_service("/setgoals/goal_poses_list")
		rospy.loginfo("set_goals_node found - set goals node!")

		self.set_goals_service = rospy.ServiceProxy("/setgoals/goal_poses_list", GoalPoses)
		
		self.gp_req = GoalPosesRequest()
		goal_poses_response = self.set_goals_service(self.gp_req)
		print goal_poses_response.goal_poses
		self.num_goals = len(goal_poses_response.goal_poses)
		assert(self.num_goals > 0)

		self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
		for i in range(self.num_goals):
			self.goal_positions[i][0] = goal_poses_response.goal_poses[i].x + self.width/2.0 #This is specific to the way the processing sketch is setup. The human control is on the right half of the window. therefore thye width/2.0 bias
			self.goal_positions[i][1] = goal_poses_response.goal_poses[i].y

		self.send_thread = threading.Thread(target=self._publish_command, args=(self.period,))
		self.send_thread.start()
		# while not rospy.is_shutdown():
		# 	print " "

	def _publish_command(self, period):
		while not rospy.is_shutdown():
			start = rospy.get_rostime()
			self.lock.acquire()
			try:
				data = CartVelCmd()
				data.velocity.data = self.user_vel.velocity.data
				data.velocity.layout.dim = self._msg_dim
				data.header.stamp = rospy.Time.now()
				data.header.frame_id = 'human_control'
			finally:
				self.lock.release()

			self.human_control_pub.publish(data)
			msg_str = self.createMessageString(self.user_vel)
			self.sendStrToProcessing(msg_str)


			end = rospy.get_rostime()

			if end - start < period:
				rospy.sleep(period - (end-start))
			else:
				rospy.logwarn("Sending data took longer than the specified period")



	def createMessageString(self, uv):
		msg_str = "HUMAN_COMMAND"
		for i in range(self.dim):
			msg_str += ","
			msg_str += str(uv.velocity.data[i])

		return msg_str

	
	def joyCB(self, msg):
		_axes = np.array(msg.axes)
		for i in range(self.dim):
			self.user_vel.velocity.data[i] = 0.0

		self.user_vel.velocity.data[0] = -_axes[0] * self._max_cart_vel[0]# + random.random()
		self.user_vel.velocity.data[1] = -_axes[1] * self._max_cart_vel[1]
		self.user_vel.header.stamp = rospy.Time.now()

	


if __name__ == '__main__':
	rospy.init_node('point_robot_human_control')
	try:
		point_robot_human_control = PointRobotHumanControl()

	except rospy.ROSInterruptException:
		pass



