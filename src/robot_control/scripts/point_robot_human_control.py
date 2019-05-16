#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

from geometry_msgs.msg import Point
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
	def __init__(self, dim=2, udp_ip='127.0.0.1', udp_recv_port=8026, udp_send_port=6001, buffer_size=8192*4):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port, buffer_size=buffer_size)
		if rospy.has_param('framerate'):
			self.frame_rate = rospy.get_param('framerate')
		else:
			self.frame_rate = 60.0

		self.period = rospy.Duration(1.0/self.frame_rate)
		self.running = True
		self.runningCV = threading.Condition()
		self.rate =rospy.Rate(self.frame_rate)
		self.lock = threading.Lock()


		rospy.Subscriber('joy', Joy, self.joyCB)
		self.human_control_pub = None
		self.human_robot_pose_pub = None
		self.initializePublishers()
		
		self.dim = dim
		self.num_goals = 2
		assert(self.num_goals > 0)
		self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')

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

		self.human_robot_pose = np.zeros(self.dim) #UNUSED for the time being. 
		self.human_robot_pose_msg = Point()
		self.getRobotPosition()

		self.human_goal_pose = Float32MultiArray()
		self.human_goal_pose.data = [list(x) for x in list(self.goal_positions)]
		self.human_goal_pose_pub.publish(self.human_goal_pose)

		self.data = CartVelCmd()
		self._msg_dim = [MultiArrayDimension()]
		self._msg_dim[0].label = 'cartesian_velocity'
		self._msg_dim[0].size = 2
		self._msg_dim[0].stride = 2
		self.data.velocity.layout.dim = _dim
		self.data.velocity.data = np.zeros(self.dim)
		self.data.header.stamp = rospy.Time.now()
		self.data.header.frame_id = 'human_control'

		rospy.Service("point_robot_human_control/set_human_goals", GoalPoses, self.set_human_goals)

		self.send_thread = threading.Thread(target=self._publish_command, args=(self.period,))
		self.send_thread.start()
		rospy.loginfo("END OF CONSTRUCTOR - point_robot_human_control_node")

	def initializePublishers(self):
		self.human_control_pub = rospy.Publisher('user_vel', CartVelCmd, queue_size=1)
		self.human_robot_pose_pub = rospy.Publisher('human_robot_pose', Point, queue_size=1)
		self.human_goal_pose_pub = rospy.Publisher('human_goal_pose', Float32MultiArray, queue_size=1)


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
			self.human_robot_pose_pub.publish(self.human_robot_pose_msg)
			msg_str = self.createMessageString(self.user_vel)
			self.sendStrToProcessing(msg_str)

			end = rospy.get_rostime()

			if end - start < period:
				rospy.sleep(period - (end-start))
			else:
				rospy.logwarn("Sending data took longer than the specified period")

	def set_human_goals(self, req):
		status = GoalPosesResponse()
		try:
			self.num_goals = len(req.goal_poses)
			assert(self.num_goals > 0)
			self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
			for i in range(self.num_goals):
				self.goal_positions[i][0] = req.goal_poses[i].x
				self.goal_positions[i][1] = req.goal_poses[i].y

			# print('HUMAN GOALS', self.goal_positions)
			self.human_goal_pose.data = [list(x) for x in list(self.goal_positions)]
			self.human_goal_pose_pub.publish(self.human_goal_pose)
		except:
			import IPython; IPython.embed(banner1='error in set_autonomy_goals service')
		status.success = True
		return status

	def createMessageString(self, uv):
		msg_str = "H_COMMAND"
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

	def getRobotPosition(self):
		msg_str = self.recvStrFromProcessing()
		if msg_str != "none":
			msg_str = msg_str.split(',')
			if msg_str[0] == "H_R_POSE":
				self.human_robot_pose_msg.x = float(msg_str[1])
				self.human_robot_pose_msg.y = float(msg_str[2])

	def step(self):
		self.getRobotPosition()

	def spin(self):
		rospy.loginfo("RUNNING")
		try:
			while not rospy.is_shutdown():
				self.runningCV.acquire()
				if self.running:
					self.step()
					self.rate.sleep()
				else:
					self.runningCV.wait(1.0)

				self.runningCV.release()
		except KeyboardInterrupt:
			rospy.logdebug('Keyboard interrupt, shutting down')
			rospy.core.signal_shutdown('Keyboard interrupt')


if __name__ == '__main__':
	rospy.init_node('point_robot_human_control')
	try:
		point_robot_human_control = PointRobotHumanControl()
		point_robot_human_control.spin()

	except rospy.ROSInterruptException:
		pass
