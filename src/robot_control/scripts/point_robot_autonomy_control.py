#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Header
from std_msgs.msg import String

from geometry_msgs.msg import Point
from robot_control.msg import CartVelCmd

import rospy
import math
import random
import numpy as np
import threading

class PointRobotAutonomyControl(RosProcessingComm):
	def __init__(self, dim=2, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port = 6000):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port)		
		print "In constructor"
		self.period = rospy.Duration(1.0/60.0)
		self.lock = threading.Lock()
		self.rate =rospy.Rate(60)
		self.dim = dim
		self.running = True
		self.runningCV = threading.Condition()

		self.autonomy_control_pub = None
		self.autonomy_robot_pose_pub = None
		self.initializePublishers()
		if rospy.has_param('max_cart_vel'):
			self._max_cart_vel = np.array(rospy.get_param('max_cart_vel'))
		else:
			self._max_cart_vel = np.ones(self.dim)
			rospy.logwarn('No rosparam for max_cart_vel found...Defaulting to max linear velocity of 50 cm/s and max rotational velocity of 50 degrees/s')


		self.autonomy_vel = CartVelCmd()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = 2
		_dim[0].stride = 2
		self.autonomy_vel.velocity.layout.dim = _dim
		self.autonomy_vel.velocity.data = np.zeros(self.dim)
		self.autonomy_vel.header.stamp = rospy.Time.now()
		self.autonomy_vel.header.frame_id = 'autonomy_control'

		self.data = CartVelCmd()
		self._msg_dim = [MultiArrayDimension()]
		self._msg_dim[0].label = 'cartesian_velocity'
		self._msg_dim[0].size = 2
		self._msg_dim[0].stride = 2
		self.data.velocity.layout.dim = _dim
		self.data.velocity.data = np.zeros(self.dim)
		self.data.header.stamp = rospy.Time.now()
		self.data.header.frame_id = 'autonomy_control'

		self.autonomy_robot_pose = Point()
		self.getRobotPosition()

		self.send_thread = threading.Thread(target=self._publish_command, args=(self.period,))
		self.send_thread.start()

		

	def initializePublishers(self):
		self.autonomy_control_pub = rospy.Publisher('autonomy_vel', CartVelCmd, queue_size=1)
		self.autonomy_robot_pose_pub = rospy.Publisher('autonomy_robot_pose', Point, queue_size=1)

	def _publish_command(self, period):
		while not rospy.is_shutdown():
			start = rospy.get_rostime()
			self.lock.acquire()
			try:
				data = CartVelCmd()
				data.velocity.data = self.autonomy_vel.velocity.data
				data.velocity.layout.dim = self._msg_dim
				data.header.stamp = rospy.Time.now()
				data.header.frame_id = 'autonomy_control'
			finally:
				self.lock.release()

			self.autonomy_control_pub.publish(data)
			self.autonomy_robot_pose_pub.publish(self.autonomy_robot_pose)
			msg_str = self.createMessageString(self.autonomy_vel)
			self.sendStrToProcessing(msg_str)

			end = rospy.get_rostime()

			if end - start < period:
				rospy.sleep(period - (end-start))
			else:
				rospy.logwarn("Sending data took longer than the specified period")

	def createMessageString(self, av):
		msg_str = "AUTONOMY_COMMAND"
		for i in range(self.dim):
			msg_str += ","
			msg_str += str(av.velocity.data[i])

		return msg_str


	def getRobotPosition(self):
		msg_str = self.recvStrFromProcessing()
		if msg_str != "none":
			msg_str = msg_str.split(',')
			self.autonomy_robot_pose.x = float(msg_str[1])
			self.autonomy_robot_pose.y = float(msg_str[2])

	def step(self):
		self.getRobotPosition()
		for i in range(self.dim):
			self.autonomy_vel.velocity.data[i] = 0.0

		#DUse the current robot position. Use the goal positions. Compute the velocity and populate the self.autonomy_vel
		self.autonomy_vel.velocity.data[0] =  0.1
		self.autonomy_vel.velocity.data[1] = 0.0

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
	rospy.init_node('point_robot_autonomy_control')
	try:
		point_robot_autonomy_control = PointRobotAutonomyControl()
		point_robot_autonomy_control.spin()
	except rospy.ROSInterruptException:
		pass



