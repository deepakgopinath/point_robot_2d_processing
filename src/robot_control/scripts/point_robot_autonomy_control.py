#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from std_msgs.msg import Header
from std_msgs.msg import String
from robot_control.srv import GoalPoses, GoalPosesResponse, GoalPosesRequest
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from geometry_msgs.msg import Point
from robot_control.msg import CartVelCmd
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from robot_control.cfg import transparency_paramsConfig as ConfigType
import rospy
import math
import random
import numpy as np
import threading

npa=np.array

class PointRobotAutonomyControl(RosProcessingComm):
	def __init__(self, intended_goal_index = 0, dim=2, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port = 6000, buffer_size=8192*4):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port, buffer_size=buffer_size)
		print "In constructor"

		if rospy.has_param('framerate'):
			self.frame_rate = rospy.get_param('framerate')
		else:
			self.frame_rate = 60.0

		# self.frame_rate = 60.0

		# self.server = DynamicReconfigureServer(ConfigType, self.reconfigureParams)



		self.is_trial_on = False
		self.period = rospy.Duration(1.0/self.frame_rate)
		self.lock = threading.Lock()
		self.rate =rospy.Rate(self.frame_rate)
		self.dim = dim
		self.running = True
		self.runningCV = threading.Condition()
		self.num_goals = 2
		self.goal_threshold = 10

		self.signal_sparsity = 0.0
		self.random_direction = 0.9
		self.rand_vec_scale = 1.0
		self.mu = [0]*self.dim
		self.cov = np.eye(self.dim)

		assert(self.num_goals > 0)
		self.intended_goal_index = intended_goal_index
		assert(self.intended_goal_index < self.num_goals)
		self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')

		self.autonomy_control_pub = None
		self.autonomy_robot_pose_pub = None
		self.autonomy_goal_pose_pub = None

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

		self.velocity_scale = 0.25

		self.autonomy_robot_pose = np.zeros(self.dim)
		self.autonomy_robot_pose_msg = Point()
		self.getRobotPosition()

		self.autonomy_goal_pose_msg = Float32MultiArray()
		_dim = [MultiArrayDimension()]*2
		_dim[0].label = 'Goal Number'
		_dim[0].size = self.num_goals
		_dim[0].stride = self.num_goals*self.dim
		_dim[1].label = 'Dimension'
		_dim[1].size = self.dim
		_dim[1].stride = self.dim
		self.autonomy_goal_pose_msg.layout.dim = _dim
		self.autonomy_goal_pose_msg.data = [list(x) for x in list(self.goal_positions)]
		self.autonomy_goal_pose_pub.publish(self.autonomy_goal_pose_msg)

		self.data = CartVelCmd()
		self._msg_dim = [MultiArrayDimension()]
		self._msg_dim[0].label = 'cartesian_velocity'
		self._msg_dim[0].size = 2
		self._msg_dim[0].stride = 2
		self.data.velocity.layout.dim = _dim
		self.data.velocity.data = np.zeros(self.dim)
		self.data.header.stamp = rospy.Time.now()
		self.data.header.frame_id = 'autonomy_control'

		self.filter_length = 10;
		self.filter_list = [[0]*self.dim] * self.filter_length

		rospy.Service("point_robot_autonomy_control/set_autonomy_goals", GoalPoses, self.set_autonomy_goals)
		rospy.Service("point_robot_autonomy_control/trigger_trial", SetBool, self.trigger_trial)

		self.send_thread = threading.Thread(target=self._publish_command, args=(self.period,))
		self.send_thread.start()
		rospy.loginfo("END OF CONSTRUCTOR - point_robot_autonomy_control_node")


	def reconfigureParams(self, level, config):
		print "IN CONFIG"
		self.signal_sparsity = config["signal_sparsity"]
		self.random_direction = config["random_direction"]
		print self.signal_sparsity, self.random_direction
		return config

	def set_autonomy_goals(self, req):
		status = GoalPosesResponse()
		try:
			self.num_goals = len(req.goal_poses)
			assert(self.num_goals > 0)
			self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
			for i in range(self.num_goals):
				self.goal_positions[i][0] = req.goal_poses[i].x
				self.goal_positions[i][1] = req.goal_poses[i].y

			# print('AUTONOMY_GOALS', self.goal_positions)
			self.autonomy_goal_pose_msg.data = [list(x) for x in list(self.goal_positions)]
			self.autonomy_goal_pose_pub.publish(self.autonomy_goal_pose_msg)
		except:
			import IPython; IPython.embed(banner1='error in set_autonomy_goals service')
		status.success = True
		return status

	def trigger_trial(self, req):
		status = SetBoolResponse()
		self.is_trial_on = req.data
		status.success = True
		return status

	def initializePublishers(self):
		self.autonomy_control_pub = rospy.Publisher('autonomy_vel', CartVelCmd, queue_size=1)
		self.autonomy_robot_pose_pub = rospy.Publisher('autonomy_robot_pose', Point, queue_size=1)
		self.autonomy_goal_pose_pub = rospy.Publisher('autonomy_goal_pose',Float32MultiArray,queue_size=1)

	def _publish_command(self, period):
		while not rospy.is_shutdown():
			if self.is_trial_on:
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
				self.autonomy_robot_pose_pub.publish(self.autonomy_robot_pose_msg)
				msg_str = self.createMessageString(self.autonomy_vel)
				self.sendStrToProcessing(msg_str)

				end = rospy.get_rostime()

				if end - start < period:
					rospy.sleep(period - (end-start))
				else:
					rospy.logwarn("Sending data took longer than the specified period")

	def createMessageString(self, av):
		msg_str = "A_COMMAND"
		for i in range(self.dim):
			msg_str += ","
			msg_str += str(av.velocity.data[i])

		return msg_str

	def getRobotPosition(self):
		msg_str = self.recvStrFromProcessing()
		if msg_str != "none":
			msg_str = msg_str.split(',')
			if msg_str[0] == "A_R_POSE":
				self.autonomy_robot_pose[0] = float(msg_str[1])
				self.autonomy_robot_pose[1] = float(msg_str[2])
				self.autonomy_robot_pose_msg.x = float(msg_str[1])
				self.autonomy_robot_pose_msg.y = float(msg_str[2])

	def step(self):
		self.getRobotPosition()
		for i in range(self.dim):
			self.autonomy_vel.velocity.data[i] = 0.0

		#compute base velocity
		# if np.random.random() < 0.1:
		# 	if self.intended_goal_index == 0:
		# 		self.intended_goal_index = 1
		# 	else:
		# 		self.intended_goal_index = 0

		if np.linalg.norm(self.goal_positions[self.intended_goal_index] - self.autonomy_robot_pose) > self.goal_threshold: #generate nonzero velocity if the robot is outisde the goal threshold distance.
			for i in range(self.dim):
				self.autonomy_vel.velocity.data[i] = self.velocity_scale*np.sign(self.goal_positions[self.intended_goal_index][i] - self.autonomy_robot_pose[i])

		#LOW PASS FILTER?

		self.filter_list.pop(0)
		self.filter_list.append(list(self.autonomy_vel.velocity.data[:self.dim]))
		self.autonomy_vel.velocity.data[:self.dim] = list(np.mean(self.filter_list, axis = 0))

		#TODO add gaussian noise to velocity
		rand_vector = np.random.multivariate_normal(self.mu, self.cov)
		rand_vector = self.rand_vec_scale*rand_vector/np.linalg.norm(rand_vector)
		self.autonomy_vel.velocity.data[:self.dim] = (1 - self.random_direction)*(self.autonomy_vel.velocity.data[:self.dim]) + (self.random_direction)*rand_vector

		rand = np.random.random()
		if rand < self.signal_sparsity:
			for i in range(self.dim):
				self.autonomy_vel.velocity.data[i] = 0.0

		# self.intended_goal_index = 0

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
