#!/usr/bin/env python
import rospy
from ros_processing_bridge.ros_processing_bridge import RosProcessingComm

import numpy as np
import random
import math
import threading
from geometry_msgs.msg import Vector3
from robot_control.srv import GoalPoses, GoalPosesResponse, GoalPosesRequest
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

npa = np.array

class SetGoalsRobot(RosProcessingComm):
	def __init__(self, dim=2, num_goals=2, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port = 6000):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port)
		self.dim = dim
		self.num_goals = num_goals
		self.max_num_goals = 5


		self.autonomy_goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
		self.autonomy_robot_position = npa([0]*self.dim, dtype='f')

		self.human_goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
		self.human_robot_position = npa([0]*self.dim, dtype='f')

		if rospy.has_param('framerate'):
			self.frame_rate = rospy.get_param('framerate')
		else:
			self.frame_rate = 60.0

		self.lock = threading.Lock()
		self.rate = rospy.Rate(self.frame_rate)
		self.running = True
		self.runningCV = threading.Condition()
		if rospy.has_param('width'):
			self.width = rospy.get_param('width')
		else:
			self.width = 1200
		self.x_offset = 20

		if rospy.has_param('height'):
			self.height = rospy.get_param('height')
		else:
			self.height = 900
		self.y_offset = 20

		self.init_autonomy_goal_positions()
		self.init_autonomy_robot_position()
		self.init_human_goal_positions()
		self.init_human_robot_position()


		self.set_autonomy_goals_service = rospy.ServiceProxy('/point_robot_autonomy_control/set_autonomy_goals', GoalPoses)
		self.set_human_goals_service = rospy.ServiceProxy('/point_robot_human_control/set_human_goals', GoalPoses)


		rospy.Service("setgoalsrobot/send_autonomy_goals_to_processing", Trigger, self.send_autonomy_goals_to_processing)
		rospy.Service("setgoalsrobot/send_autonomy_robot_pose_to_processing", Trigger, self.send_autonomy_robot_pose_to_processing)
		rospy.Service("setgoalsrobot/send_human_goals_to_processing", Trigger, self.send_human_goals_to_processing)
		rospy.Service("setgoalsrobot/send_human_robot_pose_to_processing", Trigger, self.send_human_robot_pose_to_processing)
		rospy.Service("setgoalsrobot/reset_autonomy_goals", Trigger, self.reset_autonomy_goals)
		rospy.Service("setgoalsrobot/reset_human_goals", Trigger, self.reset_human_goals)
		rospy.Service("setgoalsrobot/reset_num_goals", Trigger, self.reset_num_goals)
		rospy.Service("setgoalsrobot/reset_human_robot", Trigger, self.reset_human_robot)
		rospy.Service("setgoalsrobot/reset_autonomy_robot", Trigger, self.reset_autonomy_robot)

		rospy.loginfo("END OF CONSTRUCTOR - set_goals_robot_node")


	#autonomy robot pose and autonomy goals poses
	def send_autonomy_goals_to_processing(self, req):
		status = TriggerResponse()
		#send goals to autonomy_node
		goals_req = GoalPosesRequest()
		for i in range(self.num_goals):
			goal_pose = Vector3()
			goal_pose.x = self.autonomy_goal_positions[i][0]
			goal_pose.y = self.autonomy_goal_positions[i][1]
			goals_req.goal_poses.append(goal_pose)
		status_2 = self.set_autonomy_goals_service(goals_req)
		msg_string = self.createMsgString('autonomy_goal_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True and status_2.success
		return status

	def send_autonomy_robot_pose_to_processing(self, req):
		status = TriggerResponse()
		msg_string = self.createMsgString('autonomy_robot_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True
		return status

	#human robot pose and human goals poses
	def send_human_goals_to_processing(self, req):
		status = TriggerResponse()
		goals_req = GoalPosesRequest()
		for i in range(self.num_goals):
			goal_pose = Vector3()
			goal_pose.x = self.human_goal_positions[i][0]
			goal_pose.y = self.human_goal_positions[i][1]
			goals_req.goal_poses.append(goal_pose)
		status_2 = self.set_human_goals_service(goals_req)
		msg_string = self.createMsgString('human_goal_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True and status_2.success
		return status

	def send_human_robot_pose_to_processing(self, req):
		status = TriggerResponse()
		msg_string = self.createMsgString('human_robot_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True
		return status

	#Reset robot poses.
	def reset_autonomy_robot(self, req):
		status = TriggerResponse()
		self.createNewAutonomyRobotPosition()
		status.success = True
		return status

	def reset_human_robot(self, req):
		status = TriggerResponse()
		self.createNewHumanRobotPosition()
		status.success = True
		return status

	#Reset goals
	def reset_autonomy_goals(self, req):
		status = TriggerResponse()
		self.createNewAutonomyGoalPositions()
		status.success = True
		return status

	def reset_human_goals(self, req):
		status = TriggerResponse()
		self.createNewHumanGoalPositions()
		status.success = True
		return status

	def reset_num_goals(self, req):
		status = TriggerResponse()
		self.num_goals = np.random.choice(range(2, self.max_num_goals))
		self.autonomy_goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
		self.human_goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')
		status.success = True
		return status

	def createNewAutonomyRobotPosition(self):
		self.autonomy_robot_position = [3*self.x_offset + np.random.random()*(self.width/2.0 - 6*self.x_offset),  3*self.y_offset + np.random.random()*(self.height - 6*self.y_offset)]

	def createNewHumanRobotPosition(self):
		self.human_robot_position = [3*self.x_offset + np.random.random()*(self.width/2.0 - 6*self.x_offset),  3*self.y_offset + np.random.random()*(self.height - 6*self.y_offset)]


	def createNewAutonomyGoalPositions(self):
		for i in range(self.num_goals):
			self.autonomy_goal_positions[i] = [self.x_offset + np.random.random()*(self.width/2.0 - 2*self.x_offset),  self.y_offset + np.random.random()*(self.height - 2*self.y_offset)]

	def createNewHumanGoalPositions(self):
		for i in range(self.num_goals):
			self.human_goal_positions[i] = [self.x_offset + np.random.random()*(self.width/2.0 - 2*self.x_offset) + self.width/2.0, self.y_offset + np.random.random()*(self.height - 2*self.y_offset)]

	#Init functions
	def init_autonomy_robot_position(self):
		self.autonomy_robot_position[0] = self.width/4.0
		self.autonomy_robot_position[1] = self.height/2.0

	def init_autonomy_goal_positions(self):
		if self.num_goals == 2:
			self.autonomy_goal_positions[0][0] = self.width/8.0
			self.autonomy_goal_positions[0][1] = self.height/4.0
			self.autonomy_goal_positions[1][0] = 3.0*self.width/8.0
			self.autonomy_goal_positions[1][1] = self.height/4.0

	def init_human_robot_position(self):
		self.human_robot_position[0] = self.width/4.0
		self.human_robot_position[1] = self.height/2.0

	def init_human_goal_positions(self):
		if self.num_goals == 2:
			self.human_goal_positions[0][0] = self.width/8.0 + self.width/2.0
			self.human_goal_positions[0][1] = self.height/4.0
			self.human_goal_positions[1][0] = 3.0*self.width/8.0 + self.width/2.0
			self.human_goal_positions[1][1] = self.height/4.0

	def createMsgString(self, msg_type):
		#all of the following are outgoing messages to processing
		if msg_type == 'autonomy_goal_pose':
			msg_string = "AUTONOMY_GOALPOS"
			msg_string += ","+str(self.num_goals)
			for i in range(self.num_goals):
				for j in range(self.dim):
					msg_string += ","+str(self.autonomy_goal_positions[i][j])

		elif msg_type == 'autonomy_robot_pose':
			msg_string = "AUTONOMY_ROBOTPOS"
			for i in range(self.dim):
				msg_string += ","+str(self.autonomy_robot_position[i])

		elif msg_type == "human_goal_pose":
			msg_string = "HUMAN_GOALPOS"
			msg_string += ","+str(self.num_goals)
			for i in range(self.num_goals):
				for j in range(self.dim):
					msg_string += ","+str(self.human_goal_positions[i][j])

		elif msg_type == "human_robot_pose":
			msg_string = "HUMAN_ROBOTPOS"
			for i in range(self.dim):
				msg_string += ","+str(self.human_robot_position[i])

		return msg_string


if __name__ == '__main__':
	rospy.init_node('set_goals_and_robot')
	rospy.loginfo("In set goals_and_robot node")

	try:
		set_goals_robot_node = SetGoalsRobot()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
