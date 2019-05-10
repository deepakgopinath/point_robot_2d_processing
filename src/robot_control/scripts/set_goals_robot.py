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

		if rospy.has_param('height'):
			self.height = rospy.get_param('height')
		else:
			self.height = 900

		self.init_autonomy_goal_positions()
		self.init_autonomy_robot_position()
		self.init_human_goal_positions()
		self.init_human_robot_position()

		rospy.Service("setgoalsrobot/autonomy_goal_poses_list", GoalPoses, self.autonomy_goal_poses_list)
		rospy.Service("setgoalsrobot/human_goal_poses_list", GoalPoses, self.human_goal_poses_list)
		rospy.Service("setgoalsrobot/send_autonomy_goals_to_processing", Trigger, self.send_autonomy_goals_to_processing)
		rospy.Service("setgoalsrobot/send_autonomy_robot_pose_to_processing", Trigger, self.send_autonomy_robot_pose_to_processing)
		rospy.Service("setgoalsrobot/send_human_goals_to_processing", Trigger, self.send_human_goals_to_processing)
		rospy.Service("setgoalsrobot/send_human_robot_pose_to_processing", Trigger, self.send_human_robot_pose_to_processing)
		rospy.Service("setgoalsrobot/reset_autonomy_goals", Trigger, self.reset_autonomy_goals)
		rospy.Service("setgoalsrobot/reset_human_goals", Trigger, self.reset_human_goals)


	#autonomy robot pose and autonomy goals poses
	def send_autonomy_goals_to_processing(self, req):
		status = TriggerResponse()
		msg_string = self.createMsgString('autonomy_goal_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True
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
		msg_string = self.createMsgString('human_goal_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True
		return status

	def send_human_robot_pose_to_processing(self, req):
		status = TriggerResponse()
		msg_string = self.createMsgString('human_robot_pose')
		self.sendStrToProcessing(msg_string)
		status.success = True
		return status

	def autonomy_goal_poses_list(self, objs):
		goal_poses_response = GoalPosesResponse()
		for i in range(self.num_goals):
			goal_pose = Vector3()
			goal_pose.x = self.autonomy_goal_positions[i][0]
			goal_pose.y = self.autonomy_goal_positions[i][1]
			goal_poses_response.goal_poses.append(goal_pose)

		goal_poses_response.status = True
		return goal_poses_response

	def human_goal_poses_list(self, objs):
		goal_poses_response = GoalPosesResponse()
		for i in range(self.num_goals):
			goal_pose = Vector3()
			goal_pose.x = self.human_goal_positions[i][0]
			goal_pose.y = self.human_goal_positions[i][1]
			goal_poses_response.goal_poses.append(goal_pose)

		goal_poses_response.status = True
		return goal_poses_response

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

	def createNewAutonomyGoalPositions(self):
		if self.num_goals == 2:
			self.autonomy_goal_positions[0][0] = 0.0 + np.random.random()*self.width/2.0
			self.autonomy_goal_positions[0][1] = 0.0 + np.random.random()*self.height
			self.autonomy_goal_positions[1][0] = 0.0 + np.random.random()*self.width/2.0
			self.autonomy_goal_positions[1][1] = 0.0 + np.random.random()*self.height

	def createNewHumanGoalPositions(self):
		if self.num_goals == 2:
			self.human_goal_positions[0][0] = 0.0 + np.random.random()*self.width/2.0 + self.width/2.0
			self.human_goal_positions[0][1] = 0.0 + np.random.random()*self.height
			self.human_goal_positions[1][0] = 0.0 + np.random.random()*self.width/2.0 + self.width/2.0
			self.human_goal_positions[1][1] = 0.0 + np.random.random()*self.height


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
