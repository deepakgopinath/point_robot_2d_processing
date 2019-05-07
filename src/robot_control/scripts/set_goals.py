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

class SetGoals(RosProcessingComm):
	def __init__(self, dim=2, num_goals=2, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port = 6000):
		RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port)
		self.dim = dim
		self.num_goals = num_goals
		self.goal_positions = npa([[0]*self.dim]*self.num_goals, dtype= 'f')

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

		self.init_goal_positions()

		rospy.Service("setgoals/goal_poses_list", GoalPoses, self.goal_poses_list)
		rospy.Service("setgoals/send_goals_to_processing", Trigger, self.send_goals_to_processing)
		rospy.Service("setgoals/reset_goals", Trigger, self.reset_goals)

	def send_goals_to_processing(self, req):
		status = TriggerResponse()
		msg_string = self.createMsgString()
		self.sendStrToProcessing(msg_string)
		status.success = True
		return status

	def reset_goals(self, req):
		status = TriggerResponse()
		self.createNewGoalPositions()
		status.success = True
		return status

	def createNewGoalPositions(self):
		pass


	def createMsgString(self):
		msg_string = "GOALPOS"
		msg_string += ","+str(self.num_goals)
		for i in range(self.num_goals):
			for j in range(self.dim):
				msg_string += ","+str(self.goal_positions[i][j])

		return msg_string

	def init_goal_positions(self):
		if self.num_goals == 2:
			self.goal_positions[0][0] = self.width/8.0
			self.goal_positions[0][1] = self.height/4.0
			self.goal_positions[1][0] = 3.0*self.width/8.0
			self.goal_positions[1][1] = self.height/4.0

	def goal_poses_list(self, objs):
		goal_poses_response = GoalPosesResponse()
		for i in range(self.num_goals):
			goal_pose = Vector3()
			goal_pose.x = self.goal_positions[i][0]
			goal_pose.y = self.goal_positions[i][1]
			goal_poses_response.goal_poses.append(goal_pose)

		goal_poses_response.status = True
		return goal_poses_response

if __name__ == '__main__':
	rospy.init_node('set_goals')
	rospy.loginfo("In set goals node")

	try:
		set_goals_node = SetGoals()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
