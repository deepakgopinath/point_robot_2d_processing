#!/usr/bin/env python

from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import rospy
import math
import random

class PointRobotHumanControl(RosProcessingComm):
	def __init__(self):
		print "In constructor"

if __name__ == '__main__':
	rospy.init_node('point_robot_human_control')
	try:
		point_robot_human_control = PointRobotHumanControl()
	except rospy.ROSInterruptException:
		pass



