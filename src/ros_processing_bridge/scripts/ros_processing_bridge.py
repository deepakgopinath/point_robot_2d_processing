#!/usr/bin/env python


import socket
import sys
import errno
import time
import rospy

class RosProcessingCom(object):
	def __init__(self, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port=6000):
		pass

	def sendStrToProcessing(self, msg_str):
		pass

	def recvStrFromProcessing(self, msg_str):
		pass