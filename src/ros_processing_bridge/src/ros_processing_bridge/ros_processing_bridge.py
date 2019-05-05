#!/usr/bin/env python
import socket
import sys
import errno
import time
import rospy

class RosProcessingComm(object):
	def __init__(self, udp_ip="127.0.0.1", udp_recv_port=8025, udp_send_port=6000):
		
		self.rate = 40.0
		self.udp_ip = udp_ip
		self.udp_send_port = udp_send_port
		self.udp_recv_port = udp_recv_port
		
		self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		self.sock_recv.bind((self.udp_ip, self.udp_recv_port))
		self.sock_recv.setblocking(0)

		rospy.loginfo("Connected")

	def sendStrToProcessing(self, msg_str):
		self.sock_send.sendto(msg_str.encode('utf-8'), (self.udp_ip, self.udp_send_port))


	def recvStrFromProcessing(self):
		msg_string = "none"
		try:
			msg = self.sock_recv.recv(4096)
		except socket.error as e:
			err = e.args[0]
			if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
				pass
			else:
				print (e)
				sys.exit(1)
		else:
			msg_string = msg
			# rospy.logfatal("Received message from PROCESSING APP")

		return msg_string