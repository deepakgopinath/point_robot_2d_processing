#!/usr/bin/env python
import rospy
from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import threading

class ProcessKeyStrokes(RosProcessingComm):
    def __init__(self, udp_ip="127.0.0.1", udp_recv_port=8026, udp_send_port = 6000):
        RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port)
        if rospy.has_param('framerate'):
            self.frame_rate = rospy.get_param('framerate')
        else:
            self.frame_rate = 60.0

        self.lock = threading.Lock()
        self.rate = rospy.Rate(self.frame_rate)
        self.running = True
        self.runningCV = threading.Condition()
        #GOal positions
        rospy.loginfo("Waiting for set_goals_node - process_key_strokes node ")
        rospy.wait_for_service("/setgoals/send_goals_to_processing")
        rospy.wait_for_service("/setgoals/reset_goals")
        rospy.loginfo("set_goals_node found - process_key_strokes node!")

        rospy.loginfo("Waiting for point_robot_autonomy_control - process_key_strokes node ")
        rospy.wait_for_service("/point_robot_autonomy_control/trigger_trial")
        rospy.loginfo("point_robot_autonomy_control found - process_key_strokes node!")


        self.send_goals_to_processing_service = rospy.ServiceProxy('/setgoals/send_goals_to_processing', Trigger)
        self.reset_goals = rospy.ServiceProxy('/setgoals/reset_goals', Trigger)
        self.autonomy_node_trigger_trial = rospy.ServiceProxy('/point_robot_autonomy_control/trigger_trial', SetBool)
        # self.autonomy_node_retrieve_goals = rospy.ServiceProxy('/point_robot_autonomy_control/retrieve_goals', Trigger)

    def step(self):
        msg_str = self.recvStrFromProcessing()
        if msg_str != "none":
            msg_str = msg_str.split(',')
            print msg_str
            if msg_str[0] == 'GOALS_READY':
                self.send_goals_to_processing_service()
            elif msg_str[0] == 'GOALS_RESET':
                self.reset_goals()
                self.send_goals_to_processing_service()
            elif msg_str[0] == 'BEGIN_TRIAL':
                trigger_msg = SetBoolRequest()
                trigger_msg.data = True
                self.autonomy_node_trigger_trial(trigger_msg)
            elif msg_str[0] == 'END_TRIAL':
                trigger_msg = SetBoolRequest()
                trigger_msg.data = False
                self.autonomy_node_trigger_trial(trigger_msg)

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
    rospy.init_node('process_key_strokes')
    try:
        process_key_strokes = ProcessKeyStrokes()
        process_key_strokes.spin()
    except rospy.ROSInterruptException:
        pass
