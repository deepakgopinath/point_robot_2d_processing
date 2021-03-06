#!/usr/bin/env python
import rospy
from ros_processing_bridge.ros_processing_bridge import RosProcessingComm
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import String
import threading

class ProcessKeyStrokes(RosProcessingComm):
    def __init__(self, udp_ip="127.0.0.1", udp_recv_port=8027, udp_send_port = 6002, buffer_size=4096):
        RosProcessingComm.__init__(self, udp_ip=udp_ip, udp_recv_port=udp_recv_port, udp_send_port=udp_send_port, buffer_size=buffer_size)
        if rospy.has_param('framerate'):
            self.frame_rate = rospy.get_param('framerate')
        else:
            self.frame_rate = 60.0

        self.lock = threading.Lock()
        self.rate = rospy.Rate(self.frame_rate)
        self.running = True
        self.runningCV = threading.Condition()
        #GOal positions
        rospy.loginfo("Waiting for set_goals_robot_node - process_key_strokes node ")
        rospy.wait_for_service("/setgoalsrobot/send_autonomy_goals_to_processing")
        rospy.wait_for_service("/setgoalsrobot/send_human_goals_to_processing")
        rospy.wait_for_service("/setgoalsrobot/reset_autonomy_goals")
        rospy.wait_for_service("/setgoalsrobot/reset_human_goals")
        rospy.loginfo("set_goals_robot_node found - process_key_strokes node!")

        rospy.loginfo("Waiting for point_robot_autonomy_control - process_key_strokes node ")
        rospy.wait_for_service("/point_robot_autonomy_control/trigger_trial")
        rospy.loginfo("point_robot_autonomy_control found - process_key_strokes node!")


        self.send_autonomy_goals_to_processing_service = rospy.ServiceProxy('/setgoalsrobot/send_autonomy_goals_to_processing', Trigger)
        self.send_human_goals_to_processing_service = rospy.ServiceProxy('/setgoalsrobot/send_human_goals_to_processing', Trigger)
        self.send_autonomy_robot_pose_to_processing_service = rospy.ServiceProxy('/setgoalsrobot/send_autonomy_robot_pose_to_processing', Trigger)
        self.send_human_robot_pose_to_processing_service = rospy.ServiceProxy('/setgoalsrobot/send_human_robot_pose_to_processing', Trigger)
        self.reset_autonomy_goals = rospy.ServiceProxy('/setgoalsrobot/reset_autonomy_goals', Trigger)
        self.reset_human_goals = rospy.ServiceProxy('/setgoalsrobot/reset_human_goals', Trigger)
        self.reset_num_goals = rospy.ServiceProxy('/setgoalsrobot/reset_num_goals', Trigger)
        self.reset_autonomy_robot = rospy.ServiceProxy('/setgoalsrobot/reset_autonomy_robot', Trigger)
        self.reset_human_robot = rospy.ServiceProxy('/setgoalsrobot/reset_human_robot', Trigger)
        self.autonomy_node_trigger_trial = rospy.ServiceProxy('/point_robot_autonomy_control/trigger_trial', SetBool)

        self.begin_trial_pub = None
        self.end_trial_pub = None
        self.goals_reset_pub = None
        self.session_init_pub = None
        self.initializePublishers()

        rospy.loginfo("END OF CONSTRUCTOR - process_key_strokes_node")


    def initializePublishers(self):
        self.begin_trial_pub = rospy.Publisher('begin_trial', String, queue_size=1)
        self.end_trial_pub = rospy.Publisher('end_trial', String, queue_size=1)
        self.goals_reset_pub = rospy.Publisher('goals_reset', String, queue_size=1)
        self.session_init_pub = rospy.Publisher('session_init', String, queue_size=1)

    def step(self):
        msg_str = self.recvStrFromProcessing()
        if msg_str != "none":
            msg_str = msg_str.split(',')
            print msg_str
            if msg_str[0] == 'GOALS_READY':
                self.send_autonomy_goals_to_processing_service()
                self.send_human_goals_to_processing_service()
            elif msg_str[0] == 'GOALS_RESET':
                msg = String("goals_reset")
                self.goals_reset_pub.publish(msg)
                #reset all goals.
                self.reset_num_goals()
                self.reset_autonomy_goals()
                self.reset_human_goals()
                self.send_autonomy_goals_to_processing_service()
                self.send_human_goals_to_processing_service()

                #reset autonomy and human robot pose.
                self.reset_autonomy_robot()
                self.reset_human_robot()
                self.send_autonomy_robot_pose_to_processing_service()
                self.send_human_robot_pose_to_processing_service()

            elif msg_str[0] == 'BEGIN_TRIAL':
                msg = String("begin_trial")
                self.begin_trial_pub.publish(msg)

                trigger_msg = SetBoolRequest()
                trigger_msg.data = True
                self.autonomy_node_trigger_trial(trigger_msg)

            elif msg_str[0] == 'END_TRIAL':
                trigger_msg = SetBoolRequest()
                trigger_msg.data = False
                self.autonomy_node_trigger_trial(trigger_msg)

                msg = String("end_trial")
                self.end_trial_pub.publish(msg)

            elif msg_str[0] == 'ROBOT_READY':
                msg = String("session_init")
                self.session_init_pub.publish(msg)

                self.send_autonomy_robot_pose_to_processing_service()
                self.send_human_robot_pose_to_processing_service()

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
