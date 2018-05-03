#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep
import socket
import json

from network_master.msg import DensoStatusMessage
from std_msgs.msg import Float32MultiArray, Bool, UInt8

#####################################
# Global Variables
#####################################
NODE_NAME = "robot_control_control_sender"
DEFAULT_HERTZ = 100

DEFAULT_IP = "192.168.1.5"
DEFAULT_PORT = 9877

ABS_POS_TOPIC = "denso_control/absolute_position"
REL_POS_TOPIC = "denso_control/relative_position"
ABS_JOINT_TOPIC = "denso_control/absolute_joints"
REL_JOINT_TOPIC = "denso_control/relative_joints"

SPEED_TOPIC = "denso_control/speed"
MOTOR_TOPIC = "denso_control/motors_enabled"


#####################################
# ControlSender Class Definition
#####################################
class ControlSender(object):
    def __init__(self):
        super(ControlSender, self).__init__()

        rospy.init_node(NODE_NAME)

        self.address = rospy.get_param("~server_address", DEFAULT_IP)
        self.port = rospy.get_param("~server_port", DEFAULT_PORT)

        self.abs_position_topic = rospy.get_param("~abs_position_topic_sub", ABS_POS_TOPIC)
        self.rel_position_topic = rospy.get_param("~rel_position_topic_sub", REL_POS_TOPIC)
        self.abs_joint_topic = rospy.get_param("~abs_joint_topic_sub", ABS_JOINT_TOPIC)
        self.rel_joint_topic = rospy.get_param("~rel_joint_topic_sub", REL_JOINT_TOPIC)

        self.speed_topic = rospy.get_param("~speed_topic_sub", SPEED_TOPIC)
        self.motor_topic = rospy.get_param("~motor_control_topic_sub", MOTOR_TOPIC)

        self.abs_position_subscriber = rospy.Subscriber(self.abs_position_topic, Float32MultiArray, self.abs_position_callback)
        self.rel_position_subscriber = rospy.Subscriber(self.rel_position_topic, Float32MultiArray, self.rel_position_callback)
        self.abs_joints_subscriber = rospy.Subscriber(self.abs_joint_topic, Float32MultiArray, self.abs_joints_callback)
        self.rel_joints_subscriber = rospy.Subscriber(self.rel_joint_topic, Float32MultiArray, self.rel_joints_callback)

        self.speed_subscriber = rospy.Subscriber(self.speed_topic, UInt8, self.speed_control_callback)
        self.speed_subscriber = rospy.Subscriber(self.motor_topic, Bool, self.motor_control_callback)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.control_tcp_client = None

        self.command_queue = []

        # ########### Start class ##########
        self.run()

    def run(self):
        self.initalize_tcp_client()

        while not rospy.is_shutdown():
            start_time = time()

            self.process_command_queue_item()

            time_diff = time() - start_time
            sleep(max(self.wait_time - time_diff, 0))

    def initalize_tcp_client(self):
        self.control_tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_tcp_client.connect((self.address, self.port))

    def process_command_queue_item(self):
        if self.command_queue:
            data = self.command_queue[0]
            del self.command_queue[0]

            try:
                self.control_tcp_client.sendall(json.dumps(data))
                self.control_tcp_client.sendall("#####")
                # print "sending"
            except:
                pass

    def speed_control_callback(self, data):
        # print data.data
        self.command_queue.append({"change_robot_speed": data.data})

    def motor_control_callback(self, data):
        self.command_queue.append({"enable_motors": data.data})

    def abs_position_callback(self, data):
        # print data.data
        self.command_queue.append({"move_position_abs": data.data})

    def rel_position_callback(self, data):
        # print data.data
        self.command_queue.append({"move_position_rel": data.data})

    def abs_joints_callback(self, data):
        # print data.data
        self.command_queue.append({"move_joint_abs": data.data})

    def rel_joints_callback(self, data):
        # print data.data
        self.command_queue.append({"move_joint_rel": data.data})


if __name__ == "__main__":
    ControlSender()
