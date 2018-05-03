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

#####################################
# Global Variables
#####################################
NODE_NAME = "robot_control_status_receiver"
DEFAULT_HERTZ = 1000

DEFAULT_IP = "192.168.1.5"
DEFAULT_PORT = 9876

DEFAULT_STATUS_TOPIC = "denso_status"


#####################################
# ControlSender Class Definition
#####################################
class StatusReceiver(object):
    def __init__(self):
        super(StatusReceiver, self).__init__()

        rospy.init_node(NODE_NAME)

        self.address = rospy.get_param("~server_address", DEFAULT_IP)
        self.port = rospy.get_param("~server_port", DEFAULT_PORT)
        self.status_publish_topic = rospy.get_param("~status_topic_pub", DEFAULT_STATUS_TOPIC)
        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.status_publisher = rospy.Publisher(self.status_publish_topic, DensoStatusMessage, queue_size=1)

        self.status_tcp_client = None
        self.pound_count = 0

        self.current_message = ""

        # ########### Start class ##########
        self.run()

    def run(self):
        self.initalize_tcp_client()

        while not rospy.is_shutdown():
            start_time = time()

            self.current_message += self.status_tcp_client.recv(4096)

            found_pound = self.current_message.find("#####")

            if found_pound != -1:
                split_message = str(self.current_message[:found_pound])

                self.current_message = self.current_message[found_pound + 5:]

                try:
                    json_message = json.loads(split_message)

                    message = DensoStatusMessage()

                    message.positions = (
                        json_message["position"]["x"],
                        json_message["position"]["y"],
                        json_message["position"]["z"],
                        json_message["position"]["rx"],
                        json_message["position"]["ry"],
                        json_message["position"]["rz"],
                        json_message["position"]["fig"]

                    )

                    message.joints = (
                        json_message["joints"]["1"],
                        json_message["joints"]["2"],
                        json_message["joints"]["3"],
                        json_message["joints"]["4"],
                        json_message["joints"]["5"],
                        json_message["joints"]["6"],
                    )

                    message.motor_enabled = json_message["statuses"]["motor_enabled"]
                    message.arm_normal = json_message["statuses"]["arm_normal"]
                    message.error = json_message["statuses"]["error"]
                    message.speed = json_message["statuses"]["speed"]
                    message.arm_busy = json_message["statuses"]["arm_busy"]
                    message.tank_psi = json_message["statuses"]["tank_psi"]

                    self.status_publisher.publish(message)

                except Exception, e:
                    print e

                self.current_message = ""

            time_diff = time() - start_time
            sleep(max(self.wait_time - time_diff, 0))

    def initalize_tcp_client(self):
        self.status_tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.status_tcp_client.connect((self.address, self.port))


if __name__ == '__main__':
    StatusReceiver()
