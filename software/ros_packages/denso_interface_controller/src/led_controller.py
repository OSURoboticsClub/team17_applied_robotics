#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep
import serial
import random
import json

from std_msgs.msg import UInt8MultiArray

#####################################
# Global Variables
#####################################
NODE_NAME = "denso_led_controller"
DEFAULT_HERTZ = 50

CONTROL_TOPIC = "denso_led_controller/control"

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 57600


#####################################
# ControlSender Class Definition
#####################################
class InterfaceController(object):
    def __init__(self):
        super(InterfaceController, self).__init__()

        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.serial_device = serial.Serial(port=self.port, baudrate=self.baud)

        self.control_subscriber = rospy.Subscriber(CONTROL_TOPIC, UInt8MultiArray, self.control_requested_callback)

        self.command_queue = []

        temp_arary = []
        for _ in range(18):
            temp_arary.append(chr(255))
            temp_arary.append(chr(165))
            temp_arary.append(chr(0))

        self.command_queue.append(UInt8MultiArray(data=temp_arary))

        # ########### Start class ##########
        self.run()

    def run(self):


        while not rospy.is_shutdown():
            start_time = time()

            # if self.serial_device.inWaiting():
            #     print self.serial_device.readline()

            self.send_control_packet_if_command_available()

            time_diff = time() - start_time
            sleep(max(self.wait_time - time_diff, 0))

    def send_control_packet_if_command_available(self):
        if self.command_queue:
            item = self.command_queue[0]
            del self.command_queue[0]

            output = {
                "leds": [ord(i) for i in item.data]
            }

            # print json.dumps(output)
            self.serial_device.write(json.dumps(output) + "\n")

    def control_requested_callback(self, data):
        self.command_queue.append(data)

if __name__ == "__main__":
    InterfaceController()
