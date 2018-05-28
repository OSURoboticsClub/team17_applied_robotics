#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep
import serial
import random

from denso_interface_controller.msg import InterfaceControlMessage, InterfaceStatusMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "denso_interface_controller"
DEFAULT_HERTZ = 30

STATUSES_TOPIC = "denso_interface_controller/status"
CONTROL_TOPIC = "denso_interface_controller/control"

DEFAULT_PORT = "/dev/ttyUSB0"
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

        self.status_publisher = rospy.Publisher(STATUSES_TOPIC, InterfaceStatusMessage, queue_size=1)
        self.control_subscriber = rospy.Subscriber(CONTROL_TOPIC, InterfaceControlMessage, self.control_requested_callback)

        self.command_queue = []

        # ########### Start class ##########
        self.run()

    def run(self):

        while not rospy.is_shutdown():
            start_time = time()

            self.broadcast_status_if_available()
            self.send_control_packet_if_command_available()

            time_diff = time() - start_time
            sleep(max(self.wait_time - time_diff, 0))

    def broadcast_status_if_available(self):
        if self.serial_device.inWaiting():

            try:
                line = self.serial_device.readline()

                blocks = line.strip("\r\n").split("|")

                status_message = InterfaceStatusMessage()
                status_message.current_actual_pressure = float(blocks[0])
                status_message.current_set_pressure = float(blocks[1])
                status_message.compressor_on = True if blocks[2] == '1' else False
                status_message.ball_detected = True if blocks[3] == '1' else False
                status_message.motor_power_on = True if blocks[4] == '1' else False
                self.status_publisher.publish(status_message)

            except:
                print "Could not parse microcontroller message."

    def send_control_packet_if_command_available(self):
        if self.command_queue:
            item = self.command_queue[0]  # type: InterfaceControlMessage
            del self.command_queue[0]

            psi_value = int(item.set_pressure)
            should_tamp = int(item.should_tamp)
            should_shoot = int(item.should_fire)

            output = "%d|%d|%d|" % (psi_value, should_tamp, should_shoot)

            output += "\n"

            print "output:", output

            self.serial_device.write(output)

    def control_requested_callback(self, data):
        self.command_queue.append(data)

if __name__ == "__main__":
    InterfaceController()
