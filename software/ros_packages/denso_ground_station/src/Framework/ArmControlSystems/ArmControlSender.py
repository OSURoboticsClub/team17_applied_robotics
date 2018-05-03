#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
from time import time

import rospy
from std_msgs.msg import Float32MultiArray

#####################################
# Global Variables
#####################################
DEFAULT_ARM_COMMAND_TOPIC = "/denso_control/relative_position"

DRIVE_COMMAND_HERTZ = 40


#####################################
# Controller Class Definition
#####################################
class ArmControlSender(QtCore.QThread):
    def __init__(self, shared_objects):
        super(ArmControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.spacenav_thread = self.shared_objects["threaded_classes"]["Spacenav Sender"]

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        # Publishers
        self.arm_command_publisher = rospy.Publisher(DEFAULT_ARM_COMMAND_TOPIC, Float32MultiArray, queue_size=1)

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

        self.spacenav_data = None
        self.new_data = False

    def run(self):
        while self.run_thread_flag:

            start_time = time()

            self.send_arm_control_data()

            time_diff = time() - start_time
            self.msleep(max(int((self.wait_time - time_diff) * 1000), 0))

    def send_arm_control_data(self):
        if self.new_data:
            message = Float32MultiArray()

            multiplier = 3

            message.data = (
                self.spacenav_data["linear_y"] * multiplier,
                -self.spacenav_data["linear_x"] * multiplier,
                0,
                0,
                0,
                0
            )

            # message.data = (
            #     0,
            #     0,
            #     0,
            #     0, #self.spacenav_data["angular_x"],
            #     0, #self.spacenav_data["angular_y"],
            #     self.spacenav_data["angular_z"]
            # )

            # message.data = (
            #     self.spacenav_data["linear_y"],
            #     self.spacenav_data["linear_x"],
            #     self.spacenav_data["linear_z"],
            #     self.spacenav_data["angular_x"],
            #     self.spacenav_data["angular_y"],
            #     self.spacenav_data["angular_z"]
            # )
            self.arm_command_publisher.publish(message)

            # print self.spacenav_data
            self.new_data = False

    def on_spacenav_update_ready__slot(self, data):
        self.spacenav_data = data
        self.new_data = True
        # print self.spacenav_data

    def connect_signals_and_slots(self):
        self.spacenav_thread.spacenav_state_update__signal.connect(self.on_spacenav_update_ready__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
