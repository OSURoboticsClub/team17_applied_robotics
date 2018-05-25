#!/usr/bin/env python
# coding=utf-8

import rospy
from PyQt5 import QtWidgets, QtCore, QtGui, uic

import random

from denso_master.msg import DensoStatusMessage
from denso_interface_controller.msg import InterfaceStatusMessage, InterfaceControlMessage
from std_msgs.msg import UInt8, Bool, Float32MultiArray, UInt8MultiArray

DENSO_ABSOLUTE_JOINTS_TOPIC_NAME = "/denso_control/absolute_joints"

DENSO_INTERFACE_CONTROLLER_CONTROL = "/denso_interface_controller/control"
DENSO_INTERFACE_CONTROLLER_STATUS = "/denso_interface_controller/status"
DENSO_LED_CONTROLLER_CONTROL = "/denso_led_controller/control"

FIRE_JOINT_POSITIONS = (0, -30, 135, 0, 29, -5)
CATCH_JOINT_POSITIONS = (0, -95, 96, 0, 90, -5)

CATCH_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=CATCH_JOINT_POSITIONS)
FIRE_JOINT_POSITIONS_MESSAGE = Float32MultiArray(data=FIRE_JOINT_POSITIONS)

ADVERSARY_RANDOM_J1_MIN = -35
ADVERSARY_RANDOM_J1_MAX = 35

ADVERSARY_RANDOM_J5_MIN = 20
ADVERSARY_RANDOM_J5_MAX = 45


class GameManager(QtCore.QThread):
    # ########## create signals for slots ##########
    motor_enabled_stylesheet_change__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(GameManager, self).__init__()

        self.run_thread_flag = True

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        # ########## set vars to gui elements
        self.friendly_degrees_from_center_spinbox = self.left_screen.friendly_degrees_from_center_spinbox  # type: QtWidgets.QSpinBox
        self.friendly_degrees_from_45_spinbox = self.left_screen.friendly_degrees_from_45_spinbox  # type: QtWidgets.QSpinBox
        self.friendly_psi_spinbox = self.left_screen.friendly_psi_spinbox  # type: QtWidgets.QSpinBox
        self.run_friendly_pushbutton = self.left_screen.run_friendly_pushbutton  # type: QtWidgets.QSpinBox

        self.adversary_degrees_from_center_spinbox = self.left_screen.adversary_degrees_from_center_spinbox  # type: QtWidgets.QSpinBox
        self.adversary_degrees_from_45_spinbox = self.left_screen.adversary_degrees_from_45_spinbox  # type: QtWidgets.QSpinBox
        self.adversary_psi_spinbox = self.left_screen.adversary_psi_spinbox  # type: QtWidgets.QSpinBox
        self.run_adversary_pushbutton = self.left_screen.run_adversary_pushbutton  # type: QtWidgets.QSpinBox

        self.controller_status_subscriber = rospy.Subscriber(DENSO_INTERFACE_CONTROLLER_STATUS, InterfaceStatusMessage,
                                                             self.on_new_controller_status_update_received)

        self.abs_joints_publisher = rospy.Publisher(DENSO_ABSOLUTE_JOINTS_TOPIC_NAME, Float32MultiArray, queue_size=1)

        self.led_controller_publisher = rospy.Publisher(DENSO_LED_CONTROLLER_CONTROL, UInt8MultiArray, queue_size=1)

        self.interface_controller_publisher = rospy.Publisher(DENSO_INTERFACE_CONTROLLER_CONTROL, InterfaceControlMessage, queue_size=1)
        self.interface_controller_message = InterfaceControlMessage()

        self.run_friendly_flag = False
        self.run_adversary_flag = False

        self.interface_controller_status = None

    def run(self):
        while self.run_thread_flag:
            message = UInt8MultiArray()
            values = []
            for _ in range(23):
                values.append(0)
                values.append(0)
                values.append(255)
            message.data = values

            self.led_controller_publisher.publish(message)

            if self.run_friendly_flag:
                self.run_friendly()
                self.run_friendly_flag = False
            elif self.run_adversary_flag:
                self.run_adversary()
                self.run_adversary_flag = False
            self.msleep(20)

    # ##### For J5, increasing values makese it point down #####
    # ##### For J1, increasing values is counterclockwise and increasing values #####

    def run_friendly(self):
        degrees_from_center = self.friendly_degrees_from_center_spinbox.value()
        degrees_from_45 = self.friendly_degrees_from_45_spinbox.value()
        psi = self.friendly_psi_spinbox.value()

        # ##### Show Friendly Lights :) #####
        message = UInt8MultiArray()
        values = []
        for _ in range(23):
            values.append(255)
            values.append(255)
            values.append(0)
        message.data = values

        self.led_controller_publisher.publish(message)

        # ##### Charge tank and Tamp Ball #####
        self.interface_controller_message.set_pressure = psi + 5
        # self.interface_controller_message.should_tamp = 1
        print self.interface_controller_message
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_tamp = 0

        # ##### Wait until feedback says tank is charging #####
        if psi > 5:
            while not self.interface_controller_status.compressor_on:
                self.msleep(50)

        # ##### Start at Fire Position #####
        self.abs_joints_publisher.publish(FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Adjust to fire angles #####
        fire_joint_positions = list(FIRE_JOINT_POSITIONS)
        fire_joint_positions[0] -= degrees_from_center
        fire_joint_positions[4] -= degrees_from_45

        self.abs_joints_publisher.publish(Float32MultiArray(data=tuple(fire_joint_positions)))

        # ##### Wait for compressor to finish building pressure #####
        while self.interface_controller_status.compressor_on:
            print self.interface_controller_status
            self.msleep(100)

        # ##### Countdown before fire #####
        for t in range(3):
            values = []
            for i in range(23):
                if i > (float(3 - t) / 3) * 23:
                    values.append(0)
                    values.append(0)
                    values.append(0)
                else:
                    values.append(0)
                    values.append(255)
                    values.append(0)

            message.data = values

            self.led_controller_publisher.publish(message)
            self.msleep(1000)

        message = UInt8MultiArray()
        values = []
        for _ in range(23):
            values.append(255)
            values.append(0)
            values.append(0)
        message.data = values

        self.led_controller_publisher.publish(message)

        # ##### Fire! #####
        self.interface_controller_message.should_fire = 1
        self.interface_controller_message.set_pressure = 0
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_fire = 0

        # ##### Wait before moving #####
        self.msleep(1000)

        # ##### Move to Catch Position #####
        self.abs_joints_publisher.publish(CATCH_JOINT_POSITIONS_MESSAGE)

    def run_adversary(self):
        degrees_from_center = self.adversary_degrees_from_center_spinbox.value()
        degrees_from_45 = self.adversary_degrees_from_45_spinbox.value()
        psi = self.adversary_psi_spinbox.value()

        # ##### Show Friendly Lights :) #####
        message = UInt8MultiArray()
        values = []
        for _ in range(23):
            values.append(255)
            values.append(255)
            values.append(0)
        message.data = values

        self.led_controller_publisher.publish(message)

        # ##### Charge tank and Tamp Ball #####
        self.interface_controller_message.set_pressure = psi + 5
        # self.interface_controller_message.should_tamp = 1
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_tamp = 0

        # ##### Wait until feedback says tank is charging #####
        if psi > 5:
            while not self.interface_controller_status.compressor_on:
                self.msleep(50)

        # ##### Start at Fire Position #####
        self.abs_joints_publisher.publish(FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Adjust to fire angles #####
        fire_joint_positions = list(FIRE_JOINT_POSITIONS)
        fire_joint_positions[0] -= degrees_from_center
        fire_joint_positions[4] -= degrees_from_45

        self.abs_joints_publisher.publish(Float32MultiArray(data=tuple(fire_joint_positions)))

        # ##### Wait for compressor to finish building pressure #####
        while self.interface_controller_status.compressor_on:
            print self.interface_controller_status
            self.msleep(100)

        # ##### Countdown before fire #####
        for t in range(3):
            values = []
            for i in range(23):
                if i > (float(3 - t) / 3) * 23:
                    values.append(0)
                    values.append(0)
                    values.append(0)
                else:
                    values.append(0)
                    values.append(255)
                    values.append(0)

            message.data = values

            self.led_controller_publisher.publish(message)
            self.msleep(1000)

        # ##### Fakeout #####
        message = Float32MultiArray()

        rand_j1 = random.randint(ADVERSARY_RANDOM_J1_MIN, ADVERSARY_RANDOM_J1_MAX)
        rand_j5 = random.randint(ADVERSARY_RANDOM_J5_MIN, ADVERSARY_RANDOM_J5_MAX)

        fire_message = list(FIRE_JOINT_POSITIONS)
        fire_message[0] = rand_j1
        fire_message[4] = rand_j5

        message.data = tuple(fire_message)

        self.abs_joints_publisher.publish(message)

        # ##### Final LED FOR FIRE #####
        message = UInt8MultiArray()
        values = []
        for _ in range(23):
            values.append(255)
            values.append(0)
            values.append(0)
        message.data = values

        self.led_controller_publisher.publish(message)

        # ##### Purposeful bad timing #####
        self.msleep(random.randint(500, 3000))

        # ##### Fire! #####
        self.interface_controller_message.should_fire = 1
        self.interface_controller_message.set_pressure = 0
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_fire = 0

        # ##### Wait before moving #####
        self.msleep(1000)

        # ##### Move to Catch Position #####
        self.abs_joints_publisher.publish(CATCH_JOINT_POSITIONS_MESSAGE)


    def on_new_controller_status_update_received(self, data):
        self.interface_controller_status = data

    def on_friendly_clicked__slot(self):
        if not self.run_adversary_flag:
            self.run_friendly_flag = True

    def on_adversary_clicked__slot(self):
        if not self.run_friendly_flag:
            self.run_adversary_flag = True

    def connect_signals_and_slots(self):
        self.run_friendly_pushbutton.clicked.connect(self.on_friendly_clicked__slot)
        self.run_adversary_pushbutton.clicked.connect(self.on_adversary_clicked__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
