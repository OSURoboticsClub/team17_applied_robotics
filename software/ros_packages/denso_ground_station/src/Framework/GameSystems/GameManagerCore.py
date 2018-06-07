#!/usr/bin/env python
# coding=utf-8

import rospy
from PyQt5 import QtWidgets, QtCore, QtGui, uic

import random
import time

import Resources.definitions as definitions
from denso_master.msg import DensoStatusMessage
from denso_interface_controller.msg import InterfaceStatusMessage, InterfaceControlMessage
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

DENSO_STATUS_TOPIC_NAME = "/denso_status"
DENSO_ABSOLUTE_JOINTS_TOPIC_NAME = "/denso_control/absolute_joints"

DENSO_INTERFACE_CONTROLLER_CONTROL = "/denso_interface_controller/control"
DENSO_INTERFACE_CONTROLLER_STATUS = "/denso_interface_controller/status"
DENSO_LED_CONTROLLER_CONTROL = "/denso_led_controller/control"

COLOR_BLUE = (0, 0, 255)
COLOR_RED = (255, 0, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_PURPLE = (255, 0, 255)
COLOR_ORANGE = (255, 165, 0)
COLOR_BLACK = (0, 0, 0)

AXES_DEGREES_ERROR = 0.01


class GameManager(QtCore.QThread):
    # ########## create signals for slots ##########
    update_degrees_from_center_friendly__signal = QtCore.pyqtSignal(int)
    update_degrees_from_45_friendly__signal = QtCore.pyqtSignal(int)
    update_psi_friendly__signal = QtCore.pyqtSignal(int)

    update_degrees_from_center_adversary__signal = QtCore.pyqtSignal(int)
    update_degrees_from_45_adversary__signal = QtCore.pyqtSignal(int)
    update_psi_adversary__signal = QtCore.pyqtSignal(int)

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
        self.run_friendly_pushbutton = self.left_screen.run_friendly_pushbutton  # type: QtWidgets.QPushButton
        self.friendly_continuous_run_checkbox = self.left_screen.friendly_continuous_run_checkbox  # type: QtWidgets.QCheckBox

        self.adversary_degrees_from_center_spinbox = self.left_screen.adversary_degrees_from_center_spinbox  # type: QtWidgets.QSpinBox
        self.adversary_degrees_from_45_spinbox = self.left_screen.adversary_degrees_from_45_spinbox  # type: QtWidgets.QSpinBox
        self.adversary_psi_spinbox = self.left_screen.adversary_psi_spinbox  # type: QtWidgets.QSpinBox
        self.run_adversary_pushbutton = self.left_screen.run_adversary_pushbutton  # type: QtWidgets.QPushButton
        self.adversary_continuous_run_checkbox = self.left_screen.adversary_continuous_run_checkbox  # type: QtWidgets.QCheckBox

        self.z1s1_pushbutton = self.left_screen.z1s1_pushbutton  # type: QtWidgets.QPushButton
        self.z1s2_pushbutton = self.left_screen.z1s2_pushbutton  # type: QtWidgets.QPushButton
        self.z2s3_pushbutton = self.left_screen.z2s3_pushbutton  # type: QtWidgets.QPushButton
        self.z2s4_pushbutton = self.left_screen.z2s4_pushbutton  # type: QtWidgets.QPushButton
        self.z2s5_pushbutton = self.left_screen.z2s5_pushbutton  # type: QtWidgets.QPushButton
        self.z3s6_pushbutton = self.left_screen.z3s6_pushbutton  # type: QtWidgets.QPushButton
        self.z3s7_pushbutton = self.left_screen.z3s7_pushbutton  # type: QtWidgets.QPushButton
        self.z3s8_pushbutton = self.left_screen.z3s8_pushbutton  # type: QtWidgets.QPushButton
        self.z3s9_pushbutton = self.left_screen.z3s9_pushbutton  # type: QtWidgets.QPushButton

        self.global_offset_spinbox = self.left_screen.global_offset_spinbox  # type: QtWidgets.QSpinBox

        self.BUTTON_MAPPINGS_FRIENDLY = {
            self.z1s1_pushbutton: definitions.Z1_S1_FRIENDLY,
            self.z1s2_pushbutton: definitions.Z1_S2_FRIENDLY,

            self.z2s3_pushbutton: definitions.Z2_S3_FRIENDLY,
            self.z2s4_pushbutton: definitions.Z2_S4_FRIENDLY,
            self.z2s5_pushbutton: definitions.Z2_S5_FRIENDLY,

            self.z3s6_pushbutton: definitions.Z3_S6_FRIENDLY,
            self.z3s7_pushbutton: definitions.Z3_S7_FRIENDLY,
            self.z3s8_pushbutton: definitions.Z3_S8_FRIENDLY,
            self.z3s9_pushbutton: definitions.Z3_S9_FRIENDLY
        }

        self.BUTTON_MAPPINGS_ADVERSARY = {
            self.z1s1_pushbutton: definitions.Z1_S1_ADVERSARY,
            self.z1s2_pushbutton: definitions.Z1_S2_ADVERSARY,

            self.z2s3_pushbutton: definitions.Z2_S3_ADVERSARY,
            self.z2s4_pushbutton: definitions.Z2_S4_ADVERSARY,
            self.z2s5_pushbutton: definitions.Z2_S5_ADVERSARY,

            self.z3s6_pushbutton: definitions.Z3_S6_ADVERSARY,
            self.z3s7_pushbutton: definitions.Z3_S7_ADVERSARY,
            self.z3s8_pushbutton: definitions.Z3_S8_ADVERSARY,
            self.z3s9_pushbutton: definitions.Z3_S9_ADVERSARY
        }

        self.status_subscriber = rospy.Subscriber(DENSO_STATUS_TOPIC_NAME, DensoStatusMessage,
                                                  self.on_new_denso_status_update_received)

        self.controller_status_subscriber = rospy.Subscriber(DENSO_INTERFACE_CONTROLLER_STATUS, InterfaceStatusMessage,
                                                             self.on_new_controller_status_update_received)

        self.abs_joints_publisher = rospy.Publisher(DENSO_ABSOLUTE_JOINTS_TOPIC_NAME, Float32MultiArray, queue_size=1)

        self.led_controller_publisher = rospy.Publisher(DENSO_LED_CONTROLLER_CONTROL, UInt8MultiArray, queue_size=1)

        self.interface_controller_publisher = rospy.Publisher(DENSO_INTERFACE_CONTROLLER_CONTROL, InterfaceControlMessage, queue_size=1)
        self.interface_controller_message = InterfaceControlMessage()

        self.run_friendly_flag = False
        self.run_adversary_flag = False

        self.denso_status = None  # type: DensoStatusMessage
        self.interface_controller_status = None

        self.last_color = COLOR_YELLOW

        self.zone_pressed = False

    def run(self):
        while self.run_thread_flag:
            self.show_color(COLOR_BLUE)

            if self.run_friendly_flag:
                self.run_friendly()
            elif self.run_adversary_flag:
                self.run_adversary()
            self.msleep(20)

    # ##### For J5, increasing values makese it point down #####
    # ##### For J1, increasing values is counterclockwise and increasing values #####

    def run_friendly(self):
        # ##### Show Friendly Lights :) #####
        self.show_color(COLOR_YELLOW)
        self.last_color = COLOR_YELLOW

        # ##### Wait for ball to be detected #####
        while not self.interface_controller_status.ball_detected:
            if self.last_color == COLOR_YELLOW:
                self.show_color(COLOR_BLACK)
                self.last_color = COLOR_BLACK
            else:
                self.show_color(COLOR_YELLOW)
                self.last_color = COLOR_YELLOW
            self.msleep(20)

        # ##### Start at Fire Position #####
        self.move_arm_to_position_and_wait(definitions.FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Wait for zone click if continuous #####
        if self.friendly_continuous_run_checkbox.isChecked():
            while not self.zone_pressed:
                if self.last_color == COLOR_PURPLE:
                    self.show_color(COLOR_BLACK)
                    self.last_color = COLOR_BLACK
                else:
                    self.show_color(COLOR_PURPLE)
                    self.last_color = COLOR_PURPLE
                self.msleep(20)

        self.zone_pressed = False

        # ##### Get values from gui #####
        psi = self.friendly_psi_spinbox.value()
        degrees_from_center = self.friendly_degrees_from_center_spinbox.value()
        degrees_from_45 = self.friendly_degrees_from_45_spinbox.value()

        # ##### Charge tank and Tamp Ball #####
        self.interface_controller_message.set_pressure = psi
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_tamp = 0

        # ##### Wait until feedback says tank is charging #####
        if (psi - self.interface_controller_status.current_actual_pressure) > 5:
            if psi > 5:
                while not self.interface_controller_status.compressor_on:
                    self.msleep(50)

        # ##### Adjust to fire angles if needed #####
        fire_joint_positions = list(definitions.FIRE_JOINT_POSITIONS)
        fire_joint_positions[0] -= degrees_from_center - self.global_offset_spinbox.value()
        fire_joint_positions[4] -= degrees_from_45

        self.abs_joints_publisher.publish(Float32MultiArray(data=tuple(fire_joint_positions)))

        # ##### Wait for compressor to finish building pressure #####
        while self.interface_controller_status.compressor_on:
            self.msleep(100)

        # ##### Countdown before fire #####
        message = UInt8MultiArray()

        last_time = time.time()
        num_seconds = 2
        current_time = time.time() - last_time
        while current_time < num_seconds:
            values = []
            for i in range(18):
                if i > (float(num_seconds - current_time) / num_seconds) * 18:
                    values.append(0)
                    values.append(0)
                    values.append(0)
                else:
                    values.append(0)
                    values.append(255)
                    values.append(0)

            message.data = values

            self.led_controller_publisher.publish(message)
            self.msleep(100)
            current_time = time.time() - last_time

        self.show_color(COLOR_RED)

        # ##### Fire! #####
        self.interface_controller_message.should_fire = 1
        self.interface_controller_message.set_pressure = 0
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_fire = 0

        # ##### Check if in continuous mode and continue if necessary #####
        self.msleep(500)
        if self.friendly_continuous_run_checkbox.isChecked():
            self.interface_controller_message.set_pressure = psi
            self.interface_controller_publisher.publish(self.interface_controller_message)

        # ##### Wait before moving #####
        self.msleep(500)

        # ##### Move back to Fire Position #####
        if degrees_from_center != 0 or degrees_from_45 != 0:
            self.move_arm_to_position_and_wait(definitions.FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Move to Catch Position #####
        self.move_arm_to_position_and_wait(definitions.CATCH_JOINT_POSITIONS_MESSAGE)

        # ##### Cancel run flag if not in continuous mode
        if not self.friendly_continuous_run_checkbox.isChecked():
            self.run_friendly_flag = False

    def run_adversary(self):
        # ##### Show Friendly Lights :) #####
        self.show_color(COLOR_YELLOW)
        self.last_color = COLOR_YELLOW

        # ##### Wait for ball to be detected #####
        while not self.interface_controller_status.ball_detected:
            if self.last_color == COLOR_YELLOW:
                self.show_color(COLOR_BLACK)
                self.last_color = COLOR_BLACK
            else:
                self.show_color(COLOR_YELLOW)
                self.last_color = COLOR_YELLOW
            self.msleep(20)
        self.show_color(COLOR_YELLOW)

        # ##### Start at Fire Position #####
        self.move_arm_to_position_and_wait(definitions.FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Wait for zone click if continuous #####
        if self.adversary_continuous_run_checkbox.isChecked():
            while not self.zone_pressed:
                if self.last_color == COLOR_PURPLE:
                    self.show_color(COLOR_BLACK)
                    self.last_color = COLOR_BLACK
                else:
                    self.show_color(COLOR_PURPLE)
                    self.last_color = COLOR_PURPLE
                self.msleep(20)
            self.show_color(COLOR_PURPLE)

        self.zone_pressed = False

        # ##### Get values from gui #####
        psi = self.adversary_psi_spinbox.value()
        degrees_from_center = self.adversary_degrees_from_center_spinbox.value()
        degrees_from_45 = self.adversary_degrees_from_45_spinbox.value()

        # ##### Charge tank and Tamp Ball #####
        self.interface_controller_message.set_pressure = psi
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_tamp = 0

        # ##### Wait until feedback says tank is charging #####
        if (psi - self.interface_controller_status.current_actual_pressure) > 5:
            if psi > 5:
                while not self.interface_controller_status.compressor_on:
                    self.msleep(50)

        # ##### Adjust to fire angles if needed #####
        fire_joint_positions = list(definitions.FIRE_JOINT_POSITIONS)
        fire_joint_positions[0] -= degrees_from_center - self.global_offset_spinbox.value()
        fire_joint_positions[4] = random.randint(definitions.ADVERSARY_RANDOM_J5_MIN, definitions.ADVERSARY_RANDOM_J5_MAX)

        self.abs_joints_publisher.publish(Float32MultiArray(data=tuple(fire_joint_positions)))

        # ##### Wait for compressor to finish building pressure #####
        while self.interface_controller_status.compressor_on:
            self.move_arm_random_and_wait()

        # ##### Preparation for firing #####
        self.interface_controller_message.should_fire = 1
        self.interface_controller_message.set_pressure = 0

        # ##### Countdown before fire #####
        message = UInt8MultiArray()

        last_time = time.time()
        num_seconds = 3
        fakeout_time = random.randint(500, num_seconds * 1000) / 1000.0
        current_time = time.time() - last_time

        has_fired = False

        print fakeout_time
        while current_time < num_seconds:
            values = []
            for i in range(18):
                if i > (float(num_seconds - current_time) / num_seconds) * 18:
                    values.append(0)
                    values.append(0)
                    values.append(0)
                else:
                    values.append(0)
                    values.append(255)
                    values.append(0)

            message.data = values

            self.led_controller_publisher.publish(message)

            if (time.time() - last_time) > fakeout_time:
                self.move_arm_to_position_and_wait(self.get_adversary_fire_message())
                self.interface_controller_publisher.publish(self.interface_controller_message)
                has_fired = True
                break
            else:
                self.move_arm_random_and_wait()
                self.msleep(random.randint(1, 500))
            current_time = time.time() - last_time

        if not has_fired:
            self.move_arm_to_position_and_wait(self.get_adversary_fire_message())
            self.interface_controller_publisher.publish(self.interface_controller_message)

        self.show_color(COLOR_RED)

        # ##### Disable firing #####
        self.interface_controller_message.should_fire = 0

        # ##### Check if in continuous mode and continue if necessary #####
        self.msleep(500)
        if self.adversary_continuous_run_checkbox.isChecked():
            self.interface_controller_message.set_pressure = psi
            self.interface_controller_publisher.publish(self.interface_controller_message)

        # ##### Wait before moving #####
        self.msleep(500)

        # ##### Move back to Fire Position #####
        if degrees_from_center != 0 or degrees_from_45 != 0:
            self.move_arm_to_position_and_wait(definitions.FIRE_JOINT_POSITIONS_MESSAGE)

        # ##### Move to Catch Position #####
        self.move_arm_to_position_and_wait(definitions.CATCH_JOINT_POSITIONS_MESSAGE)

        # ##### Cancel run flag if not in continuous mode
        if not self.adversary_continuous_run_checkbox.isChecked():
            self.run_adversary_flag = False

    def arm_arrived_at_position(self, desired_position):
        if not self.denso_status:
            return False

        joint_positions = self.denso_status.joints

        within_error = True

        for i in range(len(joint_positions)):
            if not (abs(joint_positions[i] - desired_position[i]) < AXES_DEGREES_ERROR):
                within_error = False

        return within_error

    def move_arm_random_and_wait(self):
        message = Float32MultiArray()
        rand_j1 = random.randint(definitions.ADVERSARY_RANDOM_J1_MIN, definitions.ADVERSARY_RANDOM_J1_MAX)
        rand_j5 = random.randint(definitions.ADVERSARY_RANDOM_J5_MIN, definitions.ADVERSARY_RANDOM_J5_MAX)

        fire_message = list(definitions.FIRE_JOINT_POSITIONS)
        fire_message[0] = rand_j1
        fire_message[4] = rand_j5

        message.data = tuple(fire_message)

        self.move_arm_to_position_and_wait(message)

    def get_adversary_fire_message(self):
        degrees_from_center = self.adversary_degrees_from_center_spinbox.value()
        degrees_from_45 = self.adversary_degrees_from_45_spinbox.value()

        fire_joint_positions = list(definitions.FIRE_JOINT_POSITIONS)

        message = Float32MultiArray()

        if degrees_from_center >= 0:
            rand_j1 = 3 - self.global_offset_spinbox.value()
        else:
            rand_j1 = -3 - self.global_offset_spinbox.value()

        print rand_j1

        rand_j5 = fire_joint_positions[4] - degrees_from_45

        fire_message = list(definitions.FIRE_JOINT_POSITIONS)
        fire_message[0] = rand_j1
        fire_message[4] = rand_j5

        message.data = tuple(fire_message)

        return message

    def show_color(self, color):
        message = UInt8MultiArray()
        values = []
        for _ in range(18):
            values.append(color[0])
            values.append(color[1])
            values.append(color[2])
        message.data = values

        self.led_controller_publisher.publish(message)

    def move_arm_to_position_and_wait(self, message):
        self.abs_joints_publisher.publish(message)
        while not self.arm_arrived_at_position(message.data):
            self.msleep(10)

    def on_zone_pushbutton_clicked__slot(self):
        event_sender = self.sender()

        new_zone_friendly = self.BUTTON_MAPPINGS_FRIENDLY[event_sender]
        new_zone_adversary = self.BUTTON_MAPPINGS_ADVERSARY[event_sender]

        self.update_degrees_from_center_friendly__signal.emit(new_zone_friendly["degrees_from_center"])
        self.update_degrees_from_45_friendly__signal.emit(new_zone_friendly["degrees_from_45"])
        self.update_psi_friendly__signal.emit(new_zone_friendly["psi"])

        self.update_degrees_from_center_adversary__signal.emit(new_zone_adversary["degrees_from_center"])
        self.update_degrees_from_45_adversary__signal.emit(new_zone_adversary["degrees_from_45"])
        self.update_psi_adversary__signal.emit(new_zone_adversary["psi"])

        self.zone_pressed = True

    def on_new_denso_status_update_received(self, data):
        self.denso_status = data

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

        self.z1s1_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z1s2_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z2s3_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z2s4_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z2s5_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z3s6_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z3s7_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z3s8_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)
        self.z3s9_pushbutton.clicked.connect(self.on_zone_pushbutton_clicked__slot)

        self.update_degrees_from_center_friendly__signal.connect(self.friendly_degrees_from_center_spinbox.setValue)
        self.update_degrees_from_center_adversary__signal.connect(self.adversary_degrees_from_center_spinbox.setValue)

        self.update_degrees_from_45_friendly__signal.connect(self.friendly_degrees_from_45_spinbox.setValue)
        self.update_degrees_from_45_adversary__signal.connect(self.adversary_degrees_from_45_spinbox.setValue)

        self.update_psi_friendly__signal.connect(self.friendly_psi_spinbox.setValue)
        self.update_psi_adversary__signal.connect(self.adversary_psi_spinbox.setValue)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
