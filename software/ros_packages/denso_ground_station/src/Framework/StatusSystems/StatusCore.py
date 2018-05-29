#!/usr/bin/env python
# coding=utf-8

import rospy
from PyQt5 import QtWidgets, QtCore, QtTest
import paramiko
import random
import Resources.definitions as definitions
from denso_master.msg import DensoStatusMessage
from denso_interface_controller.msg import InterfaceStatusMessage, InterfaceControlMessage
from std_msgs.msg import UInt8, Bool, Float32MultiArray, UInt8MultiArray


DENSO_STATUS_TOPIC_NAME = "/denso_status"
DENSO_SPEED_TOPIC_NAME = "/denso_control/speed"
DENSO_MOTOR_TOPIC_NAME = "/denso_control/motors_enabled"

DENSO_ABSOLUTE_JOINTS_TOPIC_NAME = "/denso_control/absolute_joints"

DENSO_INTERFACE_CONTROLLER_STATUS = "/denso_interface_controller/status"
DENSO_INTERFACE_CONTROLLER_CONTROL = "/denso_interface_controller/control"
DENSO_LED_CONTROLLER_CONTROL = "/denso_led_controller/control"

ACCESS_POINT_IP = "192.168.1.14"
ACCESS_POINT_USER = "denso"
ACCESS_POINT_PASSWORD = "denso"


class SensorCore(QtCore.QThread):
    # ########## create signals for slots ##########
    motor_enabled_stylesheet_change__signal = QtCore.pyqtSignal(str)
    arm_normal_stylesheet_change__signal = QtCore.pyqtSignal(str)
    arm_busy_stylesheet_change__signal = QtCore.pyqtSignal(str)
    error_stylesheet_change__signal = QtCore.pyqtSignal(str)
    tank_charging_stylesheet_change__signal = QtCore.pyqtSignal(str)
    ball_detected_stylesheet_change__signal = QtCore.pyqtSignal(str)

    arm_speed_change__signal = QtCore.pyqtSignal(str)
    tank_psi_change__signal = QtCore.pyqtSignal(str)
    tank_psi_set_change__signal = QtCore.pyqtSignal(str)

    x_changed__signal = QtCore.pyqtSignal(float)
    y_changed__signal = QtCore.pyqtSignal(float)
    z_changed__signal = QtCore.pyqtSignal(float)
    rx_changed__signal = QtCore.pyqtSignal(float)
    ry_changed__signal = QtCore.pyqtSignal(float)
    rz_changed__signal = QtCore.pyqtSignal(float)

    j1_changed__signal = QtCore.pyqtSignal(float)
    j2_changed__signal = QtCore.pyqtSignal(float)
    j3_changed__signal = QtCore.pyqtSignal(float)
    j4_changed__signal = QtCore.pyqtSignal(float)
    j5_changed__signal = QtCore.pyqtSignal(float)
    j6_changed__signal = QtCore.pyqtSignal(float)

    def __init__(self, shared_objects):
        super(SensorCore, self).__init__()

        self.run_thread_flag = True

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        # ########## set vars to gui elements
        self.motor_enabled_label = self.left_screen.motor_enabled_label  # type: QtWidgets.QLabel
        self.arm_speed_label = self.left_screen.arm_speed_label  # type: QtWidgets.QLabel
        self.arm_normal_label = self.left_screen.arm_normal_label  # type: QtWidgets.QLabel
        self.arm_busy_label = self.left_screen.arm_busy_label  # type: QtWidgets.QLabel
        self.arm_error_label = self.left_screen.error_label  # type: QtWidgets.QLabel
        self.tank_psi_label = self.left_screen.tank_psi_label  # type: QtWidgets.QLabel
        self.set_psi_label = self.left_screen.set_psi_label  # type: QtWidgets.QLabel
        self.compressor_on_label = self.left_screen.compressor_on_label  # type: QtWidgets.QLabel
        self.ball_detected_label = self.left_screen.ball_detected_label  # type: QtWidgets.QLabel

        self.motor_enable_button = self.left_screen.motor_enable_button  # type: QtWidgets.QPushButton
        self.motor_disable_button = self.left_screen.motor_disable_button  # type: QtWidgets.QPushButton

        self.arm_set_speed_spinbox = self.left_screen.arm_speed_spinbox  # type: QtWidgets.QSpinBox
        self.arm_set_speed_button = self.left_screen.arm_set_speed_button  # type: QtWidgets.QPushButton

        self.preset_catch_button = self.left_screen.preset_catch_button  # type: QtWidgets.QPushButton
        self.preset_fire_button = self.left_screen.preset_fire_button  # type: QtWidgets.QPushButton

        self.adversary_demo_button = self.left_screen.adversary_demo_button  # type: QtWidgets.QPushButton

        self.set_psi_spinbox = self.left_screen.set_psi_spinbox  # type: QtWidgets.QSpinBox
        self.set_psi_pushbutton = self.left_screen.set_psi_pushbutton  # type: QtWidgets.QPushButton

        self.fire_button = self.left_screen.fire_button  # type: QtWidgets.QPushButton
        self.tamp_button = self.left_screen.tamp_button  # type: QtWidgets.QPushButton

        self.shutdown_systems_pushbutton = self.left_screen.shutdown_systems_pushbutton  # type: QtWidgets.QPushButton

        self.x_lcdnumber = self.left_screen.x_lcdnumber  # type: QtWidgets.QLCDNumber
        self.y_lcdnumber = self.left_screen.y_lcdnumber  # type: QtWidgets.QLCDNumber
        self.z_lcdnumber = self.left_screen.z_lcdnumber  # type: QtWidgets.QLCDNumber
        self.rx_lcdnumber = self.left_screen.rx_lcdnumber  # type: QtWidgets.QLCDNumber
        self.ry_lcdnumber = self.left_screen.ry_lcdnumber  # type: QtWidgets.QLCDNumber
        self.rz_lcdnumber = self.left_screen.rz_lcdnumber  # type: QtWidgets.QLCDNumber

        self.j1_lcdnumber = self.left_screen.j1_lcdnumber  # type: QtWidgets.QLCDNumber
        self.j2_lcdnumber = self.left_screen.j2_lcdnumber  # type: QtWidgets.QLCDNumber
        self.j3_lcdnumber = self.left_screen.j3_lcdnumber  # type: QtWidgets.QLCDNumber
        self.j4_lcdnumber = self.left_screen.j4_lcdnumber  # type: QtWidgets.QLCDNumber
        self.j5_lcdnumber = self.left_screen.j5_lcdnumber  # type: QtWidgets.QLCDNumber
        self.j6_lcdnumber = self.left_screen.j6_lcdnumber  # type: QtWidgets.QLCDNumber

        self.status_subscriber = rospy.Subscriber(DENSO_STATUS_TOPIC_NAME, DensoStatusMessage, self.on_new_status_update_received)
        self.controller_status_subscriber = rospy.Subscriber(DENSO_INTERFACE_CONTROLLER_STATUS, InterfaceStatusMessage, self.on_new_controller_status_update_received)

        self.abs_joints_publisher = rospy.Publisher(DENSO_ABSOLUTE_JOINTS_TOPIC_NAME, Float32MultiArray, queue_size=1)

        self.speed_publisher = rospy.Publisher(DENSO_SPEED_TOPIC_NAME, UInt8, queue_size=1)
        self.motor_enable_publisher = rospy.Publisher(DENSO_MOTOR_TOPIC_NAME, Bool, queue_size=1)

        self.interface_controller_publisher = rospy.Publisher(DENSO_INTERFACE_CONTROLLER_CONTROL, InterfaceControlMessage, queue_size=1)
        self.interface_controller_message = InterfaceControlMessage()

        self.status_data = None
        self.new_statuses = False

        self.controller_status_data = None
        self.new_controller_status_data = False

        self.ssh_client = None

    def run(self):
        self.setup_and_connect_ssh_client()

        while self.run_thread_flag:
            self.update_statuses_if_needed()
            self.update_controller_statuses_if_needed()
            self.msleep(20)

    def setup_and_connect_ssh_client(self):
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(ACCESS_POINT_IP, username=ACCESS_POINT_USER, password=ACCESS_POINT_PASSWORD,
                                compress=True)

    def on_new_status_update_received(self, data):
        self.status_data = data
        self.new_statuses = True

    def on_new_controller_status_update_received(self, data):
        self.controller_status_data = data
        self.new_controller_status_data = True

    def update_statuses_if_needed(self):
        if self.new_statuses:
            if self.status_data.motor_enabled:
                self.motor_enabled_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.motor_enabled_stylesheet_change__signal.emit("background-color:darkred;")

            self.arm_speed_change__signal.emit(str(self.status_data.speed))

            if self.status_data.arm_normal:
                self.arm_normal_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.arm_normal_stylesheet_change__signal.emit("background-color:darkred;")

            if not self.status_data.arm_busy:
                self.arm_busy_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.arm_busy_stylesheet_change__signal.emit("background-color:darkred;")

            if self.status_data.error == "":
                self.error_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.error_stylesheet_change__signal.emit("background-color:darkred;")

            self.x_changed__signal.emit(self.status_data.positions[0])
            self.y_changed__signal.emit(self.status_data.positions[1])
            self.z_changed__signal.emit(self.status_data.positions[2])
            self.rx_changed__signal.emit(self.status_data.positions[3])
            self.ry_changed__signal.emit(self.status_data.positions[4])
            self.rz_changed__signal.emit(self.status_data.positions[5])

            self.j1_changed__signal.emit(self.status_data.joints[0])
            self.j2_changed__signal.emit(self.status_data.joints[1])
            self.j3_changed__signal.emit(self.status_data.joints[2])
            self.j4_changed__signal.emit(self.status_data.joints[3])
            self.j5_changed__signal.emit(self.status_data.joints[4])
            self.j6_changed__signal.emit(self.status_data.joints[5])

            self.new_statuses = False

    def update_controller_statuses_if_needed(self):
        if self.new_controller_status_data:
            self.tank_psi_change__signal.emit(str(int(self.controller_status_data.current_actual_pressure)))
            self.tank_psi_set_change__signal.emit(str(int(self.controller_status_data.current_set_pressure)))

            if self.controller_status_data.compressor_on:
                self.tank_charging_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.tank_charging_stylesheet_change__signal.emit("background-color:darkred;")

            if self.controller_status_data.ball_detected:
                self.ball_detected_stylesheet_change__signal.emit("background-color:darkgreen;")
            else:
                self.ball_detected_stylesheet_change__signal.emit("background-color:darkred;")

            self.new_controller_status_data = False

    def on_tank_fire_pressed__slot(self):
        self.interface_controller_message.should_fire = 1
        self.interface_controller_message.set_pressure = 0
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_fire = 0

    def on_tamp_pressed__slot(self):
        self.interface_controller_message.should_tamp = 1
        self.interface_controller_publisher.publish(self.interface_controller_message)
        self.interface_controller_message.should_tamp = 0

    def on_set_tank_psi_pressed__slot(self):
        self.interface_controller_message.set_pressure = self.set_psi_spinbox.value()
        self.interface_controller_publisher.publish(self.interface_controller_message)

    def on_motor_enabled_pressed__slot(self):
        self.motor_enable_publisher.publish(1)

    def on_motor_disabled_pressed__slot(self):
        self.motor_enable_publisher.publish(0)

    def on_set_arm_speed_pressed__slot(self):
        self.speed_publisher.publish(self.arm_set_speed_spinbox.value())

    def on_catch_pressed__slot(self):
        self.abs_joints_publisher.publish(definitions.CATCH_JOINT_POSITIONS_MESSAGE)

    def on_fire_pressed__slot(self):
        self.abs_joints_publisher.publish(definitions.FIRE_JOINT_POSITIONS_MESSAGE)

    def on_adversary_demo_button_pressed__slot(self):

        num_fakeouts = random.randint(8, 15)

        for i in range(num_fakeouts):
            message = Float32MultiArray()

            rand_j1 = random.randint(definitions.ADVERSARY_RANDOM_J1_MIN, definitions.ADVERSARY_RANDOM_J1_MAX)
            rand_j5 = random.randint(definitions.ADVERSARY_RANDOM_J5_MIN, definitions.ADVERSARY_RANDOM_J5_MAX)

            fire_message = list(definitions.FIRE_JOINT_POSITIONS)
            fire_message[0] = rand_j1
            fire_message[4] = rand_j5

            message.data = tuple(fire_message)

            self.abs_joints_publisher.publish(message)

        self.on_catch_pressed__slot()

    def on_shutdown_systems_clicked__slot(self):
        self.ssh_client.exec_command("/home/denso/Github/team17_applied_robotics/software/environment/denso_shutdown.sh")
        self.msleep(1000)
        QtTest.QTest.keyPress(self.left_screen, QtCore.Qt.Key_Q, QtCore.Qt.ControlModifier)
        QtTest.QTest.keyRelease(self.left_screen, QtCore.Qt.Key_Q, QtCore.Qt.ControlModifier)

    def connect_signals_and_slots(self):
        self.motor_enabled_stylesheet_change__signal.connect(self.motor_enabled_label.setStyleSheet)
        self.arm_normal_stylesheet_change__signal.connect(self.arm_normal_label.setStyleSheet)
        self.arm_busy_stylesheet_change__signal.connect(self.arm_busy_label.setStyleSheet)
        self.error_stylesheet_change__signal.connect(self.arm_error_label.setStyleSheet)
        self.tank_charging_stylesheet_change__signal.connect(self.compressor_on_label.setStyleSheet)
        self.ball_detected_stylesheet_change__signal.connect(self.ball_detected_label.setStyleSheet)

        self.arm_speed_change__signal.connect(self.arm_speed_label.setText)
        self.tank_psi_change__signal.connect(self.tank_psi_label.setText)
        self.tank_psi_set_change__signal.connect(self.set_psi_label.setText)

        self.x_changed__signal.connect(self.x_lcdnumber.display)
        self.y_changed__signal.connect(self.y_lcdnumber.display)
        self.z_changed__signal.connect(self.z_lcdnumber.display)
        self.rx_changed__signal.connect(self.rx_lcdnumber.display)
        self.ry_changed__signal.connect(self.ry_lcdnumber.display)
        self.rz_changed__signal.connect(self.rz_lcdnumber.display)

        self.j1_changed__signal.connect(self.j1_lcdnumber.display)
        self.j2_changed__signal.connect(self.j2_lcdnumber.display)
        self.j3_changed__signal.connect(self.j3_lcdnumber.display)
        self.j4_changed__signal.connect(self.j4_lcdnumber.display)
        self.j5_changed__signal.connect(self.j5_lcdnumber.display)
        self.j6_changed__signal.connect(self.j6_lcdnumber.display)

        self.motor_enable_button.clicked.connect(self.on_motor_enabled_pressed__slot)
        self.motor_disable_button.clicked.connect(self.on_motor_disabled_pressed__slot)
        self.arm_set_speed_button.clicked.connect(self.on_set_arm_speed_pressed__slot)

        self.preset_catch_button.clicked.connect(self.on_catch_pressed__slot)
        self.preset_fire_button.clicked.connect(self.on_fire_pressed__slot)

        self.adversary_demo_button.clicked.connect(self.on_adversary_demo_button_pressed__slot)

        self.set_psi_pushbutton.clicked.connect(self.on_set_tank_psi_pressed__slot)

        self.fire_button.clicked.connect(self.on_tank_fire_pressed__slot)
        self.tamp_button.clicked.connect(self.on_tamp_pressed__slot)

        self.shutdown_systems_pushbutton.clicked.connect(self.on_shutdown_systems_clicked__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False


if __name__ == "__main__":
    rover_statuses = SensorCore()
    rover_statuses.run()
