#!/usr/bin/env python
# coding=utf-8

import rospy
from PyQt5 import QtWidgets, QtCore, QtGui, uic

from denso_master.msg import DensoStatusMessage
from std_msgs.msg import UInt8, Bool


DENSO_STATUS_TOPIC_NAME = "/denso_status"
DENSO_SPEED_TOPIC_NAME = "/denso_control/speed"
DENSO_MOTOR_TOPIC_NAME = "/denso_control/motors_enabled"


class SensorCore(QtCore.QThread):
    # ########## create signals for slots ##########
    motor_enabled_stylesheet_change__signal = QtCore.pyqtSignal(str)
    arm_normal_stylesheet_change__signal = QtCore.pyqtSignal(str)
    arm_busy_stylesheet_change__signal = QtCore.pyqtSignal(str)
    error_stylesheet_change__signal = QtCore.pyqtSignal(str)

    arm_speed_change__signal = QtCore.pyqtSignal(str)
    tank_psi_change__signal = QtCore.pyqtSignal(str)

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

        self.motor_enable_button = self.left_screen.motor_enable_button  # type: QtWidgets.QPushButton
        self.motor_disable_button = self.left_screen.motor_disable_button  # type: QtWidgets.QPushButton

        self.arm_set_speed_spinbox = self.left_screen.arm_speed_spinbox  # type: QtWidgets.QSpinBox
        self.arm_set_speed_button = self.left_screen.arm_set_speed_button  # type: QtWidgets.QPushButton

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

        self.speed_publisher = rospy.Publisher(DENSO_SPEED_TOPIC_NAME, UInt8, queue_size=1)
        self.motor_enable_publisher = rospy.Publisher(DENSO_MOTOR_TOPIC_NAME, Bool, queue_size=1)

        self.status_data = None
        self.new_statuses = False

    def run(self):
        while self.run_thread_flag:
            self.update_statuses_if_needed()
            self.msleep(20)

    def on_new_status_update_received(self, data):
        self.status_data = data
        self.new_statuses = True

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

            self.tank_psi_change__signal.emit(str(self.status_data.tank_psi))

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

    def on_motor_enabled_pressed__slot(self):
        self.motor_enable_publisher.publish(1)

    def on_motor_disabled_pressed__slot(self):
        self.motor_enable_publisher.publish(0)

    def on_set_arm_speed_pressed__slot(self):
        self.speed_publisher.publish(self.arm_set_speed_spinbox.value())

    def connect_signals_and_slots(self):
        self.motor_enabled_stylesheet_change__signal.connect(self.motor_enabled_label.setStyleSheet)
        self.arm_normal_stylesheet_change__signal.connect(self.arm_normal_label.setStyleSheet)
        self.arm_busy_stylesheet_change__signal.connect(self.arm_busy_label.setStyleSheet)
        self.error_stylesheet_change__signal.connect(self.arm_error_label.setStyleSheet)

        self.arm_speed_change__signal.connect(self.arm_speed_label.setText)
        self.tank_psi_change__signal.connect(self.tank_psi_label.setText)

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

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False


if __name__ == "__main__":
    rover_statuses = SensorCore()
    rover_statuses.run()
