#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import pythoncom
import win32com.client
import time
from time import time
import json

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 10


#####################################
# Controller Class Definition
#####################################
class ArmStatusSender(QtCore.QThread):

    position = {
        "x": 0,
        "y": 0,
        "z": 0,
        "rx": 0,
        "ry": 0,
        "rz": 0,
        "fig": 0
    }

    joints = {
        1: 0,
        2: 0,
        3: 0,
        4: 0,
        5: 0,
        6: 0
    }

    statuses = {
        "motor_enabled": True,
        "arm_normal": False,
        "error": "",

        "speed": 0,
        "arm_busy": True,

        "tank_psi": 0
    }

    def __init__(self, shared_objects):
        super(ArmStatusSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.cao_engine = None
        self.controller = None
        self.arm = None

        self.robot_enabled = None
        self.robot_normal = None
        self.robot_error = None
        self.robot_busy = None
        self.robot_speed = None

        self.robot_current_position = None
        self.robot_current_joint_angles = None

    def run(self):
        self.initialize_cao_engine_and_watchers()

        while self.run_thread_flag:
            start_time = time()

            self.get_statuses()
            self.get_position()
            self.get_joint_angles()

            self.send_status_package()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def initialize_cao_engine_and_watchers(self):
        pythoncom.CoInitialize()
        self.cao_engine = win32com.client.Dispatch("CAO.CaoEngine")
        self.controller = self.cao_engine.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
        self.arm = self.controller.AddRobot("Arm1", "")

        self.robot_enabled = self.arm.AddVariable("@SERVO_ON", "")
        self.robot_normal = self.controller.AddVariable("@NORMAL_STATUS", "")
        self.robot_error = self.controller.AddVariable("@ERROR_DESCRIPTION", "")
        self.robot_busy = self.arm.AddVariable("@BUSY_STATUS", "")

        self.robot_speed = self.arm.AddVariable("@EXTSPEED", "")
        self.robot_current_position = self.arm.AddVariable("@CURRENT_POSITION", "")
        self.robot_current_joint_angles = self.arm.AddVariable("@CURRENT_ANGLE", "")

    def get_statuses(self):
        motor_on = self.robot_enabled.Value
        normal = self.robot_normal.Value
        error = self.robot_error.Value
        busy = self.robot_busy.Value
        robot_speed = self.robot_speed.Value

        self.statuses = {
            "motor_enabled": motor_on,
            "arm_normal": normal,
            "error": error,

            "speed": robot_speed,
            "arm_busy": busy,

            "tank_psi": 0
        }

    def get_position(self):
        x, y, z, rx, ry, rz, fig = self.robot_current_position.Value
        self.position = {
            "x": x,
            "y": y,
            "z": z,
            "rx": rx,
            "ry": ry,
            "rz": rz,
            "fig": fig
        }

    def send_status_package(self):
        package = {
            "statuses": self.statuses,
            "position": self.position,
            "joints": self.joints
        }

        # print(json.dumps(package))

    def get_joint_angles(self):
        j1, j2, j3, j4, j5, j6, _, _ = self.robot_current_joint_angles.Value

        self.joints = {
            1: j1,
            2: j2,
            3: j3,
            4: j4,
            5: j5,
            6: j6
        }
        # print(self.robot_current_joint_angles.Value)

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
