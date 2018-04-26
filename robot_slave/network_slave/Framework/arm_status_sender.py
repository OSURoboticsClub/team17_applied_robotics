#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import pythoncom
import win32com.client
import time
from time import time

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 10


#####################################
# Controller Class Definition
#####################################
class ArmStatusSender(QtCore.QThread):
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

        self.arm_power_status = None
        self.arm_speed_status = None
        self.arm_current_position = None

    def run(self):
        pythoncom.CoInitialize()
        self.cao_engine = win32com.client.Dispatch("CAO.CaoEngine")
        self.controller = self.cao_engine.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
        self.arm = self.controller.AddRobot("Arm1", "")

        self.arm_power_status = self.arm.AddVariable("@SERVO_ON", "")
        self.arm_speed_status = self.arm.AddVariable("@EXTSPEED", "")
        self.arm_current_position = self.arm.AddVariable("@CURRENT_POSITION", "")

        while self.run_thread_flag:

            start_time = time()
            j1, j2, j3, j4, j5, j6, _ = self.arm_current_position.Value

            print("sender", self.arm_current_position.Value)

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
