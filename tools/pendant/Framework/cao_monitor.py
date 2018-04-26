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
THREAD_HERTZ = 100


#####################################
# Controller Class Definition
#####################################
class CAOMonitor(QtCore.QThread):
    j1_set__signal = QtCore.pyqtSignal(float)
    j2_set__signal = QtCore.pyqtSignal(float)
    j3_set__signal = QtCore.pyqtSignal(float)
    j4_set__signal = QtCore.pyqtSignal(float)
    j5_set__signal = QtCore.pyqtSignal(float)
    j6_set__signal = QtCore.pyqtSignal(float)

    def __init__(self, shared_objects):
        super(CAOMonitor, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.main_screen = self.shared_objects["screens"]["main_screen"]
        self.motor_power_label = self.main_screen.motor_power_label  # type: QtWidgets.QLabel

        self.max_speed_label = self.main_screen.max_speed_label  # type: QtWidgets.QLabel\

        self.j1_spinbox = self.main_screen.j1_spinbox  # type: QtWidgets.QSpinBox
        self.j2_spinbox = self.main_screen.j2_spinbox  # type: QtWidgets.QSpinBox
        self.j3_spinbox = self.main_screen.j3_spinbox  # type: QtWidgets.QSpinBox
        self.j4_spinbox = self.main_screen.j4_spinbox  # type: QtWidgets.QSpinBox
        self.j5_spinbox = self.main_screen.j5_spinbox  # type: QtWidgets.QSpinBox
        self.j6_spinbox = self.main_screen.j6_spinbox  # type: QtWidgets.QSpinBox

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

            self.j1_set__signal.emit(j1)
            self.j2_set__signal.emit(j2)
            self.j3_set__signal.emit(j3)
            self.j4_set__signal.emit(j4)
            self.j5_set__signal.emit(j5)
            self.j6_set__signal.emit(j6)

            self.max_speed_label.setText(str(self.arm_speed_status.Value))

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def connect_signals_and_slots(self):
        self.j1_set__signal.connect(self.j1_spinbox.setValue)
        self.j2_set__signal.connect(self.j2_spinbox.setValue)
        self.j3_set__signal.connect(self.j3_spinbox.setValue)
        self.j4_set__signal.connect(self.j4_spinbox.setValue)
        self.j5_set__signal.connect(self.j5_spinbox.setValue)
        self.j6_set__signal.connect(self.j6_spinbox.setValue)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False