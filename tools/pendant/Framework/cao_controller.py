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

P0 = (216.1302490234375, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P0_2 = (220.1302490234375, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P1 = (251.22869873046875, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P2 = (216.1302490234375, 0.10808953642845154, 606.7885131835938, 63.89561462402344, 8.09478759765625, 83.43250274658203)


#####################################
# Controller Class Definition
#####################################
class CAOController(QtCore.QThread):
    j1_set__signal = QtCore.pyqtSignal(float)
    j2_set__signal = QtCore.pyqtSignal(float)
    j3_set__signal = QtCore.pyqtSignal(float)
    j4_set__signal = QtCore.pyqtSignal(float)
    j5_set__signal = QtCore.pyqtSignal(float)
    j6_set__signal = QtCore.pyqtSignal(float)

    def __init__(self, shared_objects):
        super(CAOController, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.main_screen = self.shared_objects["screens"]["main_screen"]
        self.motor_power_label = self.main_screen.motor_power_label  # type: QtWidgets.QLabel

        self.control_mode_pushbutton = self.main_screen.control_mode_stored_points_radiobutton  # type: QtWidgets.QRadioButton
        self.control_mode_pushbutton.setChecked(True)

        self.move_p0_pushbutton = self.main_screen.movep0_pushbutton  # type: QtWidgets.QPushButton
        self.move_p1_pushbutton = self.main_screen.movep1_pushbutton  # type: QtWidgets.QPushButton
        self.move_p2_pushbutton = self.main_screen.movep2_pushbutton  # type: QtWidgets.QPushButton

        self.max_speed_label = self.main_screen.max_speed_label  # type: QtWidgets.QLabel
        self.speed_limit_spinbox = self.main_screen.speed_limit_spinbox  # type: QtWidgets.QSpinBox
        self.speed_limit_apply_pushbutton = self.main_screen.speed_limit_apply_pushbutton  # type: QtWidgets.QPushButton

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

        self.command_queue = []

        self.enable_motors = False

        self.set_motor_speed = True
        self.new_motor_speed = 5

        self.move_p0 = False
        self.move_p1 = False
        self.move_p2 = False

        self.arm_last_power_status = -1

    def run(self):
        pythoncom.CoInitialize()
        self.cao_engine = win32com.client.Dispatch("CAO.CaoEngine")
        self.controller = self.cao_engine.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
        self.arm = self.controller.AddRobot("Arm1", "")

        # p_val_0 = self.controller.AddVariable("P0", "")
        self.arm_power_status = self.arm.AddVariable("@SERVO_ON", "")
        self.arm_speed_status = self.arm.AddVariable("@EXTSPEED", "")
        self.arm_current_position = self.arm.AddVariable("@CURRENT_POSITION", "")
        # self.arm_current_joint_states = self.arm.AddVariable("@CURJNT", "")

        temp = 1

        while self.run_thread_flag:

            start_time = time()
            # print(self.arm_current_joint_states.Value)
            test = list(self.arm_current_position.Value)
            #
            # self.j1_set__signal.emit(j1)
            # self.j2_set__signal.emit(j2)
            # self.j3_set__signal.emit(j3)
            # self.j4_set__signal.emit(j4)
            # self.j5_set__signal.emit(j5)
            # self.j6_set__signal.emit(j6)

            self.max_speed_label.setText(str(self.arm_speed_status.Value))

            if self.arm_power_status.Value != self.arm_last_power_status:
                if self.arm_power_status.Value:
                    self.motor_power_label.setText("MOTOR POWER ON")
                    self.motor_power_label.setStyleSheet("background-color: green;")
                else:
                    self.motor_power_label.setText("MOTOR POWER OFF")
                    self.motor_power_label.setStyleSheet("background-color: darkred;")

                self.arm_last_power_status = self.arm_power_status.Value

            if self.enable_motors:
                if not self.arm_power_status.Value:
                    self.arm.Execute("Motor", 1)  # Enable motor power
                    self.arm.Execute("TakeArm", 0)
                else:
                    self.arm.Execute("Motor", 0)  # Enable motor power

                self.enable_motors = False

            if self.set_motor_speed:
                self.arm.Execute("ExtSpeed", (self.new_motor_speed, self.new_motor_speed, self.new_motor_speed))
                self.set_motor_speed = False

            if self.move_p0:
                test[0] += 10
                print(test)
                self.arm.Move(1, str(tuple(test)), "")
                self.move_p0 = False

            if self.move_p1:
                self.arm.Move(1, str(P0_2), "")
                self.move_p1 = False

            if self.move_p2:
                self.arm.Move(1, str(P2), "")
                self.move_p2 = False

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def toggle_motor_power__slot(self, _):
        self.enable_motors = True

    def set_motor_speed__slot(self):
        self.set_motor_speed = True
        self.new_motor_speed = self.speed_limit_spinbox.value()

    def move_p0__slot(self):
        self.move_p0 = True

    def move_p1__slot(self):
        self.move_p1 = True

    def move_p2__slot(self):
        self.move_p2 = True

    def connect_signals_and_slots(self):
        self.motor_power_label.mousePressEvent = self.toggle_motor_power__slot

        self.speed_limit_apply_pushbutton.clicked.connect(self.set_motor_speed__slot)

        self.move_p0_pushbutton.clicked.connect(self.move_p0__slot)
        self.move_p1_pushbutton.clicked.connect(self.move_p1__slot)
        self.move_p2_pushbutton.clicked.connect(self.move_p2__slot)

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