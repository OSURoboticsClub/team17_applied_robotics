#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from std_msgs.msg import Float32MultiArray

#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Logitech Logitech Extreme 3D Pro"

DEFAULT_ARM_COMMAND_TOPIC = "/denso_control/relative_joints"

DRIVE_COMMAND_HERTZ = 15

Z_AXIS_DEADBAND = 0.2
Y_AXIS_DEADBAND = 0.05
X_AXIS_DEADBAND = 0.05

THROTTLE_MIN = 0.05

PAUSE_STATE_CHANGE_TIME = 0.5

CAMERA_CHANGE_TIME = 0.2
GUI_ELEMENT_CHANGE_TIME = 0.2
CAMERA_TOGGLE_CHANGE_TIME = 0.35


#####################################
# Controller Class Definition
#####################################
class LogitechJoystick(QtCore.QThread):
    def __init__(self):
        super(LogitechJoystick, self).__init__()

        # ########## Thread Flags ##########
        self.run_thread_flag = True
        self.setup_controller_flag = True
        self.data_acquisition_and_broadcast_flag = True
        self.controller_acquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "x_axis": 512,
            "y_axis": 512,
            "z_axis": 128,
            "throttle_axis": 128,

            "hat_x_axis": 0,
            "hat_y_axis": 0,

            "trigger_pressed": 0,
            "thumb_pressed": 0,
            "three_pressed": 0,
            "four_pressed": 0,
            "five_pressed": 0,
            "six_pressed": 0,

            "seven_pressed": 0,
            "eight_pressed": 0,
            "nine_pressed": 0,
            "ten_pressed": 0,
            "eleven_pressed": 0,
            "twelve_pressed": 0,
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "x_axis",
            "ABS_Y": "y_axis",
            "ABS_RZ": "z_axis",
            "ABS_THROTTLE": "throttle_axis",

            "ABS_HAT0X": "hat_x_axis",
            "ABS_HAT0Y": "hat_y_axis",

            "BTN_TRIGGER": "trigger_pressed",
            "BTN_THUMB": "thumb_pressed",
            "BTN_THUMB2": "three_pressed",
            "BTN_TOP": "four_pressed",
            "BTN_TOP2": "five_pressed",
            "BTN_PINKIE": "six_pressed",

            "BTN_BASE": "seven_pressed",
            "BTN_BASE2": "eight_pressed",
            "BTN_BASE3": "nine_pressed",
            "BTN_BASE4": "ten_pressed",
            "BTN_BASE5": "eleven_pressed",
            "BTN_BASE6": "twelve_pressed",
        }

        self.ready = False

        self.start()

    def run(self):

        while self.run_thread_flag:
            if self.setup_controller_flag:
                self.controller_acquired = self.__setup_controller()
                self.setup_controller_flag = False
            if self.data_acquisition_and_broadcast_flag:
                self.__get_controller_data()

    def __setup_controller(self):
        for device in devices.gamepads:
            if device.name == GAME_CONTROLLER_NAME:
                self.gamepad = device

                return True
        return False

    def __get_controller_data(self):
        if self.controller_acquired:
            events = self.gamepad.read()

            for event in events:
                if event.code in self.raw_mapping_to_class_mapping:
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state
                    # print event.code

            self.ready = True


#####################################
# Controller Class Definition
#####################################
class JoystickControlSender(QtCore.QThread):
    def __init__(self, shared_objects):
        super(JoystickControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.joystick = LogitechJoystick()

        # ########## Class Variables ##########
        # Publishers
        self.arm_command_publisher = rospy.Publisher(DEFAULT_ARM_COMMAND_TOPIC, Float32MultiArray, queue_size=1)

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

        self.drive_paused = True

        self.last_pause_state_time = time()
        self.last_gui_element_change_time = time()
        self.last_camera_change_time = time()
        self.last_camera_toggle_time = time()

    def run(self):
        while self.run_thread_flag:

            start_time = time()

            self.check_and_set_pause_state()
            self.__update_and_publish()

            time_diff = time() - start_time
            self.msleep(max(int((self.wait_time - time_diff) * 1000), 0))

    def connect_signals_and_slots(self):
        pass

    def check_and_set_pause_state(self):
        thumb_pressed = self.joystick.controller_states["thumb_pressed"]
        if thumb_pressed and (time() - self.last_pause_state_time) > PAUSE_STATE_CHANGE_TIME:
            self.drive_paused = not self.drive_paused
            self.last_pause_state_time = time()

    def __update_and_publish(self):
        self.publish_drive_command()

    def publish_drive_command(self):
        if self.drive_paused:
            pass

        else:
            arm_message = self.get_control_message()
            self.arm_command_publisher.publish(arm_message)

        # print arm_message
        # self.arm_command_publisher.publish(arm_message)

    def get_control_message(self):
        control_message = Float32MultiArray()

        multiplier = 1.75

        j1 = self.joystick.controller_states["eleven_pressed"]
        j2 = self.joystick.controller_states["twelve_pressed"]
        j3 = self.joystick.controller_states["nine_pressed"]
        j4 = self.joystick.controller_states["ten_pressed"]
        j5 = self.joystick.controller_states["seven_pressed"]
        j6 = self.joystick.controller_states["eight_pressed"]

        y_axis = multiplier * (-(self.joystick.controller_states["y_axis"] - 512) / 512.0)

        if abs(y_axis) < Y_AXIS_DEADBAND:
            y_axis = 0

        control_message.data = (
            y_axis if j1 else 0,
            y_axis if j2 else 0,
            y_axis if j3 else 0,
            y_axis if j4 else 0,
            y_axis if j5 else 0,
            y_axis if j6 else 0,
        )

        return control_message

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
