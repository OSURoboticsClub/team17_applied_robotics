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

P0 = (216.1302490234375, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P1 = (251.22869873046875, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P2 = (216.1302490234375, 0.10808953642845154, 606.7885131835938, 63.89561462402344, 8.09478759765625, 83.43250274658203)

J0 = (-2.4949951171875, -68.55029296875, 161.4649658203125, 0.2345581203699112, -40.739683151245117, 60.7391586303711)


#####################################
# Controller Class Definition
#####################################
class ArmControlReceiver(QtCore.QThread):
    def __init__(self, shared_objects):
        super(ArmControlReceiver, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.status_sender_class = self.shared_objects["threaded_classes"]["Arm Status Sender"]

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.cao_engine = None
        self.controller = None
        self.arm = None

        self.CONTROL_COMMANDS = {
            "enable_motors": self.enable_motors,
            "change_robot_speed": self.change_robot_speed,
            "move_position_abs": self.move_arm_position_absolute,
            "move_position_rel": self.move_arm_position_relative,
            "move_joint_abs": self.move_joints_absolute,
            "move_joint_rel": self.move_joints_relative,

            "charge_tank_psi": 0,
            "fire_tank": 0
        }

        self.command_queue = []

    def run(self):
        self.initialize_cao_engine()
        self.add_item_to_command_queue({"enable_motors": True})
        self.add_item_to_command_queue({"change_robot_speed": 90})
        self.add_item_to_command_queue({"move_position_abs": P1})
        self.add_item_to_command_queue({"move_position_abs": P0})
        self.add_item_to_command_queue({"move_position_abs": P2})
        self.add_item_to_command_queue({"move_joint_abs": J0})

        temp = 0

        while self.run_thread_flag:
            start_time = time()

            if temp:
                self.add_item_to_command_queue({"move_joint_rel": (35, 10, 10, 10, 20, 45)})
                temp = 0
            else:
                self.add_item_to_command_queue({"move_joint_rel": (-35, -10, -10, -10, -20, -45)})
                temp = 1

            self.process_command_queue_item()

            time_diff = time() - start_time
            self.msleep(max(int(self.wait_time - time_diff), 0))

    def initialize_cao_engine(self):
        pythoncom.CoInitialize()
        self.cao_engine = win32com.client.Dispatch("CAO.CaoEngine")
        self.controller = self.cao_engine.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
        self.arm = self.controller.AddRobot("Arm1", "")

    def process_command_queue_item(self):
        if self.command_queue:
            key = list(self.command_queue[0].keys())[0]
            data = self.command_queue[0][key]

            del self.command_queue[0]

            command_to_run = self.CONTROL_COMMANDS.get(key)
            command_to_run(data)

    def add_item_to_command_queue(self, item):
        self.command_queue.append(item)

    def enable_motors(self, should_enable):
        try:
            if should_enable:
                self.arm.Execute("Motor", 1)
                self.arm.Execute("TakeArm", 0)
            else:
                self.arm.Execute("Motor", 0)
                self.arm.Execute("GiveArm", 0)
        except:
            print("Arm not able to change to state", "on." if should_enable else "off.")

    def change_robot_speed(self, speed):
        self.arm.Execute("ExtSpeed", (speed, speed, speed))

    def move_arm_position_absolute(self, position):
        self.arm.Move(1, "@P " + str(tuple(position)), "")

    def move_arm_position_relative(self, position_offsets):
        current_position = self.status_sender_class.position

        new_position = (
            current_position["x"] + position_offsets[0],
            current_position["y"] + position_offsets[1],
            current_position["z"] + position_offsets[2],
            current_position["rx"] + position_offsets[3],
            current_position["ry"] + position_offsets[4],
            current_position["rz"] + position_offsets[5],
        )

        self.move_arm_position_absolute(new_position)

    def move_joints_absolute(self, joint_positions):
        self.arm.Move(1, "J" + str(joint_positions))

    def move_joints_relative(self, joint_position_offsets):
        current_position = self.status_sender_class.joints

        new_joint_positions = (
            current_position[1] + joint_position_offsets[0],
            current_position[2] + joint_position_offsets[1],
            current_position[3] + joint_position_offsets[2],
            current_position[4] + joint_position_offsets[3],
            current_position[5] + joint_position_offsets[4],
            current_position[6] + joint_position_offsets[5],
        )

        self.move_joints_absolute(new_joint_positions)

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
