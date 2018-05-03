#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import pythoncom
import win32com.client
from time import time
import socket
import json

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 100

P0 = (216.1302490234375, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P1 = (251.22869873046875, -9.575998306274414, 572.6145629882812, 63.89561462402344, 8.09478759765625, 83.43250274658203)
P2 = (216.1302490234375, 0.10808953642845154, 606.7885131835938, 63.89561462402344, 8.09478759765625, 83.43250274658203)

J0 = (-2.4949951171875, -68.55029296875, 161.4649658203125, 0.2345581203699112, -40.739683151245117, 60.7391586303711)

BAD_VAL = -1000000

TCP_PORT = 9877


#####################################
# Controller Class Definition
#####################################
class RAWControlReceiver(QtCore.QThread):

    new_message__signal = QtCore.pyqtSignal(dict)

    def __init__(self):
        super(RAWControlReceiver, self).__init__()
        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.control_tcp_server = None
        self.client_connection = None
        self.client_address = None

        self.current_message = ""

        self.num_messages = 0
        self.last_time = time()

    def run(self):
        self.initialize_tcp_server()
        while self.run_thread_flag:
            self.check_for_new_command_message()
            # self.msleep(2)

    def initialize_tcp_server(self):
        self.control_tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_tcp_server.bind(('', TCP_PORT))
        self.control_tcp_server.listen(5)

    def check_for_new_command_message(self):
        try:
            self.current_message += self.client_connection.recv(8)

            found_pound = self.current_message.find("#####")

            if found_pound != -1:
                split_message = str(self.current_message[:found_pound])

                self.current_message = self.current_message[found_pound + 5:]

                try:
                    json_message = json.loads(split_message)

                    self.num_messages += 1

                    if time() - self.last_time > 1:
                        print "Num commands received:", self.num_messages
                        self.num_messages = 0
                        self.last_time = time()

                    self.new_message__signal.emit(json_message)
                except Exception, e:
                    print e, "could not parse"
        except Exception, e:
            print e, "other"
            self.client_connection, self.client_address = self.control_tcp_server.accept()


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

        # ########## Get the settings instance ##########
        self.message_handler = RAWControlReceiver()
        self.message_handler.start()

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.control_tcp_server = None
        self.client_connection = None
        self.client_address = None

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

        self.current_message = ""

        self.command_queue = []

        self.num_commands = 0

        self.last_commands_time = time()

    def run(self):
        self.initialize_cao_engine()

        while self.run_thread_flag:
            start_time = time()

            # self.add_item_to_command_queue({"move_joint_rel": (10, 0, 0, 0, 0, 0)})
            # self.add_item_to_command_queue({"move_joint_rel": (-10, 0, 0, 0, 0, 0)})

            self.process_command_queue_item()

            if time() - self.last_commands_time > 1:
                print "Num commands processed:", self.num_commands
                self.num_commands = 0
                self.last_commands_time = time()

            time_diff = time() - start_time
            self.msleep(max(int(self.wait_time - time_diff), 0))

    def initialize_cao_engine(self):
        pythoncom.CoInitialize()
        self.cao_engine = win32com.client.Dispatch("CAO.CaoEngine")
        self.controller = self.cao_engine.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
        self.arm = self.controller.AddRobot("Arm1", "")

    def on_new_message__signal(self, message):
        self.command_queue.append(message)

    def process_command_queue_item(self):
        if self.command_queue:
            key = list(self.command_queue[0].keys())[0]
            data = self.command_queue[0][key]

            del self.command_queue[0]

            command_to_run = self.CONTROL_COMMANDS.get(key)
            command_to_run(data)

            self.num_commands += 1

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
        try:
            if self.status_sender_class.statuses["motor_enabled"]:
                self.arm.Move(1, "@P " + str(tuple(position)), "NEXT")
        except:
            pass

    def move_arm_position_relative(self, position_offsets):
        current_position = self.status_sender_class.position

        if current_position["rz"] == BAD_VAL or len(position_offsets) == position_offsets.count(0):
            return

        new_position = (
            current_position["x"] + position_offsets[0],
            current_position["y"] + position_offsets[1],
            current_position["z"] + position_offsets[2],
            current_position["rx"] + position_offsets[3],
            current_position["ry"] + position_offsets[4],
            current_position["rz"] + position_offsets[5],
        )
        # print "here"
        self.move_arm_position_absolute(new_position)

    def move_joints_absolute(self, joint_positions):
        try:
            if self.status_sender_class.statuses["motor_enabled"]:
                self.arm.Move(1, "J" + str(tuple(joint_positions)), "NEXT")
        except:
            pass

    def move_joints_relative(self, joint_position_offsets):
        current_position = self.status_sender_class.joints

        if current_position[6] == BAD_VAL or len(joint_position_offsets) == joint_position_offsets.count(0):
            return

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
        self.message_handler.new_message__signal.connect(self.on_new_message__signal)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.message_handler.run_thread_flag = False
        self.message_handler.wait()
        self.run_thread_flag = False
