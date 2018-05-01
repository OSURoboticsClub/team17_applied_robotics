# coding:utf-8
#####################################
# Imports
#####################################
# Python native imports
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import qdarkstyle

from Framework.cao_controller import CAOController
from Framework.cao_monitor import CAOMonitor

#####################################
# Global Variables
#####################################
UI_FILE = "Resources/Ui/pendant.ui"


# # CAO Engine
# eng = win32com.client.Dispatch("CAO.CaoEngine")
#
# ctrl = eng.Workspaces(0).AddController("RC", "CaoProv.DENSO.NetwoRC", "", "conn=eth:192.168.1.10")
# Arm1 = ctrl.AddRobot("Arm1", "")
#
# # Get and print a variable
# # p_val_0 = ctrl.AddVariable("P0", "")
# # p_val_1 = ctrl.AddVariable("P1", "")
# # p_val_2 = ctrl.AddVariable("P2", "")
# # print("P0 is: ", p_val_0)
# # print("P1 is: ", p_val_1)
# # print("P2 is: ", p_val_2)
#
# ctrl.Execute("PutAutoMode", 2)  # Put controller into external auto mode
#
# Arm1.Execute("Motor", 1)  # Enable motor power
#
# try:
#     Arm1.Execute("TakeArm", 0)
# except Exception:
#     Arm1.Execute("Motor", 0)
#     exit()
#
# Arm1.Execute("ExtSpeed", (20, 20, 20))
#
# while True:
#     try:
#         Arm1.Move(1, "@P P0", "")
#         Arm1.Move(1, "@P P1", "")
#         Arm1.Move(1, "@P P2", "")
#     except KeyboardInterrupt:
#         Arm1.Execute("GiveArm", )
#         Arm1.Execute("Motor", 0)
#         exit()


#####################################
# ApplicationWindow Class Definition
#####################################
class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None, ui_file_path=None):
        super(ApplicationWindow, self).__init__(parent)

        uic.loadUi(ui_file_path, self)

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


#####################################
# GroundStation Class Definition
#####################################
class GroundStation(QtCore.QObject):
    SCREEN_ID = 0

    exit_requested_signal = QtCore.pyqtSignal()

    start_threads_signal = QtCore.pyqtSignal()
    connect_signals_and_slots_signal = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        # ########## Get the Pick And Plate instance of the logger ##########
        # self.logger = logging.getLogger("groundstation")

        self.shared_objects = {
            "screens": {},
            "regular_classes": {},
            "threaded_classes": {}
        }

        # ###### Instantiate Left And Right Screens ######
        self.shared_objects["screens"]["main_screen"] = \
            self.create_application_window(UI_FILE, "Denso Pendant",
                                           self.SCREEN_ID)  # type: ApplicationWindow

        # ##### Instantiate Regular Classes ######

        # ##### Instantiate Threaded Classes ######
        self.__add_thread("CAO Controller", CAOController(self.shared_objects))
        self.__add_thread("CAO Monitor", CAOMonitor(self.shared_objects))

        self.connect_signals_and_slots_signal.emit()
        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def __add_thread(self, thread_name, instance):
        self.shared_objects["threaded_classes"][thread_name] = instance
        instance.setup_signals(self.start_threads_signal, self.connect_signals_and_slots_signal,
                               self.kill_threads_signal)

    def __connect_signals_to_slots(self):
        self.shared_objects["screens"]["main_screen"].exit_requested_signal.connect(self.on_exit_requested__slot)

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        # Wait for Threads
        for thread in self.shared_objects["threaded_classes"]:
            self.shared_objects["threaded_classes"][thread].wait()

        QtGui.QGuiApplication.exit()

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        app_window = ApplicationWindow(parent=None, ui_file_path=ui_file_path)  # Make a window in this application
        app_window.setWindowTitle(title)  # Sets the window title

        app_window.setWindowFlags(app_window.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

        # app_window.setGeometry(
        #     system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        app_window.show()  # Shows the window in full screen mode

        return app_window


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work properly

    # ########## Start the QApplication Framework ##########
    application = QtWidgets.QApplication(sys.argv)  # Create the ase qt gui application
    application.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

    # ########## Set Organization Details for QSettings ##########
    QtCore.QCoreApplication.setOrganizationName("OSURC")
    QtCore.QCoreApplication.setOrganizationDomain("http://osurobotics.club/")
    QtCore.QCoreApplication.setApplicationName("pendant")

    # ########## Start Ground Station If Ready ##########
    ground_station = GroundStation()
    application.exec_()  # Execute launching of the application