import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal

from Framework.arm_status_sender import ArmStatusSender as ArmStatusSender
from Framework.arm_control_receiver import ArmControlReceiver as ArmControlReceiver


#####################################
# GroundStation Class Definition
#####################################
class NetworkSlave(QtCore.QObject):
    start_threads_signal = QtCore.pyqtSignal()
    connect_signals_and_slots_signal = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(NetworkSlave, self).__init__(parent)

        # ########## Get the Pick And Plate instance of the logger ##########
        # self.logger = logging.getLogger("groundstation")

        self.shared_objects = {
            "regular_classes": {},
            "threaded_classes": {}
        }

        # ##### Instantiate Regular Classes ######

        # ##### Instantiate Threaded Classes ######
        self.__add_thread("Arm Status Sender", ArmStatusSender(self.shared_objects))
        self.__add_thread("Arm Control Receiver", ArmControlReceiver(self.shared_objects))

        self.connect_signals_and_slots_signal.emit()
        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def __add_thread(self, thread_name, instance):
        self.shared_objects["threaded_classes"][thread_name] = instance
        instance.setup_signals(self.start_threads_signal, self.connect_signals_and_slots_signal,
                               self.kill_threads_signal)

    def __connect_signals_to_slots(self):
        pass

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        # Wait for Threads
        for thread in self.shared_objects["threaded_classes"]:
            self.shared_objects["threaded_classes"][thread].wait()

        QtCore.QCoreApplication.exit()


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work properly

    # ########## Start the QApplication Framework ##########
    application = QtCore.QCoreApplication(sys.argv)  # Create the ase qt gui application

    # ########## Set Organization Details for QSettings ##########
    QtCore.QCoreApplication.setOrganizationName("OSURC")
    QtCore.QCoreApplication.setOrganizationDomain("http://osurobotics.club/")
    QtCore.QCoreApplication.setApplicationName("denso_slave")

    # ########## Start Ground Station If Ready ##########
    network_slave = NetworkSlave()
    application.exec_()  # Execute launching of the application