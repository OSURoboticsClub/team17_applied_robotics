#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from time import time
import cv2
import qimage2ndarray
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras/"
EXCLUDED_CAMERAS = ["zed"]

PRIMARY_LABEL_MAX = (640, 360)
SECONDARY_LABEL_MAX = (640, 360)
TERTIARY_LABEL_MAX = (640, 360)

GUI_SELECTION_CHANGE_TIMEOUT = 3  # Seconds

STYLESHEET_SELECTED = "border: 2px solid orange; background-color:black;"
STYLESHEET_UNSELECTED = "background-color:black;"


#####################################
# RoverVideoCoordinator Class Definition
#####################################
class RoverVideoCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(RoverVideoCoordinator, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.video_label = self.right_screen.video_label  # type:QtWidgets.QLabel
        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.camera_subscriber = rospy.Subscriber("/zed/right/image_raw_color/compressed", CompressedImage, self.__image_data_received_callback)

        self.bridge = CvBridge()
        self.raw_image = None
        self.pixmap = None
        self.new_frame = False

    def run(self):
        while self.run_thread_flag:
            self.__show_video_enabled()
            self.msleep(10)

    def __show_video_enabled(self):
        if self.new_frame:
            try:
                opencv_image = self.bridge.compressed_imgmsg_to_cv2(self.raw_image, "rgb8")

                self.__create_final_pixmaps(opencv_image)
            except Exception, error:
                print "Failed with error:" + str(error)

            self.new_frame = False

    def __image_data_received_callback(self, raw_image):
        self.raw_image = raw_image
        self.new_frame = True

    def __create_final_pixmaps(self, opencv_image):
        height, width, _ = opencv_image.shape

        opencv_image_sized = cv2.resize(opencv_image, (1920, 1080))

        self.pixmap = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(opencv_image_sized))
        self.pixmap_ready_signal.emit()

    def __on_pixmap_ready__slot(self):
        try:
            self.video_label.setPixmap(self.pixmap)
        except:
            print "text"

    def connect_signals_and_slots(self):
        self.pixmap_ready_signal.connect(self.__on_pixmap_ready__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)


    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
