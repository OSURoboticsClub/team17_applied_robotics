#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep


#####################################
# Global Variables
#####################################
NODE_NAME = "robot_control_network_master"


DEFAULT_HERTZ = 30


#####################################
# ControlCoordinator Class Definition
#####################################
class DriveCoordinator(object):
    def __init__(self):

        rospy.init_node(NODE_NAME)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)


        # ########## Run the Class ##########
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            start_time = time()

            print "running"

            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))



if __name__ == '__main__':
    DriveCoordinator()

