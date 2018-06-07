#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, UInt8
import time
import random

rospy.init_node("another_test")

pub = rospy.Publisher("/denso_control/relative_joints", Float32MultiArray, queue_size=1)

message = Float32MultiArray()

time.sleep(2)


while not rospy.is_shutdown():


    message.data = (0, 0, 0.25, 0, 0, 0) # Catch

    pub.publish(message)

    time.sleep(3)

