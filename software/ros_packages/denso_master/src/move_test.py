#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, UInt8
import time
import random

rospy.init_node("another_test")

pub = rospy.Publisher("/denso_control/absolute_joints", Float32MultiArray, queue_size=10)

message = Float32MultiArray()

while not rospy.is_shutdown():


    message.data = (0, -70, 70, 0, 90, -5) # Catch

    pub.publish(message)

    time.sleep(3)

    message.data = (0, -30, 135, 0, 29, -5) # Fire

    pub.publish(message)

    time.sleep(3)
