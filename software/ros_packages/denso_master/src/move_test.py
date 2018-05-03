#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, UInt8
import time
import random

rospy.init_node("another_test")

pub = rospy.Publisher("/denso_control/relative_joints", Float32MultiArray, queue_size=10)
speed_pub = rospy.Publisher("/denso_control/speed", UInt8, queue_size=10)

message = Float32MultiArray()
speed = UInt8()

while not rospy.is_shutdown():


    message.data = (-20.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    pub.publish(message)

    message.data = (20.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    pub.publish(message)

    time.sleep(1)