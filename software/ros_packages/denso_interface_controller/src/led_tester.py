#!/usr/bin/env python
import rospy
import random
import time
from std_msgs.msg import UInt8MultiArray

STATUSES_TOPIC = "denso_led_controller/control"

rospy.init_node("tester_led")

publisher = rospy.Publisher(STATUSES_TOPIC, UInt8MultiArray, queue_size=1)

message = UInt8MultiArray()

i = 0

while i < 1000:
    message.data = [random.randint(0, 255) for _ in range(23*3)]

    publisher.publish(message)

    # time.sleep(0.01)
    i += 1
