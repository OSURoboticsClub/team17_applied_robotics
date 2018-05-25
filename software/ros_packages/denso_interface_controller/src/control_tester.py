#!/usr/bin/env python
import rospy
import random
import time
from denso_interface_controller.msg import InterfaceControlMessage
from std_msgs.msg import UInt8MultiArray

STATUSES_TOPIC = "denso_interface_controller/control"

rospy.init_node("tester")

publisher = rospy.Publisher(STATUSES_TOPIC, InterfaceControlMessage, queue_size=1)

message = InterfaceControlMessage()
i = 0

time.sleep(2)

# while i < 10:
#     message.delay_before_processing = 1.0
#
#     message.set_pressure = random.randint(0, 50)
#     message.should_fire = 1 if random.randint(0, 20) == 0 else 0
#     message.should_tamp = 1 if random.randint(0, 1) == 0 else 0
#     message.delay_before_processing = random.randint(0, 1)
#
#     publisher.publish(message)
#
#     time.sleep(1)
#     i += 1

message.set_pressure = 0
message.should_fire = 0

publisher.publish(message)

# time.sleep(10)
#
#
# message.set_pressure = 0
# message.should_fire = 1
#
# publisher.publish(message)

input("Press Enter")

