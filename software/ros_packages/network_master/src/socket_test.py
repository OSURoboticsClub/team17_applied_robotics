import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("ano")

pub = rospy.Publisher("/denso_control/relative_joints", Float32MultiArray, queue_size=1)

message = Float32MultiArray()

message.data = (20, 0, 0, 0, 0, 0)

pub.publish(message)
