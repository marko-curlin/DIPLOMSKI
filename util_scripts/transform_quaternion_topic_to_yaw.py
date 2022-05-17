import os
import sys

p = os.path.abspath('.')
sys.path.insert(1, p)

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import util
import math


def convert_quaternion_to_yaw(odom_data):
    rospy.logdebug("Received data from odom topic")

    yaw = Float64()
    yaw.data = math.degrees(util.get_yaw_from_pose_message(odom_data.pose.pose))
    yaw_publisher.publish(yaw)


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('quaternion_to_yaw', log_level=rospy.INFO)

    yaw_publisher = rospy.Publisher('/yaw', Float64, queue_size=None)
    rospy.Subscriber('/T265/camera/odom/sample', Odometry, convert_quaternion_to_yaw)

    rospy.spin()
