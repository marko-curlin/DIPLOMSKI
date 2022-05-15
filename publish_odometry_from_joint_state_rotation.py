import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import statistics

import util

odom_publisher: rospy.Publisher
first_callback = True
start_wheel_position = 0, 0, 0, 0

WHEEL_CONSTANTS = -0.000157125245834, -0.000157137139815, -0.000157141608, -0.000157132896103


def convert_joint_state_to_odometry(joint_state_data):
    rospy.logdebug("Received data from /viv/viv_epos_driver/joint_state")
    global first_callback, start_wheel_position
    if first_callback:
        start_wheel_position = joint_state_data.position
        first_callback = False

    # global position, velocity
    # position = joint_state_data.position
    # velocity = joint_state_data.velocity
    yaw = statistics.mean(
        (wheel_constant * wheel_position for wheel_constant, wheel_position in zip(WHEEL_CONSTANTS, joint_state_data.position))
    )
    odometry = Odometry()
    odometry.pose.pose.orientation = util.yaw_to_quaternion(yaw)  # might not work as the function returns np.array
    odom_publisher.publish(odometry)


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('joint_state_to_odometry_conversion')

    odom_publisher = rospy.Publisher('/viv/marko/odom', Odometry, queue_size=None)
    rospy.Subscriber('/viv/viv_epos_driver/joint_state', JointState, convert_joint_state_to_odometry)

    rospy.spin()
