import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import statistics

odom_publisher: rospy.Publisher

WHEEL_CONSTANTS = 0.000132233552078, -0.00013074184131, -0.000132463427028, 0.000130776432695


def convert_joint_state_to_odometry(joint_state_data):
    rospy.logdebug("Received data from /viv/viv_epos_driver/joint_state")
    # global position, velocity
    # position = joint_state_data.position
    # velocity = joint_state_data.velocity
    pose_point = Point()
    pose_point.x = statistics.mean(
        (wheel_constant * wheel_position for wheel_constant, wheel_position in zip(WHEEL_CONSTANTS, joint_state_data.position))
    )
    odometry = Odometry()
    odometry.pose.pose.position = pose_point
    odom_publisher.publish(odometry)


if __name__ == '__main__':
    # Starts a new node
    rospy.init_node('joint_state_to_odometry_conversion')

    odom_publisher = rospy.Publisher('/viv/marko/odom', Odometry, queue_size=None)
    rospy.Subscriber('/viv/viv_epos_driver/joint_state', JointState, convert_joint_state_to_odometry)

    rospy.spin()
