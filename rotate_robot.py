from math import radians, degrees
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
import util

start_position = 0, 0, 0, 0
position = 0, 0, 0, 0
velocity = 0, 0, 0, 0
yaw = 0
start_yaw = 0
velocity_publisher: rospy.Publisher


def odom_callback(odom_data):
    # rospy.logdebug("Received data from /T265/camera/odom/sample")
    global yaw
    pose = odom_data.pose.pose  # first pose is PoseWithCovariance
    yaw = util.get_yaw_from_pose_message(pose)


def joint_state_callback(joint_state_data):
    # rospy.logdebug("Received data from /viv/viv_epos_driver/joint_state")
    global position, velocity
    position = joint_state_data.position
    velocity = joint_state_data.velocity


def shutdown_func():
    vel_msg = Twist()

    # Once shutdown, stops the robot
    vel_msg.angular.z = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)

    rospy.sleep(1)
    # The important bits
    rospy.loginfo(f"Final yaw (deg): {degrees(yaw)}")
    rospy.loginfo(f"Final change in yaw (deg): {degrees(yaw - start_yaw)}")
    rospy.loginfo(f"Final position: {position}")
    rospy.loginfo(f"Final change in position of motor 1: {position[0] - start_position[0]}")
    rospy.loginfo(f"Final change in position of motor 2: {position[1] - start_position[1]}")
    rospy.loginfo(f"Final change in position of motor 3: {position[2] - start_position[2]}")
    rospy.loginfo(f"Final change in position of motor 4: {position[3] - start_position[3]}")


def rotate():
    global velocity_publisher, start_position, start_yaw

    # Starts a new node
    rospy.init_node('rotate')

    rospy.on_shutdown(shutdown_func)

    velocity_publisher = rospy.Publisher('/viv/marko/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_epos_driver/joint_state', JointState, joint_state_callback)
    rospy.Subscriber('/T265/camera/odom/sample', Odometry, odom_callback)

    # Receiving the user's input
    print("Let's rotate your robot")
    speed = float(input("Input the turning speed: "))

    rospy.sleep(1)  # just to make sure we read the data from the subscriber into the global variables

    twist_msg = Twist()

    twist_msg.angular.z = speed

    start_position = position
    start_yaw = yaw

    rospy.loginfo(f"Starting position: {start_position}")
    rospy.loginfo(f"Starting yaw (deg): {degrees(start_yaw)}")

    rospy.sleep(1)

    # Loop to rotate VIV
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Publish the twist
        rospy.logdebug(f"Publishing twist msg\n{twist_msg}")
        velocity_publisher.publish(twist_msg)

        rospy.loginfo(f"Current position: {position}")
        rospy.loginfo(f"Current velocity: {velocity}")

        rospy.loginfo(f"Current change in position of motor 1: {position[0] - start_position[0]}")
        rospy.loginfo(f"Current change in position of motor 2: {position[1] - start_position[1]}")
        rospy.loginfo(f"Current change in position of motor 3: {position[2] - start_position[2]}")
        rospy.loginfo(f"Current change in position of motor 4: {position[3] - start_position[3]}")

        rospy.loginfo(f"Current yaw (deg): {degrees(yaw)}")
        rospy.loginfo(f"Current change in yaw (deg): {degrees(yaw - start_yaw)}")

        rate.sleep()  # let's not spam the robot


if __name__ == '__main__':
    rotate()
