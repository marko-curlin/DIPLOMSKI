from math import radians, degrees
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import util

x = 0
y = 0
yaw = 0


def callback(odom_data):
    # rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global yaw, x, y
    pose = odom_data.pose.pose  # first pose is PoseWithCovariance
    yaw = util.get_yaw_from_pose_message(pose)
    x = pose.position.x
    y = pose.position.y


def rotate():
    # Starts a new node
    rospy.init_node('rotate')

    velocity_publisher = rospy.Publisher('/viv/viv_velocity_controller/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_velocity_controller/odom', Odometry, callback)

    # Receiving the user's input
    print("Let's turn your robot")
    speed = float(input("Input the turning speed: "))
    wanted_rotation = radians(float(input("Type your wanted rotation (deg): ")))

    rospy.sleep(1)  # just to make sure we read the data from /odom into the global variables

    twist_msg = Twist()

    twist_msg.angular.z = speed

    start_yaw = yaw
    start_x = x
    start_y = y

    rospy.loginfo("Starting to rotate the robot")
    rospy.loginfo(f"Starting yaw: {degrees(start_yaw)}")

    rospy.sleep(1)

    # Loop to rotate VIV the specified wanted_rotation
    rate = rospy.Rate(10)
    while abs(yaw - start_yaw) < wanted_rotation:
        # Publish the twist
        rospy.logdebug(f"Publishing twist msg\n{twist_msg}")
        velocity_publisher.publish(twist_msg)

        # t1 = rospy.Time.now().to_sec()
        rospy.loginfo(f"Current yaw: {degrees(yaw)}")
        rospy.loginfo(f"Current change in yaw: {degrees(yaw - start_yaw)}")
        rospy.loginfo(f"Current distance from start point in x axis: {x - start_x}")
        rospy.loginfo(f"Current distance from start point in y axis: {y - start_y}")

        rate.sleep()  # let's not spam the robot

    # After the loop, stops the robot
    twist_msg.angular.z = 0
    # Force the robot to stop
    velocity_publisher.publish(twist_msg)

    rospy.sleep(1)
    rospy.loginfo(f"Final yaw {degrees(yaw)}")
    rospy.loginfo(f"Final coordinates ({x}, {y})")
    # The important bits
    rospy.loginfo(f"Final change in yaw {degrees(yaw - start_yaw)}")
    rospy.loginfo(f"Final distance from start point in x axis {x - start_x}")
    rospy.loginfo(f"Final distance from start point in y axis {y - start_y}")

    rospy.signal_shutdown("Script over")


if __name__ == '__main__':
    rotate()
