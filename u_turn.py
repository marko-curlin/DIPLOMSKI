from math import radians, degrees
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import util

yaw = 0


def callback(odom_data):
    # rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global yaw  # what is the value rad or deg
    yaw = util.get_yaw_from_pose_message(odom_data.pose.pose) # first pose is PoseWithCovariance


def rotate():
    # Starts a new node
    rospy.init_node('u_turn')

    velocity_publisher = rospy.Publisher('/viv/marko/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/T265/camera/odom/sample', Odometry, callback)

    # Receiving the user's input
    print("Let's make a U-turn")
    forward_speed = float(input("Input the forward speed: "))
    rotational_speed = float(input("Input the rotational speed: "))
    # wanted_rotation = radians(float(input("Type your wanted rotation (deg): ")))
    wanted_rotation = radians(180)

    twist_msg = Twist()

    twist_msg.linear.x = forward_speed
    twist_msg.angular.z = rotational_speed

    start_rotation = yaw

    rospy.loginfo("Starting to U-turn the robot")
    rospy.loginfo(f"Starting rotation in z axis: {start_rotation}")

    # Loop to rotate VIV the specified wanted_rotation
    rate = rospy.Rate(10)
    while abs(yaw - start_rotation) < wanted_rotation:
        # Publish the twist
        rospy.logdebug(f"Publishing twist msg\n{twist_msg}")
        velocity_publisher.publish(twist_msg)

        # t1 = rospy.Time.now().to_sec()
        rospy.loginfo(f"Currently rotated in z axis {degrees(yaw)} deg")
        rospy.loginfo(f"Current rotation {degrees(abs(yaw - start_rotation))} deg")

        rate.sleep()  # let's not spam the robot

    # After the loop, stops the robot
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    # Force the robot to stop
    velocity_publisher.publish(twist_msg)

    rospy.sleep(1)
    rospy.loginfo(f"Final rotation in z axis {yaw}")

    rospy.signal_shutdown("Script over")


if __name__ == '__main__':
    rotate()
