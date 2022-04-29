import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import util

moved = 0
x = 0
y = 0
yaw = 0


def callback(odom_data):
    rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global moved, x, y, yaw
    pose = odom_data.pose.pose  # first pose is PoseWithCovariance
    position = pose.position
    moved = math.sqrt(position.x**2 + position.y**2)
    x = position.x
    y = position.y
    yaw = util.get_yaw_from_pose_message(pose)


def move():
    # Starts a new node
    rospy.init_node('move_straight')

    velocity_publisher = rospy.Publisher('/viv/viv_velocity_controller/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_velocity_controller/odom', Odometry, callback)

    vel_msg = Twist()

    # Receiving the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed: "))
    distance = float(input("Type your distance: "))

    rospy.sleep(1)  # just to make sure we read the data from /odom into the global variables

    vel_msg.linear.x = speed

    start_distance = moved
    start_x = x
    start_y = y
    start_yaw = yaw

    rospy.loginfo(f"Starting to move the robot in a straight line at an angle of {yaw}")
    rospy.loginfo(f"Starting offset from (0,0): {start_distance}")
    rospy.loginfo(f"Starting offset in x axis: {start_x}")
    rospy.loginfo(f"Starting offset in y axis: {start_y}")
    rospy.loginfo(f"Starting yaw: {start_yaw}")

    rospy.sleep(1)

    # Loop to move VIV the specified distance
    rate = rospy.Rate(10)
    while abs(moved - start_distance) < distance:
        # Publish the velocity
        rospy.logdebug(f"Publishing velocity\n{vel_msg}")
        velocity_publisher.publish(vel_msg)

        rospy.loginfo(f"Current distance from (0,0) {moved}")
        rospy.loginfo(f"Current distance from start point {moved - start_distance}")
        rospy.loginfo(f"Current distance from start point in x axis {x - start_x}")
        rospy.loginfo(f"Current distance from start point in y axis {y - start_y}")
        rospy.loginfo(f"Current change in yaw {start_yaw - yaw}")

        rate.sleep()  # let's not spam the robot

    # After the loop, stops the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)

    rospy.sleep(1)
    rospy.loginfo(f"Final offset {moved}")
    rospy.loginfo(f"Final offset in x axis {x}")
    rospy.loginfo(f"Final offset in y axis {y}")
    rospy.loginfo(f"Final yaw {yaw}")
    # The important bits
    rospy.loginfo(f"Final distance from start point {moved - start_distance}")
    rospy.loginfo(f"Final distance from start point in x axis {x - start_x}")
    rospy.loginfo(f"Final distance from start point in y axis {y - start_y}")
    rospy.loginfo(f"Final change in yaw {start_yaw - yaw}")

    rospy.signal_shutdown("Script over")


if __name__ == '__main__':
    move()
