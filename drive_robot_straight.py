import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

moved_x = 0


def callback(odom_data):
    rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global moved_x
    position = odom_data.pose.pose.position
    moved_x = position.x  # calculate absolute offset using x and y


def move():
    global moved_x

    # Starts a new node
    rospy.init_node('move_straight')

    velocity_publisher = rospy.Publisher('/viv/viv_velocity_controller/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_velocity_controller/odom', Odometry, callback)

    vel_msg = Twist()

    # Receiving the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed: "))
    distance = float(input("Type your distance: "))

    vel_msg.linear.x = speed

    start_distance = moved_x

    rospy.loginfo("Starting to move the robot")
    rospy.loginfo(f"Starting offset in x axis: {start_distance}")

    # Loop to move VIV the specified distance
    while abs(moved_x-start_distance) < distance:
        # Publish the velocity
        rospy.logdebug(f"Publishing velocity\n{vel_msg}")
        velocity_publisher.publish(vel_msg)

        # t1 = rospy.Time.now().to_sec()
        rospy.loginfo(f"Currently offset in x axis {moved_x}")
        rospy.loginfo(f"Currently moved {abs(moved_x-start_distance)}")

    # After the loop, stops the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)

    rospy.sleep(1)
    rospy.loginfo(f"Final offset in x axis {moved_x}")

    rospy.signal_shutdown("Script over")


if __name__ == '__main__':
    move()
