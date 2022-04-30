import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

start_position = 0, 0, 0, 0
position = 0, 0, 0, 0
velocity = 0, 0, 0, 0
velocity_publisher: rospy.Publisher


def callback(joint_state_data):
    rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global position, velocity
    position = joint_state_data.position
    velocity = joint_state_data.velocity


def shutdown_func():
    vel_msg = Twist()

    # Once shutdown, stops the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)

    rospy.sleep(1)
    # The important bits
    rospy.loginfo(f"Final position {position}")
    rospy.loginfo(f"Final change in position of motor 1 {position[0] - start_position[0]}")
    rospy.loginfo(f"Final change in position of motor 2 {position[1] - start_position[1]}")
    rospy.loginfo(f"Final change in position of motor 3 {position[2] - start_position[2]}")
    rospy.loginfo(f"Final change in position of motor 4 {position[3] - start_position[3]}")


def move():
    global velocity_publisher, start_position

    # Starts a new node
    rospy.init_node('move_straight')

    rospy.on_shutdown(shutdown_func)

    velocity_publisher = rospy.Publisher('/viv/marko/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_epos_driver/joint_state', JointState, callback)

    vel_msg = Twist()

    # Receiving the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed: "))

    rospy.sleep(1)  # just to make sure we read the data from the subscriber into the global variables

    vel_msg.linear.x = speed

    start_position = position

    rospy.loginfo(f"Starting position: {start_position}")

    rospy.sleep(1)

    # Loop to move VIV the specified distance
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Publish the velocity
        rospy.logdebug(f"Publishing velocity\n{vel_msg}")
        velocity_publisher.publish(vel_msg)

        rospy.loginfo(f"Current position {position}")
        rospy.loginfo(f"Current velocity {velocity}")

        rospy.loginfo(f"Current change in position of motor 1 {position[0] - start_position[0]}")
        rospy.loginfo(f"Current change in position of motor 2 {position[1] - start_position[1]}")
        rospy.loginfo(f"Current change in position of motor 3 {position[2] - start_position[2]}")
        rospy.loginfo(f"Current change in position of motor 4 {position[3] - start_position[3]}")

        rate.sleep()  # let's not spam the robot


if __name__ == '__main__':
    move()
