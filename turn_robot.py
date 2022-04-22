from math import radians
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import tf

turned_z = 0


def callback(odom_data):
    # rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global turned_z
    quaternion = (
        odom_data.pose.pose.orientation.x,
        odom_data.pose.pose.orientation.y,
        odom_data.pose.pose.orientation.z,
        odom_data.pose.pose.orientation.w
    )
    orientation = tf.transformations.euler_from_quaternion(quaternion)
    # rospy.loginfo(f"Setting turned_z to {orientation[2]}")
    turned_z = orientation[2]


def rotate():
    global turned_z

    # Starts a new node
    rospy.init_node('move_straight')

    velocity_publisher = rospy.Publisher('/viv/viv_velocity_controller/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/viv/viv_velocity_controller/odom', Odometry, callback)

    # Receiving the user's input
    print("Let's turn your robot")
    speed = float(input("Input the turning speed: "))
    wanted_rotation = radians(float(input("Type your wanted rotation (deg): ")))

    twist_msg = Twist()

    twist_msg.angular.z = speed

    start_rotation = turned_z

    rospy.loginfo("Starting to rotate the robot")
    rospy.loginfo(f"Starting rotation in z axis: {start_rotation}")

    # Loop to rotate VIV the specified wanted_rotation
    while abs(turned_z - start_rotation) < wanted_rotation:
        # Publish the twist
        rospy.logdebug(f"Publishing twist msg\n{twist_msg}")
        velocity_publisher.publish(twist_msg)

        # t1 = rospy.Time.now().to_sec()
        rospy.loginfo(f"Currently rotated in z axis {turned_z}")
        rospy.loginfo(f"Current rotation {abs(turned_z - start_rotation)}")

    # After the loop, stops the robot
    twist_msg.angular.z = 0
    # Force the robot to stop
    velocity_publisher.publish(twist_msg)

    rospy.sleep(1)
    rospy.loginfo(f"Final rotation in z axis {turned_z}")

    rospy.signal_shutdown("Script over")


if __name__ == '__main__':
    rotate()
