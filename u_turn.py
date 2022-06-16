from math import radians, degrees
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
import util

yaw = 0
yaw_publisher: rospy.Publisher

voltage, current = 0, 0
power_publisher: rospy.Publisher


def write_voltage(voltage_data):
    global voltage
    # rospy.logdebug("Received data from voltage topic")
    voltage = voltage_data.data


def write_current_and_print(current_data):
    global current
    # rospy.logdebug("Received data from current topic")
    current = current_data.data
    power = voltage*current
    print(f"power: {power}")

    power_data = Float32()
    power_data.data = power
    power_publisher.publish(power_data)


def odom_callback(odom_data):
    # rospy.logdebug("Received data from /viv/viv_velocity_controller/odom")
    global yaw  # radians
    yaw = util.get_yaw_from_pose_message(odom_data.pose.pose)  # first pose is PoseWithCovariance

    yaw_data = Float32()
    yaw_data.data = degrees(yaw)
    yaw_publisher.publish(yaw_data)


def rotate():
    global yaw_publisher, power_publisher
    # Starts a new node
    rospy.init_node('u_turn')

    yaw_publisher = rospy.Publisher('/T265/camera/odom/yaw', Float32, queue_size=None)

    velocity_publisher = rospy.Publisher('/viv/marko/cmd_vel', Twist, queue_size=None)
    rospy.Subscriber('/T265/camera/odom/sample', Odometry, odom_callback)

    rospy.Subscriber('/battery_voltage_sensor_reading', Float32, write_voltage)
    rospy.Subscriber('/battery_current_sensor_reading', Float32, write_current_and_print)
    power_publisher = rospy.Publisher('/battery_power_sensor_calculation', Float32, queue_size=None)

    # Receiving the user's input
    print("Let's make a U-turn")
    rotational_speed = float(input("Input the rotational speed: "))
    forward_speed = float(input("Input the forward speed: "))
    # wanted_rotation = radians(float(input("Type your wanted rotation (deg): ")))
    wanted_rotation = radians(175)

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
        rospy.loginfo(f"Current rotation diff {degrees(abs(yaw - start_rotation))} deg")

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
