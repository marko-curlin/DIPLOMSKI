import rospy
from nav_msgs.msg import Odometry


def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard\n%s", data.pose)


def listen_to_odometry():
    rospy.init_node('listener', anonymous=True)

    sub = rospy.Subscriber('/viv/viv_velocity_controller/odom', Odometry, callback)

    rospy.spin()


def main():
    listen_to_odometry()


if __name__ == '__main__':
    main()
