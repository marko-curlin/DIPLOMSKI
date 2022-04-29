import tf
import math


def get_yaw_from_pose_message(pose_msg):
    quaternion = quaternion_to_tuple(pose_msg.orientation)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


def quaternion_to_tuple(quaternion):
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


def get_direction_from_points(point1, point2):
    return point2[0] - point1[0], point2[1] - point1[1]


def get_degrees_from_direction(direction):
    angle = math.atan2(direction[1], direction[0])
    # if abs(angle) > math.pi / 2:
    #     angle = angle - math.pi
    angle_degrees = math.degrees(angle)

    return angle_degrees


def test_direction():
    direction = get_direction_from_points((0, 0), (1, 0))
    print(f"direction: {direction}")
    angle_degrees = get_degrees_from_direction(direction)
    print(f"angle_degrees: {angle_degrees}")


if __name__ == '__main__':
    test_direction()
