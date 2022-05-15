import tf
import math


def get_yaw_from_pose_message(pose_msg):
    quaternion = quaternion_to_iterable(pose_msg.orientation)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


def yaw_to_quaternion(degrees_yaw):
    radian_yaw = math.radians(degrees_yaw)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, radian_yaw)
    return quaternion


def quaternion_to_iterable(quaternion):
    if hasattr(quaternion, '__iter__'):
        return quaternion
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


def get_direction_from_points(point1, point2):
    return point2[0] - point1[0], point2[1] - point1[1]


def get_degrees_from_direction(direction):
    angle = math.atan2(direction[1], direction[0])
    # if abs(angle) > math.pi / 2:
    #     angle = angle - math.pi
    angle_degrees = math.degrees(angle)

    return angle_degrees


def subtract_lists_element_wise(a, b):
    return [a_i - b_i for a_i, b_i in zip(a, b)]


def test_direction():
    direction = get_direction_from_points((0, 0), (1, 0))
    print(f"direction: {direction}")
    angle_degrees = get_degrees_from_direction(direction)
    print(f"angle_degrees: {angle_degrees}")


def test_quaternion():
    yaw = 0.1
    quaternion = yaw_to_quaternion(yaw)
    pose_msg = test_quaternion  # object() doesn't work here so use any existing object
    pose_msg.orientation = quaternion
    new_yaw = get_yaw_from_pose_message(pose_msg)
    print(f"old yaw: {yaw}")
    print(f"new yaw: {math.degrees(new_yaw)}")


if __name__ == '__main__':
    test_direction()
    test_quaternion()
