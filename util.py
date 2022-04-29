import tf


def get_yaw_from_pose_message(pose_msg):
    quaternion = quaternion_to_tuple(pose_msg.orientation)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


def quaternion_to_tuple(quaternion):
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w
