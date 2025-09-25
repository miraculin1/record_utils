#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import (
    quaternion_matrix,
    translation_matrix,
    quaternion_from_matrix,
)
from scipy.spatial.transform import Rotation as R


# 全局的 4x4 变换矩阵 (例：平移 0.1m, 旋转单位四元数)
pub = None


def _quat_to_mat(quat):
    tx, ty, tz, qx, qy, qz, qw = quat
    # 创建四元数对象
    rotation = R.from_quat([qx, qy, qz, qw])
    # 获取 3x3 旋转矩阵
    R_matrix = rotation.as_matrix()
    # 生成 4x4 变换矩阵
    T = np.eye(4)
    T[:3, :3] = R_matrix
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T


def _mat_to_quat(mat):
    R_matrix = R.from_matrix(mat[:3, :3])
    R_quat = R_matrix.as_quat()
    pos = mat[:3, 3]
    return np.concatenate([pos, R_quat])


T_rgb2e = np.array([
    [
        0.9999503814418125,
        -0.008476125530290306,
        -0.005233540900475174,
        -0.009521508361244673,
    ],
    [
        0.00839730562094776,
        0.9998536797072975,
        -0.014903168591738899,
        -0.040872989863115945,
    ],
    [
        0.005359096255021435,
        0.014858481475580016,
        0.9998752450258771,
        0.007047613347488934,
    ],
    [0.0, 0.0, 0.0, 1.0],
])

q_imu_rgb = np.array([
    -0.6432684282354907,
    -0.22729190340344108,
    -0.20132865818953167,
    0.501770763589484,
    -0.5038707736262429,
    0.5077204203374879,
    -0.48637466943133719,
])

q_imu_marker = np.array([
    0,
    0,
    0,
    0,
    0,
    0,
    1,
])

q_mocap_world = np.array([
    -0.06353380789104805,
    -0.5385446339988669,
    1.7729103627085852,
    -0.6140135377500834,
    -0.06880267317181621,
    0.0403451765704027,
    0.7852552670001917,
])


q_gripper_cam = np.array([
    0.01904596,
    0.00028386,
    -0.06739764,
    -0.72494445,
    -0.06365862,
    0.02472575,
    0.68541357,
])

T_imu_rgb = _quat_to_mat(q_imu_rgb)
T_imu_marker = _quat_to_mat(q_imu_marker)
T_mocap_world = _quat_to_mat(q_mocap_world)
T_gripper_cam = _quat_to_mat(q_gripper_cam)

T_e2mocap = np.linalg.inv(T_mocap_world) @ np.linalg.inv(T_imu_marker) @ T_imu_rgb
T_fix = T_gripper_cam
print(T_fix)


def pose_to_mat(msg):
    t = [msg.position.x, msg.position.y, msg.position.z]
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    return np.dot(translation_matrix(t), quaternion_matrix(q))


def mat_to_pose(T):
    t = T[:3, 3]
    q = quaternion_from_matrix(T)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = Point(*t)
    pose.pose.orientation = Quaternion(*q)
    return pose


def cb(msg):
    global pub
    T_in = pose_to_mat(msg.pose)
    T_out = T_in @ T_fix
    out_pose = mat_to_pose(T_out)
    out_pose.header.frame_id = "world"  # 可改
    pub.publish(out_pose)


if __name__ == "__main__":
    rospy.init_node("pose_transformer")
    pub = rospy.Publisher("/pose_transformed", PoseStamped, queue_size=10)
    #  rospy.Subscriber("/vrpn_client_node/Tracker0921/pose", PoseStamped, cb)
    rospy.Subscriber("/vrpn_client_node/tracker_wx/pose", PoseStamped, cb)
    rospy.spin()
