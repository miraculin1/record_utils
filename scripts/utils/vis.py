#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np

# 适配无显示环境
import matplotlib
try:
    import os
    if os.environ.get("DISPLAY", "") == "":
        matplotlib.use("Agg")
except Exception:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

from tf.transformations import quaternion_matrix


def pose_to_T(position, orientation_xyzw):
    """position: (x,y,z), orientation_xyzw: (x,y,z,w) -> 4x4"""
    T = np.eye(4)
    T[:3, 3] = np.asarray(position, dtype=float)
    R = quaternion_matrix(np.asarray(orientation_xyzw, dtype=float))  # 4x4
    T[:3, :3] = R[:3, :3]
    return T


class TrajCollector:
    def __init__(self, topic, duration, maxlen=None, frame_id_filter=""):
        self.topic = topic
        self.duration = duration
        self.frame_id_filter = frame_id_filter

        self.lock = threading.Lock()
        self.Ts = deque(maxlen=maxlen)   # 存 4x4 齐次矩阵
        self.stamps = deque(maxlen=maxlen)

        self.sub1 = rospy.Subscriber(self.topic, PoseStamped, self.cb_pose, queue_size=100)
        #  self.sub2 = rospy.Subscriber(self.topic, Odometry, self.cb_odom, queue_size=100)

    def _accept(self, header_frame_id):
        if not self.frame_id_filter:
            return True
        return (header_frame_id == self.frame_id_filter)

    def cb_pose(self, msg: PoseStamped):
        if not self._accept(msg.header.frame_id):
            return
        p = msg.pose.position
        q = msg.pose.orientation
        T = pose_to_T(
            (p.x, p.y, p.z),
            (q.x, q.y, q.z, q.w)
        )
        with self.lock:
            self.Ts.append(T)
            self.stamps.append(msg.header.stamp.to_sec())

    def cb_odom(self, msg: Odometry):
        if not self._accept(msg.header.frame_id):
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T = pose_to_T(
            (p.x, p.y, p.z),
            (q.x, q.y, q.z, q.w)
        )
        with self.lock:
            self.Ts.append(T)
            self.stamps.append(msg.header.stamp.to_sec())

    def spin_and_collect(self):
        r = rospy.Rate(200)
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start >= self.duration:
                break
            r.sleep()

    def get_data(self):
        with self.lock:
            return list(self.Ts), list(self.stamps)


def plot_traj_with_axes(Ts, title="Camera Trajectory with Axes",
                        save_path=None, show=True,
                        axis_len=0.05, every=10, equal_aspect=True,
                        draw_tail=True):
        """
        Ts: [N x 4x4]
        axis_len: 坐标轴箭头长度（米）
        every: 每隔多少帧画一个坐标轴（防止太密）
        draw_tail: 是否画出整个轨迹
        """
        if len(Ts) == 0:
            rospy.logwarn("没有采集到任何位姿，无法作图。")
            return

        # 轨迹点
        pts = np.array([T[:3, 3] for T in Ts])  # (N,3)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(title)

        if draw_tail:
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], linewidth=1.5, alpha=0.9)

        # 在关键帧处画三元组坐标轴（x红，y绿，z蓝）
        for i in range(0, len(Ts), max(1, every)):
            T = Ts[i]
            o = T[:3, 3]
            R = T[:3, :3]
            x_axis = R[:, 0] * axis_len
            y_axis = R[:, 1] * axis_len
            z_axis = R[:, 2] * axis_len

            ax.quiver(o[0], o[1], o[2], x_axis[0], x_axis[1], x_axis[2], length=1.0, normalize=False, color='r')
            ax.quiver(o[0], o[1], o[2], y_axis[0], y_axis[1], y_axis[2], length=1.0, normalize=False, color='g')
            ax.quiver(o[0], o[1], o[2], z_axis[0], z_axis[1], z_axis[2], length=1.0, normalize=False, color='b')

        # 也给最后一帧画粗一点的坐标轴，便于观察当前姿态
        T_last = Ts[-1]
        o = T_last[:3, 3]
        R = T_last[:3, :3]
        ax.quiver(o[0], o[1], o[2], *(R[:, 0] * axis_len * 1.6), length=1.0, normalize=False, color='r', linewidth=2.0)
        ax.quiver(o[0], o[1], o[2], *(R[:, 1] * axis_len * 1.6), length=1.0, normalize=False, color='g', linewidth=2.0)
        ax.quiver(o[0], o[1], o[2], *(R[:, 2] * axis_len * 1.6), length=1.0, normalize=False, color='b', linewidth=2.0)

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")

        if equal_aspect:
            # 设置等比例坐标轴，避免视觉畸变
            x_range = pts[:, 0].max() - pts[:, 0].min()
            y_range = pts[:, 1].max() - pts[:, 1].min()
            z_range = pts[:, 2].max() - pts[:, 2].min()
            max_range = max(x_range, y_range, z_range, 1e-6)
            x_mid = (pts[:, 0].max() + pts[:, 0].min()) / 2.0
            y_mid = (pts[:, 1].max() + pts[:, 1].min()) / 2.0
            z_mid = (pts[:, 2].max() + pts[:, 2].min()) / 2.0
            half = max_range / 2.0
            ax.set_xlim(x_mid - half, x_mid + half)
            ax.set_ylim(y_mid - half, y_mid + half)
            ax.set_zlim(z_mid - half, z_mid + half)

        ax.grid(True, linestyle="--", alpha=0.4)

        if save_path:
            plt.tight_layout()
            plt.savefig(save_path, dpi=180)
            rospy.loginfo("图像保存到: %s", save_path)

        if show:
            try:
                plt.show()
            except Exception:
                pass

        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot 3D camera trajectory and draw axis triads along the path.")
    parser.add_argument("--topic", default="/vrpn_client_node/Tracker091504/pose",
                        help="VRPN 轨迹话题（PoseStamped 或 Odometry）")
    parser.add_argument("--duration", type=float, default=50.0,
                        help="采集时长（秒）")
    parser.add_argument("--every", type=int, default=10,
                        help="每隔多少帧画一个坐标轴（防止过密）")
    parser.add_argument("--axis-len", type=float, default=0.10,
                        help="坐标轴箭头的长度（米）")
    parser.add_argument("--frame-id", default="",
                        help="只接收指定 frame_id（留空不过滤）")
    parser.add_argument("--save", default="",
                        help="保存 PNG 路径（留空则不保存）")
    parser.add_argument("--no-show", action="store_true",
                        help="只保存不弹窗")
    parser.add_argument("--title", default="Camera Trajectory with Axes",
                        help="图标题")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("plot_cam_traj_axes", anonymous=True)
    rospy.loginfo("订阅: %s, 采集: %.2fs", args.topic, args.duration)

    collector = TrajCollector(topic=args.topic, duration=args.duration, frame_id_filter=args.frame_id)
    collector.spin_and_collect()
    Ts, stamps = collector.get_data()
    print(f"total poses{len(Ts)}")
    plot_traj_with_axes(
        Ts,
        title=args.title,
        save_path=(args.save if args.save else None),
        show=(not args.no_show),
        axis_len=args.axis_len,
        every=max(1, args.every),
        equal_aspect=True,
        draw_tail=True
    )


if __name__ == "__main__":
    main()

