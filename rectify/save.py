#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import rospy
import threading
import select
import termios
import tty
from geometry_msgs.msg import PoseStamped

class PoseSaver:
    def __init__(self):
        # Params
        self.topic1  = rospy.get_param("~topic1", "/vrpn_client_node/tracker_wx/pose")   # geometry_msgs/PoseStamped
        self.topic2  = rospy.get_param("~topic2", "/aruco_single/pose")   # geometry_msgs/PoseStamped
        self.outfile = rospy.get_param("~outfile", "poses_log.txt")
        self.append  = rospy.get_param("~append", True)        # False 则先重写文件头

        # Buffers
        self.lock = threading.Lock()
        self.q1 = None  # (x,y,z,w)
        self.t1 = None  # ROS time
        self.p1 = None  # (x,y,z)

        self.q2 = None
        self.t2 = None
        self.p2 = None

        # Subscribers
        rospy.Subscriber(self.topic1, PoseStamped, self.cb1, queue_size=20)
        rospy.Subscriber(self.topic2, PoseStamped, self.cb2, queue_size=20)

        # Prepare file (header)
        need_header = True
        if os.path.exists(self.outfile) and self.append:
            # 追加模式且已存在，不强制重写表头
            need_header = False
        if not self.append:
            # 覆盖
            open(self.outfile, "w").close()
            need_header = True

        if need_header:
            with open(self.outfile, "w") as f:
                f.write("# time_wall, time1_ros, p1.x, p1.y, p1.z, q1.x, q1.y, q1.z, q1.w, "
                        "time2_ros, p2.x, p2.y, p2.z, q2.x, q2.y, q2.z, q2.w\n")

        rospy.loginfo("PoseSaver started. Press 's' to save, 'q' to quit.")
        rospy.loginfo("topic1: %s | topic2: %s | outfile: %s",
                      self.topic1, self.topic2, os.path.abspath(self.outfile))

    def cb1(self, msg: PoseStamped):
        with self.lock:
            self.q1 = (msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z,
                       msg.pose.orientation.w)
            self.p1 = (msg.pose.position.x,
                       msg.pose.position.y,
                       msg.pose.position.z)
            self.t1 = msg.header.stamp

    def cb2(self, msg: PoseStamped):
        with self.lock:
            self.q2 = (msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z,
                       msg.pose.orientation.w)
            self.p2 = (msg.pose.position.x,
                       msg.pose.position.y,
                       msg.pose.position.z)
            self.t2 = msg.header.stamp

    def save_once(self):
        with self.lock:
            q1, p1, t1 = self.q1, self.p1, self.t1
            q2, p2, t2 = self.q2, self.p2, self.t2

        if q1 is None or q2 is None or p1 is None or p2 is None:
            rospy.logwarn("No data yet (pose1 or pose2 missing).")
            return

        wall = time.time()
        line = "{:.6f}, {:.9f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.9f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}\n".format(
            wall,
            t1.to_sec() if t1 is not None else float("nan"),
            p1[0], p1[1], p1[2],
            q1[0], q1[1], q1[2], q1[3],
            t2.to_sec() if t2 is not None else float("nan"),
            p2[0], p2[1], p2[2],
            q2[0], q2[1], q2[2], q2[3],
        )
        with open(self.outfile, "a") as f:
            f.write(line)
        rospy.loginfo("Saved poses to %s", self.outfile)

def keyboard_loop(node: PoseSaver):
    """
    Non-blocking keypress loop in terminal:
      - press 's' or 'S' to save one line
      - press 'q' or 'Q' to exit
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                ch = sys.stdin.read(1)
                if ch in ('s', 'S'):
                    node.save_once()
                elif ch in ('q', 'Q'):
                    rospy.loginfo("Quit requested.")
                    rospy.signal_shutdown("User quit")
                    break
            rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rospy.init_node("save_pose_on_key", anonymous=False)
    node = PoseSaver()
    keyboard_loop(node)

if __name__ == "__main__":
    main()

