#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import sys

def read_poses_from_txt(file_path):
    """读取文件中的位姿序列 (x y z qx qy qz qw)"""
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if len(line.split()) == 7:
                try:
                    x, y, z, qx, qy, qz, qw = map(float, line.split())
                    poses.append((x, y, z, qx, qy, qz, qw))
                except ValueError:
                    rospy.logwarn(f"Invalid line skipped: {line}")
            else:
                try:
                    _, x, y, z, qx, qy, qz, qw = map(float, line.split())
                    poses.append((x, y, z, qx, qy, qz, qw))
                except ValueError:
                    rospy.logwarn(f"Invalid line skipped: {line}")
    return poses

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pose_publisher.py pose.txt [topic_name] [rate]")
        sys.exit(1)

    file_path = sys.argv[1]
    topic_name = sys.argv[2] if len(sys.argv) > 2 else "/pose"
    rate_hz = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    poses = read_poses_from_txt(file_path)
    if not poses:
        print("❌ No valid poses found in file.")
        sys.exit(1)

    rospy.init_node("pose_publisher", anonymous=True)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(f"✅ Loaded {len(poses)} poses from {file_path}")
    rospy.loginfo(f"📡 Publishing to topic: {topic_name} at {rate_hz} Hz")

    seq = 0

    for x, y, z, qx, qy, qz, qw in poses:
        if rospy.is_shutdown():
            break
        msg = PoseStamped()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        pub.publish(msg)
        seq += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

