#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorToGrayNode:
    def __init__(self):
        # 可通过参数或命名重映射设置话题
        self.in_topic  = rospy.get_param("~in_topic",  "/camera/color/image_raw")
        self.out_topic = rospy.get_param("~out_topic", "/camera/color/image_gray")
        self.queue_size = rospy.get_param("~queue_size", 10)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.out_topic, Image, queue_size=self.queue_size)
        self.sub = rospy.Subscriber(self.in_topic, Image, self.callback, queue_size=self.queue_size)

        rospy.loginfo("color_to_gray_node started.")
        rospy.loginfo(" Subscribing : %s", self.in_topic)
        rospy.loginfo(" Publishing  : %s", self.out_topic)

    @staticmethod
    def _to_gray(img, enc: str):
        e = (enc or "").lower()
        if img.ndim == 2:
            return img  # already single-channel
        if e in ("bgr8", "bgr16"):
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if e in ("rgb8", "rgb16"):
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        if e in ("bgra8", "bgra16"):
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        if e in ("rgba8", "rgba16"):
            return cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
        # 回退：按 BGR 处理
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def callback(self, msg: Image):
        try:
            # 尽量保持原编码取出（避免不必要的色彩转换）
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            gray = self._to_gray(cv_img, msg.encoding)

            # 若非 uint8，简单拉伸到 8 位（避免发布非标准编码）
            if gray.dtype != "uint8":
                gmin, gmax = gray.min(), gray.max()
                scale = 255.0 / float(max(1, (gmax - gmin)))
                gray8 = ((gray - gmin) * scale).astype("uint8")
            else:
                gray8 = gray

            out_msg = self.bridge.cv2_to_imgmsg(gray8, encoding="mono8")
            # 继承原 header：时间戳与 frame_id 完整保留
            out_msg.header = msg.header

            self.pub.publish(out_msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "color_to_gray_node: 转换失败: %s", str(e))

def main():
    rospy.init_node("color_to_gray_node")
    ColorToGrayNode()
    rospy.spin()

if __name__ == "__main__":
    main()
