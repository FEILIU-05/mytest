#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class UcarCamera:
    def __init__(self):
        rospy.init_node("ucar_camera_listener", anonymous=True)
        rospy.loginfo("初始化 UcarCamera 节点 (订阅模式)...")

        # 交通灯检测参数
        self.min_area = 500  # 最小轮廓面积阈值
        self.aspect_ratio_range = (0.5, 2.0)  # 轮廓宽高比范围
        self.confidence_threshold = 800  # 颜色检测置信度阈值

        # 状态变量
        self.last_status = "unknown"
        self.status_count = 0
        self.required_consecutive_frames =3   # 需要连续检测到相同状态的帧数
        self.is_initialized = False  # 初始化标志

        self.status_published = False  # 是否已发布该状态

        # 图像转换工具
        self.bridge = CvBridge()

        # 发布交通灯状态
        self.traffic_light_pub = rospy.Publisher('/traffic_light_status', String, queue_size=1)

        # 订阅摄像头图像话题
        image_topic = rospy.get_param('~image_topic', '/ucar_camera/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        rospy.spin()

    def image_callback(self, msg):
        try:
            # ROS图像转OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 首次接收到图像后，标记已初始化
            if not self.is_initialized:
                self.is_initialized = True
                self.publish_status("recognition_active")
                rospy.loginfo("交通灯识别系统已激活")

            # 交通灯检测
            status = self.detect_traffic_light(rgb_frame)

            if status == self.last_status:
                self.status_count += 1
            else:
                self.last_status = status
                self.status_count = 1
                self.status_published = False  # 状态改变，允许再次发布

            # 如果状态稳定10帧且尚未发布
            if self.status_count >= self.required_consecutive_frames and not self.status_published:
                self.publish_status(status)
                rospy.loginfo("检测到交通灯状态: %s" % status)
                self.status_published = True

        except Exception as e:
            rospy.logerr("图像处理异常: %s" % str(e))

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.traffic_light_pub.publish(msg)

    def detect_traffic_light(self, image):
        try:
            hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            color_ranges = {
                'red': [(np.array([0, 100, 100]), np.array([10, 255, 255])),
                        (np.array([160, 100, 100]), np.array([180, 255, 255]))],
                'green': [(np.array([35, 100, 100]), np.array([85, 255, 255]))],
            }

            color_detections = {}

            for color, ranges in color_ranges.items():
                mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
                for lower, upper in ranges:
                    mask += cv2.inRange(hsv_image, lower, upper)

                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                # OpenCV3 findContours三返回值形式
                _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                score = 0
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > self.min_area:
                        x, y, w, h = cv2.boundingRect(contour)
                        aspect_ratio = float(w) / h
                        if self.aspect_ratio_range[0] < aspect_ratio < self.aspect_ratio_range[1]:
                            score += area

                color_detections[color] = score

            max_color = max(color_detections, key=color_detections.get)
            if color_detections[max_color] > self.confidence_threshold:
                return max_color
            else:
                return "unknown"

        except Exception as e:
            rospy.logerr("交通灯检测异常: %s" % str(e))
            return "unknown"

if __name__ == '__main__':
    try:
        UcarCamera()
    except rospy.ROSInterruptException:
        rospy.loginfo("UcarCamera 节点关闭")
