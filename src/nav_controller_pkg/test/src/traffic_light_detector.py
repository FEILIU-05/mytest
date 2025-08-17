#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TrafficLightDetector(object):
    """
    一个内置的交通灯检测器，直接处理图像流。
    """
    def __init__(self):
        rospy.loginfo("初始化内置交通灯检测器...")
        
        self.min_area = 500
        self.aspect_ratio_range = (0.5, 2.0)
        self.confidence_threshold = 800

        self.bridge = CvBridge()
        self.current_status = "unknown"
        self.status_lock = threading.Lock()

        # 注意: 确保在第6个点之后，这个话题是由 ucar_camera 发布的
        # 话题名称可以在config.yaml中配置并在此处传入，这里暂时硬编码
        image_topic = '/ucar_camera/image_raw' 
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.loginfo("交通灯检测器已订阅: %s", image_topic)

    def image_callback(self, msg):
        """ROS图像回调函数，处理每一帧图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected_color = self._detect_color_from_image(cv_image)
            with self.status_lock:
                self.current_status = detected_color
        except Exception as e:
            rospy.logerr("交通灯图像处理异常: %s" % str(e))

    def _detect_color_from_image(self, bgr_image):
        """核心检测逻辑，处理单帧BGR图像，返回颜色字符串"""
        try:
            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
            color_ranges = {
                'red': [(np.array([0, 100, 100]), np.array([10, 255, 255])),
                        (np.array([160, 100, 100]), np.array([180, 255, 255]))],
                'green': [(np.array([35, 100, 100]), np.array([85, 255, 255]))],
            }
            color_detections = {}

            for color, ranges in color_ranges.items():
                mask = cv2.inRange(hsv_image, ranges[0][0], ranges[0][1])
                if len(ranges) > 1:
                    mask2 = cv2.inRange(hsv_image, ranges[1][0], ranges[1][1])
                    mask = cv2.bitwise_or(mask, mask2)

                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                try:
                    # OpenCV 3.x+ returns only two, older versions three
                    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                except ValueError:
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                score = 0
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > self.min_area:
                        x, y, w, h = cv2.boundingRect(contour)
                        if h > 0:
                            aspect_ratio = float(w) / h
                            if self.aspect_ratio_range[0] < aspect_ratio < self.aspect_ratio_range[1]:
                                score += area
                color_detections[color] = score
            
            if color_detections:
                max_color = max(color_detections, key=color_detections.get)
                if color_detections[max_color] > self.confidence_threshold:
                    return max_color
            return "unknown"
        except Exception as e:
            rospy.logerr("交通灯颜色检测算法异常: %s" % str(e))
            return "unknown"

    def get_status(self):
        """线程安全地获取当前检测状态"""
        with self.status_lock:
            return self.current_status