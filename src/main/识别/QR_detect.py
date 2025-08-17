#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Ucar QR Code Detector (OpenCV QRCodeDetector 版)
远距离识别优化：使用 cv2.QRCodeDetector 替换 pyzbar
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import subprocess
import time


class UcarQRCodeDetector:
    def __init__(self):
        rospy.init_node("ucar_qrcode_detector", anonymous=True)

        rospy.loginfo("-------------------------------------")
        rospy.loginfo("   Ucar QR Code Detector (OpenCV版)   ")
        rospy.loginfo("-------------------------------------")
        rospy.loginfo("节点初始化开始...")

        # ---------------- 参数 ----------------
        self.img_width  = int(rospy.get_param('~image_width', 1440))
        self.img_height = int(rospy.get_param('~image_height', 900))
        self.cam_topic  = rospy.get_param('~cam_topic', "/ucar_camera/image_raw")
        self.device_path = rospy.get_param('~device_path', "/dev/video0")
        self.rate_hz    = rospy.get_param('~rate', 10)
        self.consecutive_threshold = 3

        self.qr_audio_map = {
            "Vegetable": "vegetable.wav",
            "Fruit":     "fruit.wav",
            "Dessert":   "dessert.wav"
        }
        self.audio_dir = "/home/ucar/Desktop/ucar/src/main/voice_packge/采购任务_wav"

        # ---------------- ROS -----------------
        self.image_pub = rospy.Publisher(self.cam_topic, Image, queue_size=5)
        self.bridge = CvBridge()

        # ---------------- 摄像头 --------------
        self.cap = cv2.VideoCapture(self.device_path)
        if not self.cap.isOpened():
            rospy.logerr("❌ 摄像头打开失败: %s" % self.device_path)
            rospy.signal_shutdown("摄像头初始化失败")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.rate_hz)

        rospy.loginfo("✅ 摄像头初始化成功: %dx%d @ %dHz" %
                      (self.img_width, self.img_height, self.rate_hz))

        # ---------------- QR 检测器 -----------
        self.qr_detector = cv2.QRCodeDetector()

        # ---------------- 状态 ----------------
        self.last_qr_data = None
        self.detection_count = 0
        self.is_running = True

        rospy.loginfo("--- 开始二维码检测 ---")
        self.start_detection()

    # ------------------------------------------------------------------
    # 图像预处理（完全不变）
    # ------------------------------------------------------------------
    def process_image(self, frame):
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)

        _, gray = cv2.threshold(gray, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return gray

    # ------------------------------------------------------------------
    # 二维码检测：OpenCV 版
    # ------------------------------------------------------------------
    def detect_qrcode(self, gray_img):
        """
        返回二维码字符串或 None
        """
        data, pts, _ = self.qr_detector.detectAndDecode(gray_img)
        if data:
            rospy.logdebug("OpenCV 检测到二维码: %s" % data)
            return data.strip()
        return None

    # ------------------------------------------------------------------
    # 主循环（完全不变）
    # ------------------------------------------------------------------
    def start_detection(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.is_running:
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("⚠️ 无法获取视频帧")
                rate.sleep()
                continue

            frame = cv2.resize(frame, (self.img_width, self.img_height))
            processed_frame = self.process_image(frame)
            qr_data = self.detect_qrcode(processed_frame)
            self.handle_detection_result(qr_data)
            rate.sleep()

    # ------------------------------------------------------------------
    # 结果处理（完全不变）
    # ------------------------------------------------------------------
    def handle_detection_result(self, data):
        if data:
            if data == self.last_qr_data:
                self.detection_count += 1
            else:
                self.last_qr_data = data
                self.detection_count = 1

            rospy.loginfo("检测到二维码: '%s', 连续第 %d 次" %
                          (data, self.detection_count))

            if self.detection_count >= self.consecutive_threshold:
                rospy.loginfo("✅ 成功识别二维码: '%s'" % data)
                self.set_qrcode_param(data)
                self.play_audio_for_qr(data)

                rospy.loginfo("任务完成，节点即将关闭...")
                time.sleep(1.5)
                self.is_running = False
                rospy.signal_shutdown("成功识别二维码并播报语音，任务完成。")
        else:
            self.last_qr_data = None
            self.detection_count = 0

    def set_qrcode_param(self, data):
        rospy.set_param("/procurement_task", data.lower())
        rospy.loginfo("已设置ROS参数: /procurement_task = %s" % data.lower())

    def play_audio_for_qr(self, data):
        filename = self.qr_audio_map.get(data)
        if filename:
            audio_path = os.path.join(self.audio_dir, filename)
            if os.path.exists(audio_path):
                try:
                    subprocess.Popen(['aplay', audio_path])
                    rospy.loginfo("🔊 正在播放语音: %s" % audio_path)
                except Exception as e:
                    rospy.logwarn("语音播放失败: %s" % str(e))
            else:
                rospy.logwarn("语音文件不存在: %s" % audio_path)
        else:
            rospy.logwarn("未找到 '%s' 对应的语音文件" % data)

    # ------------------------------------------------------------------
    # 析构（完全不变）
    # ------------------------------------------------------------------
    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            rospy.loginfo("--- 摄像头资源释放完毕 ---")


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.loginfo("--- 启动二维码检测节点 ---")
        detector = UcarQRCodeDetector()
        while detector.is_running and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("主程序退出。")
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户或程序终止")
    except Exception as e:
        rospy.logerr("致命错误: %s" % str(e))