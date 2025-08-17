#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar

class QRCodeDetector(object):
    """二维码检测器"""
    def __init__(self):
        self.bridge = CvBridge()
        # 话题名称可以在config.yaml中配置并在此处传入，这里暂时硬编码
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.latest_frame = None
        self.consecutive_threshold = 1 # 连续检测次数阈值
        self.last_qr_data = None
        self.detection_count = 0

    def image_callback(self, msg):
        """图像回调，更新最新帧"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("图像转换错误: %s" % str(e))

    def process_image(self, frame):
        """图像预处理逻辑"""
        frame = cv2.flip(frame, 1) # 根据实际摄像头方向决定是否需要翻转
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

    def _decode_qrcode_from_img(self, gray_img):
        """核心二维码解码逻辑"""
        try:
            barcodes = pyzbar.decode(gray_img)
            for barcode in barcodes:
                barcode_data = barcode.data.decode("utf-8", errors='ignore')
                rospy.logdebug("pyzbar 检测到二维码: %s" % barcode_data.encode('utf-8'))
                return barcode_data.strip()
        except Exception as e:
            rospy.logerr("二维码解码错误: %s" % str(e))
        return None

    def detect_qr_code(self):
        """
        检测二维码，返回是否检测到以及检测到的数据。
        此方法不再负责播放音频。
        """
        if self.latest_frame is None:
            return False, None
            
        frame = cv2.resize(self.latest_frame, (1440, 900)) # 调整图像大小以提高检测效率或精度
        processed_frame = self.process_image(frame)
        qr_data = self._decode_qrcode_from_img(processed_frame)
        
        if qr_data:
            # 确保data是unicode，避免后面转码问题
            data = unicode(qr_data, 'utf-8', errors='ignore') if isinstance(qr_data, str) else qr_data
            
            if data == self.last_qr_data:
                self.detection_count += 1
            else:
                self.last_qr_data = data
                self.detection_count = 1

            rospy.loginfo("检测到二维码: '%s', 连续第 %d 次" % (data.encode('utf-8'), self.detection_count))

            if self.detection_count >= self.consecutive_threshold:
                rospy.loginfo("✅ 成功识别二维码: '%s'" % data.encode('utf-8'))
                return True, data
        else:
            self.last_qr_data = None
            self.detection_count = 0
            
        return False, None