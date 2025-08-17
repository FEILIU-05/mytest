#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Ucar QR Code Detector (pyzbar版)
兼容低版本OpenCV，使用pyzbar替代QRCodeDetector
"""

import sys
reload(sys)
sys.setdefaultencoding('utf-8')  # 设置默认编码为utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import subprocess
import time
from pyzbar import pyzbar


class UcarQRCodeDetector:
    def __init__(self):
        rospy.init_node("ucar_qrcode_detector", anonymous=True)

        rospy.loginfo("-------------------------------------")
        rospy.loginfo(u"   Ucar QR Code Detector (pyzbar版)   ")  # 添加u前缀表示unicode
        rospy.loginfo("-------------------------------------")
        rospy.loginfo(u"节点初始化开始...")  # 添加u前缀

        # ---------------- 参数 ----------------
        self.img_width  = int(rospy.get_param('~image_width', 1440))
        self.img_height = int(rospy.get_param('~image_height', 900))
        self.cam_topic  = rospy.get_param('~cam_topic', "/usb_cam/image_raw")
        self.rate_hz    = rospy.get_param('~rate', 10)
        self.consecutive_threshold = 0

        self.qr_audio_map = {
            "Vegetable": "vegetable.wav",
            "Fruit":     "fruit.wav",
            "Dessert":   "dessert.wav"
        }
        # 将音频目录路径转为unicode字符串
        self.audio_dir = u"/home/ucar/Desktop/ucar/src/main/navacation/voice_packge/采购任务_wav"

        # ---------------- ROS -----------------
        self.image_pub = rospy.Publisher("/ucar_camera/image_processed", Image, queue_size=5)
        self.bridge = CvBridge()
        
        # 订阅/usb_cam/image_raw话题
        self.image_sub = rospy.Subscriber(self.cam_topic, Image, self.image_callback)
        self.latest_frame = None

        rospy.loginfo(u"✅ 成功订阅图像话题: %s" % self.cam_topic)

        # ---------------- 状态 ----------------
        self.last_qr_data = None
        self.detection_count = 0
        self.is_running = True

        rospy.loginfo(u"--- 开始二维码检测 ---")
        self.start_detection()

    # 图像回调函数，接收订阅的图像数据
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(u"图像转换错误: %s" % unicode(str(e), 'utf-8', errors='ignore'))

    # ------------------------------------------------------------------
    # 图像预处理
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
    # 二维码检测：pyzbar 版
    # ------------------------------------------------------------------
    def detect_qrcode(self, gray_img):
        """
        使用pyzbar检测二维码，返回二维码字符串或None
        """
        try:
            barcodes = pyzbar.decode(gray_img)
            for barcode in barcodes:
                # 确保解码为unicode
                barcode_data = barcode.data.decode("utf-8", errors='ignore')
                rospy.logdebug(u"pyzbar 检测到二维码: %s" % barcode_data)
                return barcode_data.strip()
        except Exception as e:
            rospy.logerr(u"二维码解码错误: %s" % unicode(str(e), 'utf-8', errors='ignore'))
        
        return None

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def start_detection(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.is_running:
            if self.latest_frame is not None:
                frame = self.latest_frame
                frame = cv2.resize(frame, (self.img_width, self.img_height))
                processed_frame = self.process_image(frame)
                qr_data = self.detect_qrcode(processed_frame)
                self.handle_detection_result(qr_data)
                
                # 发布处理后的图像
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(processed_frame, "mono8")
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    rospy.logwarn(u"图像发布失败: %s" % unicode(str(e), 'utf-8', errors='ignore'))
            else:
                rospy.logdebug(u"等待图像数据...")
                
            rate.sleep()

    # ------------------------------------------------------------------
    # 结果处理
    # ------------------------------------------------------------------
    def handle_detection_result(self, data):
        if data:
            # 确保数据为unicode
            data = unicode(data, 'utf-8', errors='ignore') if isinstance(data, str) else data
            if data == self.last_qr_data:
                self.detection_count += 1
            else:
                self.last_qr_data = data
                self.detection_count = 1

            rospy.loginfo(u"检测到二维码: '%s', 连续第 %d 次" % (data, self.detection_count))

            if self.detection_count >= self.consecutive_threshold:
                rospy.loginfo(u"✅ 成功识别二维码: '%s'" % data)
                self.set_qrcode_param(data)
                self.play_audio_for_qr(data)

                rospy.loginfo(u"任务完成，节点即将关闭...")
                time.sleep(1.5)
                self.is_running = False
                rospy.signal_shutdown(u"成功识别二维码并播报语音，任务完成。")
        else:
            self.last_qr_data = None
            self.detection_count = 0

    def set_qrcode_param(self, data):
        # 确保参数为unicode
        data = unicode(data, 'utf-8', errors='ignore') if isinstance(data, str) else data
        rospy.set_param("/procurement_task", data.lower())
        rospy.loginfo(u"已设置ROS参数: /procurement_task = %s" % data.lower())

    def play_audio_for_qr(self, data):
        # 处理数据编码
        data = unicode(data, 'utf-8', errors='ignore') if isinstance(data, str) else data
        filename = self.qr_audio_map.get(data)
        if filename:
            # 确保路径为unicode
            audio_path = os.path.join(self.audio_dir, unicode(filename, 'utf-8', errors='ignore'))
            if os.path.exists(audio_path):
                try:
                    subprocess.Popen(['aplay', audio_path.encode('utf-8')])  # 转为系统编码
                    rospy.loginfo(u"🔊 正在播放语音: %s" % audio_path)
                except Exception as e:
                    rospy.logwarn(u"语音播放失败: %s" % unicode(str(e), 'utf-8', errors='ignore'))
            else:
                rospy.logwarn(u"语音文件不存在: %s" % audio_path)
        else:
            rospy.logwarn(u"未找到 '%s' 对应的语音文件" % data)

    # ------------------------------------------------------------------
    # 析构
    # ------------------------------------------------------------------
    def __del__(self):
        rospy.loginfo(u"--- 节点资源释放完毕 ---")


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.loginfo(u"--- 启动二维码检测节点 ---")
        detector = UcarQRCodeDetector()
        while detector.is_running and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo(u"主程序退出。")
    except rospy.ROSInterruptException:
        rospy.loginfo(u"节点被用户或程序终止")
    except Exception as e:
        rospy.logerr(u"致命错误: %s" % unicode(str(e), 'utf-8', errors='ignore'))