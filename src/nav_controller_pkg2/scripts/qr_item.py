#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Ucar QR Code Detector + 目标发布与结果接收器（含房间播报）
"""
import time  # 在开头加上
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import subprocess
from pyzbar import pyzbar

class UcarQRCodeNode:
    def __init__(self):
        rospy.init_node("ucar_qrcode_node", anonymous=True)

        rospy.loginfo(u"=== Ucar QR Code 节点启动 (等待结果版) ===")

        # --- 参数 ---
        self.img_width = int(rospy.get_param('~image_width', 1440))
        self.img_height = int(rospy.get_param('~image_height', 900))
        self.cam_topic = rospy.get_param('~cam_topic', "/usb_cam/image_raw")
        self.rate_hz = rospy.get_param('~rate', 10)
        self.consecutive_threshold = 3
        self.response_timeout = rospy.get_param('~response_timeout', 180.0)

        self.qr_audio_map = {
            "Vegetable": "vegetable.wav",
            "Fruit":     "fruit.wav",
            "Dessert":   "dessert.wav"
        }

        self.room_audio_map = {
            "A": "A.wav",
            "B": "B.wav",
            "C": "C.wav"
        }

        self.audio_dir = u"/home/ucar/Desktop/ucar/src/main/navacation/voice_packge/房间_wav"

        # --- ROS 通信 ---
        self.bridge = CvBridge()
        self.latest_frame = None

        self.image_sub = rospy.Subscriber(self.cam_topic, Image, self.image_callback)
        self.target_pub = rospy.Publisher("/procurement_task", String, queue_size=10)
        self.result_sub = rospy.Subscriber("/recognition_result", String, self.result_callback)

        # --- 状态变量 ---
        self.state = "DETECTING"
        self.last_qr_data = None
        self.detection_count = 0
        self.wait_start_time = None
        self.detected_item = None  # 记录识别到的 item

        rospy.loginfo(u"✅ 初始化完成，开始识别二维码...")
        self.start_detection()

    def image_callback(self, msg):
        if self.state == "DETECTING":
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logerr(u"图像转换错误: %s" % unicode(str(e), 'utf-8', errors='ignore'))

    def process_image(self, frame):
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 9, 75, 75)
        clahe = cv2.createCLAHE(2.0, (8, 8))
        gray = clahe.apply(gray)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return gray

    def detect_qrcode(self, gray_img):
        try:
            barcodes = pyzbar.decode(gray_img)
            for barcode in barcodes:
                return barcode.data.decode("utf-8", errors='ignore').strip()
        except Exception as e:
            rospy.logerr(u"二维码解码失败: %s" % unicode(str(e), 'utf-8', errors='ignore'))
        return None

    def start_detection(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.state == "DETECTING":
                if self.latest_frame is not None:
                    frame = cv2.resize(self.latest_frame, (self.img_width, self.img_height))
                    processed = self.process_image(frame)
                    qr_data = self.detect_qrcode(processed)
                    self.handle_detection_result(qr_data)

            elif self.state == "WAITING_FOR_RESULT":
                rospy.loginfo_throttle(1, u"⏳ 等待远程识别结果中...")
                if (rospy.Time.now() - self.wait_start_time).to_sec() > self.response_timeout:
                    rospy.logerr(u"❌ 等待结果超时！")
                    self.state = "COMPLETED"

            elif self.state == "COMPLETED":
                rospy.loginfo(u"🏁 任务流程结束，节点关闭。")
                rospy.signal_shutdown(u"任务完成")
                break

            rate.sleep()

    def handle_detection_result(self, data):
        if data:
            data = unicode(data, 'utf-8', errors='ignore') if isinstance(data, str) else data
            if data == self.last_qr_data:
                self.detection_count += 1
            else:
                self.last_qr_data = data
                self.detection_count = 1

            rospy.loginfo(u"识别: '%s'，连续 %d / %d 次" % (data, self.detection_count, self.consecutive_threshold))

            if self.detection_count >= self.consecutive_threshold:
                rospy.loginfo(u"✅ 成功识别二维码: %s" % data)
                self.publish_target(data.lower())
                self.play_audio(data)

                self.state = "WAITING_FOR_RESULT"
                self.wait_start_time = rospy.Time.now()
                rospy.loginfo(u"已发布目标，现切换到等待结果状态...")
        else:
            self.last_qr_data = None
            self.detection_count = 0

    def publish_target(self, data):
        msg = String()
        msg.data = data
        self.target_pub.publish(msg)
        rospy.loginfo(u"📤 已发布目标到 /procurement_task: %s" % data)

    def result_callback(self, msg):
        if self.state == "WAITING_FOR_RESULT":
            data = msg.data.strip()
            rospy.loginfo(u"📥 成功收到远程识别结果: %s" % data)

            # 解析 item 和 room
            if ',' in data:
                parts = data.split(',')
                item = parts[0].strip()
                room = parts[1].strip().upper()

                self.detected_item = item
                rospy.loginfo(u"✅ 记录物品: %s，房间: %s" % (item, room))
                
                self.play_room_audio(room)
            else:
                rospy.logwarn(u"⚠️ 接收到的结果格式不正确，未包含','")

            self.state = "COMPLETED"
        else:
            rospy.logwarn(u"在非等待状态下收到消息，已忽略: %s" % msg.data)

    def play_audio(self, data):
        filename = self.qr_audio_map.get(data)
        if filename:
            path = os.path.join(self.audio_dir.replace("房间_wav", "采购任务_wav"), unicode(filename, 'utf-8', errors='ignore'))
            if os.path.exists(path):
                rospy.loginfo(u"🔊 播放语音: %s" % path)
                subprocess.call(['aplay', path.encode('utf-8')])  # 使用 call 等待播放完成
            else:
                rospy.logwarn(u"语音文件不存在: %s" % path)
        else:
            rospy.logwarn(u"没有找到对应语音: %s" % data)


    def play_room_audio(self, room):
        filename = self.room_audio_map.get(room)
        if filename:
            path = os.path.join(self.audio_dir, filename)
            if os.path.exists(path):
                subprocess.Popen(['aplay', path.encode('utf-8')])
                rospy.loginfo(u"🔊 播放房间语音: %s" % path)
            else:
                rospy.logwarn(u"房间语音文件不存在: %s" % path)
        else:
            rospy.logwarn(u"⚠️ 未定义房间语音: %s" % room)


if __name__ == '__main__':
    try:
        UcarQRCodeNode()
    except rospy.ROSInterruptException:
        pass
