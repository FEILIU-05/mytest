#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
navigation_controller.py  – V1.7.5 (Configurable Traffic Light Logic)
----------------------------------
* 重构配置管理，将音频路径、功能点映射等移至 config.yaml。
* 重构导航逻辑，使用函数分派替代 if/elif 链，实现功能与流程解耦。
* 集成内置交通灯图像识别逻辑，不再依赖外部话题。
* 新增“宏动作”支持，允许在一个导航点按顺序执行多个子任务。
* 新增“等待远程任务指派”功能，通过 rosbridge 接收外部 WebSocket 消息。
* 重构摄像头进程管理，实现安全的启动与重启逻辑。
* 将物体识别成功后的跳转点逻辑，从硬编码改为从配置文件动态加载。
* 将远程任务指派升级为“请求-响应”模式，由机器人主动发起通信。
* [修改] 修正了Python 2环境下，rospy.log* 打印中文/Emoji 导致的 UnicodeDecodeError。
* [最终修正] 采用更健壮的编码方式，将格式化后的unicode字符串显式编码为utf-8字节串，以防止ROS底层库的隐式ascii编码错误。
* [V1.7.5 修改] 将交通灯逻辑完全配置化，根据 config.yaml 中的规则进行跳转，并确保只通过一个绿灯路口。
"""
from __future__ import division
import os
import csv
import math
import yaml
import rospy
import tf
import actionlib
import subprocess
import signal
import time
import threading
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar import pyzbar
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String

# ==================== (重构后) 进程与节点控制 ====================

def stop_process(process, process_name="进程"):
    """
    一个健壮和通用的进程关闭函数 (Python 2 兼容版)。
    它会先尝试优雅地关闭进程，如果超时则强制终止。
    """
    # process_name 传入时可能是 unicode，先编码
    process_name_utf8 = process_name.encode('utf-8') if isinstance(process_name, unicode) else process_name

    if process is None or process.poll() is not None:
        rospy.loginfo((u"ℹ️ %s 无需关闭（不存在或已结束）。" % process_name).encode('utf-8'))
        return

    rospy.loginfo((u"🔪 准备关闭 %s..." % process_name).encode('utf-8'))
    try:
        process.send_signal(signal.SIGINT)

        timeout_sec = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if process.poll() is not None:
                rospy.loginfo((u"✅ %s 已成功关闭。" % process_name).encode('utf-8'))
                return
            time.sleep(0.1)

        rospy.logwarn((u"⚠️ %s 关闭超时，将强制终止！" % process_name).encode('utf-8'))
        process.kill()
        process.wait()
        rospy.loginfo((u"✅ %s 已被强制终止。" % process_name).encode('utf-8'))

    except Exception as e:
        rospy.logerr((u"❌ 关闭 %s 时发生未知错误: %s" % (process_name, str(e))).encode('utf-8'))

def ensure_camera_is_running(current_process):
    """
    确保默认摄像头(usb_cam)正在运行。
    如果已有进程在运行，会先将其关闭，然后再启动一个新的。
    """
    rospy.loginfo(u"🔄 准备启动/重启默认摄像头...".encode('utf-8'))
    stop_process(current_process, u"旧摄像头进程")

    rospy.loginfo(u"🚀 正在启动新的 usb_cam 节点...".encode('utf-8'))
    if not os.path.exists('/dev/video0'):
        rospy.logerr(u"❌ 摄像头设备 /dev/video0 不存在，无法启动。".encode('utf-8'))
        return None

    try:
        command = ['roslaunch', 'usb_cam', 'usb_cam-test.launch']
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(3)
        if process.poll() is None:
            rospy.loginfo(u"✅ 默认摄像头进程已成功启动。".encode('utf-8'))
            return process
        else:
            _, stderr = process.communicate()
            rospy.logerr((u"❌ 默认摄像头启动失败，错误: %s" % stderr.decode('utf-8', 'ignore')).encode('utf-8'))
            return None

    except Exception as e:
        rospy.logerr((u"❌ 启动 usb_cam-test.launch 时发生异常: %s" % str(e)).encode('utf-8'))
        return None

def launch_ucar_camera(current_process):
    """
    启动 ucar_camera 的 launch 文件，用于特定任务阶段。
    此函数会先关闭传入的旧进程，然后启动新的 ucar_camera 进程。
    """
    rospy.loginfo(u"📷 准备启动专用的 ucar_camera...".encode('utf-8'))
    stop_process(current_process, u"旧摄像头进程")

    rospy.loginfo(u"🚀 正在启动 roslaunch ucar_camera usb_cam-test.launch...".encode('utf-8'))
    command = ['roslaunch', 'ucar_camera', 'usb_cam-test.launch']
    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(3)
        if process.poll() is None:
            rospy.loginfo(u"✅ ucar_camera 已成功启动。".encode('utf-8'))
            return process
        else:
            _, stderr = process.communicate()
            rospy.logerr((u"❌ ucar_camera 启动失败，错误: %s" % stderr.decode('utf-8', 'ignore')).encode('utf-8'))
            return None

    except Exception as e:
        rospy.logerr((u"❌ 启动 ucar_camera launch 文件时发生异常: %s" % str(e)).encode('utf-8'))
        return None


def launch_navigation():
    rospy.loginfo(u"🚀 即将启动导航系统...".encode('utf-8'))
    command = ['roslaunch', 'nav_controller_pkg', 'nav.launch']
    try:
        subprocess.Popen(command)
        rospy.loginfo(u"已发送 nav.launch 启动命令。等待 10 秒...".encode('utf-8'))
        time.sleep(10)
        rospy.loginfo(u"✅ 导航系统应已准备就绪。".encode('utf-8'))
    except Exception as e:
        rospy.logerr((u"❌ 启动 nav.launch 失败: %s" % str(e)).encode('utf-8'))

def check_camera_status():
    rospy.loginfo(u"🔍 检查摄像头状态...".encode('utf-8'))
    camera_devices = ['/dev/video0', '/dev/video1', '/dev/video2']
    available_devices = []

    for device in camera_devices:
        if os.path.exists(device):
            try:
                import stat
                device_stat = os.stat(device)
                permissions = u"可读" if (device_stat.st_mode & stat.S_IRUSR) else u"不可读"
                rospy.loginfo((u"📷 发现摄像头设备: %s (%s)" % (device, permissions)).encode('utf-8'))
                available_devices.append(device)
            except Exception as e:
                rospy.logwarn((u"⚠️ 无法检查设备 %s: %s" % (device, str(e))).encode('utf-8'))
        else:
            rospy.loginfo((u"❌ 摄像头设备 %s 不存在" % device).encode('utf-8'))

    if not available_devices:
        rospy.logerr(u"❌ 未找到可用的摄像头设备".encode('utf-8'))
        return False

    return len(available_devices) > 0

# =================================================================
def q_to_yaw(q):
    return tf.transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w])[2]


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class PID(object):
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev = 0.0
        self.intg = 0.0

    def step(self, err, dt):
        self.intg += err * dt
        der = (err - self.prev) / dt if dt else 0.0
        self.prev = err
        return self.kp * err + self.ki * self.intg + self.kd * der

    def reset(self):
        self.prev = 0.0
        self.intg = 0.0


class PIDController(object):
    """完整的PID控制器实现"""
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, output_min=None, output_max=None, integral_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max if integral_max is not None else 10.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, dt=None):
        if dt is None or dt <= 0:
            dt = 0.1
        
        P_out = self.Kp * error
        
        self.integral += error * dt
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        I_out = self.Ki * self.integral
        
        D_out = 0.0
        if dt > 1e-6:
            D_out = self.Kd * ((error - self.prev_error) / dt)
        
        self.prev_error = error
        
        output = P_out + I_out + D_out
        
        if self.output_max is not None and output > self.output_max:
            output = self.output_max
            if self.integral != 0:
                self.integral = self.integral * 0.95
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
            if self.integral != 0:
                self.integral = self.integral * 0.95
                
        return output


class TrafficLightDetector:
    """
    一个内置的交通灯检测器，直接处理图像流。
    """
    def __init__(self):
        rospy.loginfo(u"初始化内置交通灯检测器...".encode('utf-8'))
        
        self.min_area = 500
        self.aspect_ratio_range = (0.5, 2.0)
        self.confidence_threshold = 800

        self.bridge = CvBridge()
        self.current_status = "unknown"
        self.status_lock = threading.Lock()

        image_topic = '/ucar_camera/image_raw'
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.loginfo((u"交通灯检测器已订阅: %s" % image_topic).encode('utf-8'))

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected_color = self._detect_color_from_image(cv_image)
            with self.status_lock:
                self.current_status = detected_color
        except Exception as e:
            rospy.logerr((u"交通灯图像处理异常: %s" % str(e)).encode('utf-8'))

    def _detect_color_from_image(self, bgr_image):
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
            rospy.logerr((u"交通灯颜色检测算法异常: %s" % str(e)).encode('utf-8'))
            return "unknown"

    def get_status(self):
        with self.status_lock:
            return self.current_status

class QRCodeDetector:
    """二维码检测器"""
    def __init__(self, audio_config):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.latest_frame = None
        self.qr_audio_map = audio_config.get('qr_map', {})
        self.audio_dir = audio_config.get('qr_dir', '')
        self.consecutive_threshold = 1
        self.last_qr_data = None
        self.detection_count = 0
        self.qr_detected = False
        self.qr_data = None

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr((u"图像转换错误: %s" % str(e)).encode('utf-8'))

    def process_image(self, frame):
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return gray

    def detect_qrcode(self, gray_img):
        try:
            barcodes = pyzbar.decode(gray_img)
            for barcode in barcodes:
                barcode_data = barcode.data.decode("utf-8", errors='ignore')
                return barcode_data.strip()
        except Exception as e:
            rospy.logerr((u"二维码解码错误: %s" % str(e)).encode('utf-8'))
        return None

    def detect_qr_code(self):
        if self.latest_frame is None:
            return False, None
        frame = cv2.resize(self.latest_frame, (1440, 900))
        processed_frame = self.process_image(frame)
        qr_data = self.detect_qrcode(processed_frame)
        if qr_data:
            data = unicode(qr_data, 'utf-8', errors='ignore') if isinstance(qr_data, str) else qr_data
            if data == self.last_qr_data:
                self.detection_count += 1
            else:
                self.last_qr_data = data
                self.detection_count = 1
            rospy.loginfo((u"检测到二维码: '%s', 连续第 %d 次" % (data, self.detection_count)).encode('utf-8'))
            if self.detection_count >= self.consecutive_threshold:
                self.qr_detected = True
                self.qr_data = data
                return True, data
        else:
            self.last_qr_data = None
            self.detection_count = 0
        return False, None

    def play_audio_for_qr(self, data):
        data = unicode(data, 'utf-8', errors='ignore') if isinstance(data, str) else data
        filename = self.qr_audio_map.get(data)
        if filename:
            audio_path = os.path.join(self.audio_dir, unicode(filename, 'utf-8', errors='ignore'))
            if os.path.exists(audio_path):
                subprocess.Popen(['aplay', audio_path.encode('utf-8')])
                rospy.loginfo((u"🔊 正在播放语音: %s" % audio_path).encode('utf-8'))
            else:
                rospy.logwarn((u"语音文件不存在: %s" % audio_path).encode('utf-8'))
        else:
            rospy.logwarn((u"未找到 '%s' 对应的语音文件" % data).encode('utf-8'))

class ObjectDetector:
    """物体检测器"""
    def __init__(self, audio_config):
        self.category_items = {
            "fruit":     ["apple", "banana", "watermelon"],
            "vegetable": ["chilli", "tomato", "potato"],
            "dessert":   ["milk", "cake", "cola"],
        }
        self.img_w = 480
        self.img_h = 320
        self.cam_hfov = math.radians(124.8)
        self.camera_matrix = None
        self.focal_x = None
        self.center_x = None
        self.camera_info_received = False
        self.target_category = ""
        self.target_classes = []
        self.last_seen_time = rospy.Time(0)
        self.bearing_rad = 0.0
        self.vision_bearing_rad = 0.0
        self.lidar_bearing_rad = None
        self.target_dist = None
        self.box_width = None
        self.offset_norm = 0.0
        self.angle_Kp = 0.9
        self.angle_Ki = 0.08
        self.angle_Kd = 0.25
        self.lateral_Kp = 0.5
        self.lateral_Ki = 0.0
        self.lateral_Kd = 0.08
        self.forward_Kp = 0.8
        self.forward_Ki = 0.05
        self.forward_Kd = 0.22
        self.target_width_m = 0.05
        self.max_lat = 0.18
        self.max_lin = 0.25
        self.max_ang = 0.6
        self.stop_min = 0.35
        self.stop_max = 0.45
        self.target_distance = (self.stop_min + self.stop_max) / 2.0
        self.target_dist_filtered = None
        self.dist_filter_alpha = 0.3
        self.angle_ = 0.0
        self.pid_angle = PIDController(Kp=self.angle_Kp, Ki=self.angle_Ki, Kd=self.angle_Kd, output_min=-self.max_ang, output_max=self.max_ang)
        self.pid_lateral = PIDController(Kp=self.lateral_Kp, Ki=self.lateral_Ki, Kd=self.lateral_Kd, output_min=-self.max_lat, output_max=self.max_lat)
        self.pid_forward = PIDController(Kp=self.forward_Kp, Ki=self.forward_Ki, Kd=self.forward_Kd, output_min=-self.max_lin, output_max=self.max_lin)
        self.search_direction = 1.0
        self.just_detected = False
        self.stable_track_time = rospy.Time(0)
        self.reached = False
        self.ok_counter = 0
        self.audio_dir = audio_config.get('object_dir', '')
        self.current_detected_object = None
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_cb)
        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_cb)
        self.update_task_from_param(force=True)

    def update_task_from_param(self, force=False):
        category = rospy.get_param('/procurement_task', '').strip().lower()
        if category != self.target_category or force:
            self.target_category = category
            self.target_classes = self.category_items.get(category, [])
            rospy.loginfo((u"[任务更新] 当前目标大类：{0} → 具体物品列表：{1}".format(category, self.target_classes)).encode('utf-8'))

    def camera_info_cb(self, msg):
        if not self.camera_info_received:
            self.focal_x = msg.K[0]
            self.center_x = msg.K[2]
            self.img_w = msg.width
            self.cam_hfov = 2.0 * math.atan(self.img_w / (2.0 * self.focal_x)) if self.focal_x > 0 else math.radians(120)
            self.camera_info_received = True
            rospy.loginfo((u"📷 相机信息已接收: f=%.1f, c=%.1f" % (self.focal_x, self.center_x)).encode('utf-8'))

    def calculate_accurate_bearing(self, center_x):
        if self.camera_info_received and self.focal_x:
            return math.atan((center_x - self.center_x) / self.focal_x)
        return ((center_x - self.img_w / 2.0) / (self.img_w / 2.0)) * (self.cam_hfov / 2.0)

    def calculate_vision_distance(self):
        if self.camera_info_received and self.box_width and self.focal_x:
            if self.box_width > 10:
                dist = (self.focal_x * self.target_width_m) / self.box_width
                if 0.1 < dist < 5.0: return dist
        return None

    def find_closest_object_angle(self, scan):
        min_dist, closest_angle = float('inf'), None
        for i, dist in enumerate(scan.ranges):
            if 0.1 < dist < 3.0:
                if dist < min_dist:
                    min_dist, closest_angle = dist, scan.angle_min + i * scan.angle_increment
        return closest_angle if abs(closest_angle or 0) < math.radians(45) else None

    def yolo_cb(self, msg):
        if not self.target_classes: return
        best_box, best_prob, detected_object = None, 0.0, None
        for box in msg.bounding_boxes:
            if box.Class.lower() in self.target_classes and box.probability > best_prob:
                best_prob, best_box, detected_object = box.probability, box, box.Class.lower()
        if best_box:
            center_x = (best_box.xmin + best_box.xmax) / 2.0
            self.box_width = best_box.xmax - best_box.xmin
            raw_bearing = self.calculate_accurate_bearing(center_x)
            self.vision_bearing_rad = 0.7 * self.vision_bearing_rad + 0.3 * raw_bearing
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 0.5:
                self.just_detected, self.stable_track_time = True, rospy.Time.now()
            self.last_seen_time = rospy.Time.now()
            self.current_detected_object = detected_object

    def lidar_cb(self, scan):
        if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
            self.target_dist = None
            return
        
        vision_dist = self.calculate_vision_distance()
        lidar_dist = None
        target_angle = self.vision_bearing_rad
        ang = max(scan.angle_min, min(scan.angle_max, target_angle))
        index = int((ang - scan.angle_min) / scan.angle_increment)
        
        valid_distances = [d for d in scan.ranges[max(0, index-3):min(len(scan.ranges), index+4)] if 0.05 < d < 5.0]
        if valid_distances: lidar_dist = np.median(valid_distances)

        fused_dist = lidar_dist if lidar_dist else vision_dist # Prioritize lidar
        if fused_dist:
            if self.target_dist_filtered is None: self.target_dist_filtered = fused_dist
            else: self.target_dist_filtered = (1 - self.dist_filter_alpha) * self.target_dist_filtered + self.dist_filter_alpha * fused_dist
            self.target_dist = self.target_dist_filtered

    def play_audio_for_object(self, object_name):
        audio_path = os.path.join(self.audio_dir, "{0}.wav".format(object_name))
        if os.path.exists(audio_path):
            subprocess.Popen(['aplay', audio_path])
            rospy.loginfo((u"🔊 正在播放语音: %s" % audio_path).encode('utf-8'))
        else:
            rospy.logwarn((u"语音文件不存在: %s" % audio_path).encode('utf-8'))

    def run_object_detection(self):
        rospy.loginfo(u"🔍 开始物体检测和微调...".encode('utf-8'))
        self.reset_detector_state()
        rate, last_time = rospy.Rate(20), rospy.Time.now()

        start_time, detection_timeout = rospy.Time.now(), 8 # Increased timeout 8s
        
        while not rospy.is_shutdown():
            dt = (rospy.Time.now() - last_time).to_sec()
            if dt <= 0: continue
            last_time = rospy.Time.now()

            if (rospy.Time.now() - start_time).to_sec() > detection_timeout:
                rospy.logwarn(u"⏰ 物体检测超时".encode('utf-8'))
                break
            
            self.update_task_from_param()
            if self.reached: break

            twist = Twist()
            if self.target_dist and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
                self.angle_ = self.vision_bearing_rad
                twist.angular.z = self.pid_angle.compute(self.angle_, dt)
                if abs(self.angle_) < math.radians(12):
                    dist_error = self.target_dist - self.target_distance
                    twist.linear.x = self.pid_forward.compute(dist_error, dt)
                else: self.pid_forward.reset()

                if abs(self.angle_) < math.radians(5) and self.stop_min < self.target_dist < self.stop_max:
                    self.ok_counter += 1
                else: self.ok_counter = 0

                if self.ok_counter >= 4:
                    self.reached = True
                    rospy.loginfo(u"✅ 已安全到位并锁定！".encode('utf-8'))
            else:
                twist.angular.z = self.search_direction * 0.35
                self.pid_angle.reset(), self.pid_forward.reset()
                rospy.logwarn_throttle(2.0, u"未找到目标，正在搜索...".encode('utf-8'))

            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        return self.reached, self.current_detected_object

    def reset_detector_state(self):
        self.reached, self.ok_counter, self.just_detected = False, 0, False
        self.last_seen_time, self.stable_track_time = rospy.Time(0), rospy.Time(0)
        self.pid_angle.reset(), self.pid_lateral.reset(), self.pid_forward.reset()
        self.bearing_rad, self.vision_bearing_rad, self.lidar_bearing_rad = 0.0, 0.0, None
        self.target_dist, self.box_width, self.target_dist_filtered = None, None, None
        rospy.loginfo(u"🔄 检测器状态已重置".encode('utf-8'))

class Navigator(object):
    def __init__(self, csv_path, tuning_path, config_path):
        self.tf = tf.TransformListener()
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo(u"正在等待 move_base action 服务器...".encode('utf-8'))
        self.mb.wait_for_server()
        rospy.loginfo(u"✅ move_base action 服务器已连接。".encode('utf-8'))

        self.tuning_cfg = self.load_yaml(tuning_path, self.get_default_tuning())
        self.app_config = self.load_yaml(config_path)
        if not self.app_config:
            rospy.logerr(u"❌ 主配置文件 config.yaml 加载失败，无法继续！".encode('utf-8'))
            rospy.signal_shutdown("Config load failed")
            return

        self.wps = self.load_csv(csv_path)
        self.pid_x, self.pid_y = PID(**self.tuning_cfg['pid_lin']), PID(**self.tuning_cfg['pid_lin'])
        self.pid_yaw = PID(**self.tuning_cfg['pid_yaw'])

        audio_cfg = self.app_config.get('audio', {})
        self.qr_detector = QRCodeDetector(audio_cfg)
        self.object_detector = ObjectDetector(audio_cfg)
        self.traffic_light_detector = TrafficLightDetector()

        self.detected_objects = []
        self.usb_cam_process = None
    
        # 为“请求-响应”通信功能进行初始化
        self.rosbridge_process = None
        self.task_assignment_data = None
        self.task_assignment_received_event = threading.Event()
        self.result_sub = rospy.Subscriber("/recognition_result", String, self._task_assignment_callback, queue_size=1)
        rospy.loginfo(u"✅ 已订阅 /recognition_result 话题，用于接收远程任务指派。".encode('utf-8'))

    def get_default_tuning(self):
        return {'pos_tol': 0.04, 'yaw_tol_deg': 2.0, 'max_nav_time': 300, 'refine_timeout': 15.0,
                'pid_lin': {'kp': 1.2, 'ki': 0.0, 'kd': 0.05}, 'pid_yaw': {'kp': 1.8, 'ki': 0.0, 'kd': 0.1},
                'max_lin': 0.35, 'max_ang': 1.5, 'min_lin': 0.10, 'min_ang': 0.25}

    def load_yaml(self, path, defaults=None):
        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            if defaults:
                for k, v in defaults.items(): cfg.setdefault(k, v)
                if 'yaw_tol_deg' in cfg: cfg['yaw_tol'] = math.radians(cfg['yaw_tol_deg'])
            rospy.loginfo((u"✅ 成功加载配置文件: %s" % path).encode('utf-8'))
            return cfg
        except Exception as e:
            rospy.logwarn((u'⚠️ 无法读取 %s: %s。将使用默认值。' % (path, str(e))).encode('utf-8'))
            if defaults and 'yaw_tol_deg' in defaults: defaults['yaw_tol'] = math.radians(defaults['yaw_tol_deg'])
            return defaults or {}

    @staticmethod
    def load_csv(path):
        wps = []
        try:
            with open(path, 'r') as f:
                for row in csv.reader(f):
                    if len(row) >= 5 and row[0].lower() != 'tag':
                        wps.append({'tag': row[0], 'x': float(row[1]), 'y': float(row[2]), 'qz': float(row[3]), 'qw': float(row[4])})
            rospy.loginfo((u'✅ 已从 %s 加载 %d 个路径点' % (os.path.basename(path), len(wps))).encode('utf-8'))
        except Exception as e:
            rospy.logerr((u"❌ 加载路径点文件 %s 失败: %s" % (path, str(e))).encode('utf-8'))
        return wps

    def _task_assignment_callback(self, msg):
        if not self.task_assignment_received_event.is_set():
            rospy.loginfo((u"📥 [任务回调] 成功收到远程回应: %s" % msg.data).encode('utf-8'))
            self.task_assignment_data = msg.data
            self.task_assignment_received_event.set()
        else:
            rospy.logwarn((u"⚠️ 在非等待状态下收到消息，已忽略: %s" % msg.data).encode('utf-8'))


    def play_room_audio(self, room_name):
        audio_cfg = self.app_config.get('audio', {})
        audio_map, audio_dir = audio_cfg.get('room_audio_map', {}), audio_cfg.get('room_audio_dir', '')
        filename = audio_map.get(room_name)
        if not filename:
            rospy.logwarn((u"⚠️ 未在 config.yaml 中找到房间 '%s' 对应的语音文件映射。" % room_name).encode('utf-8'))
            return
        if not audio_dir:
            rospy.logwarn(u"⚠️ 房间语音目录 'room_audio_dir' 未在 config.yaml 中配置!".encode('utf-8'))
            return
        audio_path = os.path.join(audio_dir, filename)
        if os.path.exists(audio_path):
            subprocess.Popen(['aplay', audio_path])
            rospy.loginfo((u"🔊 正在播放房间语音: %s" % audio_path).encode('utf-8'))
        else:
            rospy.logwarn((u"房间语音文件不存在: %s" % audio_path).encode('utf-8'))

    # ==================== 模块化动作函数 ====================

    def _action_detect_qr(self, point_idx):
        rospy.loginfo(u'🔍 [动作] 开始二维码识别...'.encode('utf-8'))
        rate, timeout, start_time = rospy.Rate(10), 60, rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < timeout:
            detected, qr_data = self.qr_detector.detect_qr_code()
            if detected:
                qr_data_lower = qr_data.lower()
                rospy.set_param("/procurement_task", qr_data_lower)
                self.qr_detector.play_audio_for_qr(qr_data)
                rospy.loginfo((u'✅ 二维码识别完成，任务类别设置为: %s' % qr_data_lower).encode('utf-8'))
                time.sleep(2)
                return {'status': 'success', 'qr_data': qr_data}
            rate.sleep()
        rospy.logwarn(u'⏰ 二维码识别超时'.encode('utf-8'))
        rospy.set_param("/procurement_task", "unknown") # 设置默认值以防后续出错
        return {'status': 'timeout'}

    def _action_detect_object(self, point_idx):
        rospy.loginfo((u'🔍 [动作] 到达第%d点, 开始物体识别...' % point_idx).encode('utf-8'))
        detected, object_name = self.object_detector.run_object_detection()
        if detected and object_name:
            if object_name not in self.detected_objects:
                self.detected_objects.append(object_name)
                rospy.loginfo((u'✅ 检测到目标物体: %s' % object_name).encode('utf-8'))
                time.sleep(0.5)
                self.object_detector.play_audio_for_object(object_name)
                rospy.set_param("/detected_objects", self.detected_objects)
                rospy.loginfo((u'📝 已记录物体: %s. 当前列表: %s' % (object_name, self.detected_objects)).encode('utf-8'))
                return {'status': 'success', 'object_name': object_name, 'target_found': True}
            else:
                rospy.loginfo((u'⚠️ 物体 %s 已经检测过了' % object_name).encode('utf-8'))
                return {'status': 'duplicate'}
        rospy.loginfo((u'❌ 在第%d点未检测到目标物体' % point_idx).encode('utf-8'))
        return {'status': 'not_found'}

    # [MODIFIED] 重构交通灯处理函数以支持配置化跳转
    def _action_handle_traffic_light(self, point_idx):
        """
        [重构后] 通用的交通灯处理动作。
        该函数会执行检测，如果检测到绿灯，会从配置文件查找并返回需要跳转的目标点。
        """
        rospy.loginfo((u'🚦 [动作] 到达第%d点，开始内置交通灯识别...' % point_idx).encode('utf-8'))
        self.cmd_pub.publish(Twist()) # 确保车辆停止
        start_time, detection_time = rospy.Time.now(), 10.0
        green_count, red_count = 0, 0
        
        rospy.loginfo((u'⏰ 开始%d秒交通灯检测...' % detection_time).encode('utf-8'))
        while (rospy.Time.now() - start_time).to_sec() < detection_time and not rospy.is_shutdown():
            status = self.traffic_light_detector.get_status()
            if status == 'green': green_count += 1
            elif status == 'red': red_count += 1
            rospy.sleep(0.1)
        
        rospy.loginfo((u'📋 检测完成，统计: 绿灯 %d 次, 红灯 %d 次' % (green_count, red_count)).encode('utf-8'))

        # 判断条件：绿灯次数大于红灯，并且至少稳定检测到20次（可根据实际情况调整）
        if green_count > red_count and green_count >= 20:
            rospy.loginfo(u'✅ 判定为绿灯！'.encode('utf-8'))
            self.play_traffic_light_audio(point_idx) # 播放通过音效

            # 从配置中读取跳转规则
            rules = self.app_config.get('traffic_light_rules', {})
            jump_target = rules.get(point_idx)

            if jump_target:
                rospy.loginfo((u"根据 traffic_light_rules 配置，找到跳转目标: %d" % jump_target).encode('utf-8'))
                return {'status': 'green_light', 'jump_target': jump_target}
            else:
                rospy.logwarn((u"⚠️ 在第%d点检测到绿灯，但在 config.yaml 中未找到对应的跳转规则！将继续直行。" % point_idx).encode('utf-8'))
                return {'status': 'green_light', 'jump_target': None} 
        else:
            rospy.loginfo(u'❌ 判定为红灯或无信号，将继续前往下一个导航点。'.encode('utf-8'))
            return {'status': 'red_light'}
            
    def _action_request_info_and_switch_camera(self, point_idx):
        rospy.loginfo((u'🔄 [宏动作] 到达第%d点, 开始与外部设备通信...' % point_idx).encode('utf-8'))
        self.cmd_pub.publish(Twist())

        task_assignment_result = self._internal_request_and_wait_for_task()
        if task_assignment_result['status'] == 'failed':
            rospy.logerr(u"❌ 通信子任务失败，但仍将尝试切换摄像头。".encode('utf-8'))
        else:
            rospy.loginfo(u"✅ 通信子任务完成（或超时），继续下一步。".encode('utf-8'))
            
        rospy.loginfo(u"--- 延时2秒后，开始执行下一个子任务 ---".encode('utf-8'))
        time.sleep(2)

        camera_switch_result = self._internal_launch_special_camera()
        return camera_switch_result
        
    def _internal_request_and_wait_for_task(self):
        rospy.loginfo(u'📡 [子任务] 开始执行“请求-响应”通信...'.encode('utf-8'))

        try:
            rospy.loginfo(u'🚀 正在启动 rosbridge_server...'.encode('utf-8'))
            command = ['roslaunch', 'rosbridge_server', 'rosbridge_websocket.launch']
            self.rosbridge_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(3)
            if self.rosbridge_process.poll() is not None:
                raise RuntimeError("rosbridge_server 启动后立即退出")
        except Exception as e:
            rospy.logerr((u"❌ 启动 rosbridge_server 失败: %s" % str(e)).encode('utf-8'))
            return {'status': 'failed', 'reason': 'rosbridge_launch_failed'}

        try:
            category = rospy.get_param('/procurement_task', None)
            if not category:
                rospy.logerr(u"❌ 无法从参数服务器 /procurement_task 获取任务类别，通信中止！".encode('utf-8'))
                stop_process(self.rosbridge_process, u"rosbridge 进程")
                return {'status': 'failed', 'reason': 'missing_procurement_task_param'}
            
            task_pub = rospy.Publisher('/procurement_task', String, queue_size=1, latch=True)
            task_pub.publish(String(data=category))
            rospy.loginfo((u"📤 已向 /procurement_task 发布任务类别: '%s'" % category).encode('utf-8'))
            time.sleep(1.0)
        except Exception as e:
            rospy.logerr((u"❌ 发布任务类别时出错: %s" % str(e)).encode('utf-8'))
            stop_process(self.rosbridge_process, u"rosbridge 进程")
            return {'status': 'failed', 'reason': 'publish_failed'}

        self.task_assignment_received_event.clear()
        timeout = 5
        rospy.loginfo((u'⏳ 等待远程设备回应中 (超时时间: %d 秒)...' % timeout).encode('utf-8'))
        received = self.task_assignment_received_event.wait(timeout)
        
        stop_process(self.rosbridge_process, u"rosbridge 进程")
        self.rosbridge_process = None

        if not received:
            rospy.logwarn(u"⏰ 等待远程回应超时！将继续执行后续任务。".encode('utf-8'))
            return {'status': 'timeout'}

        try:
            data_str = self.task_assignment_data.strip()
            if not data_str: raise ValueError("收到的消息为空。")
            
            if data_str == "not_found" or data_str == "unknown_category":
                rospy.logwarn((u"⚠️ 远程设备未能找到匹配项，结果: '%s'" % data_str).encode('utf-8'))
                return {'status': 'not_found_remotely'}
            
            parts = data_str.split(',')
            if len(parts) == 2:
                cargo, room = parts[0].strip().lower(), parts[1].strip().upper()
                rospy.loginfo((u"📥 解析回应成功 -> 货物: '%s', 房间: '%s'" % (cargo, room)).encode('utf-8'))
                if cargo and cargo not in self.detected_objects:
                    self.detected_objects.append(cargo)
                    rospy.set_param("/detected_objects", self.detected_objects)
                if room: self.play_room_audio(room)
                return {'status': 'success', 'cargo': cargo, 'room': room}
            else:
                rospy.logwarn((u"⚠️ 收到的回应格式不正确: %s" % self.task_assignment_data).encode('utf-8'))
                return {'status': 'failed', 'reason': 'invalid_message_format'}
        except Exception as e:
            rospy.logerr((u"❌ 处理回应消息时出错: %s" % str(e)).encode('utf-8'))
            return {'status': 'failed', 'reason': 'processing_error'}

    def _internal_launch_special_camera(self):
        rospy.loginfo(u'📷 [子任务] 开始执行特殊摄像头操作...'.encode('utf-8'))
        rospy.loginfo(u'🛑 机器人将在此处静止10秒...'.encode('utf-8'))
        self.cmd_pub.publish(Twist())
        time.sleep(10)
        rospy.loginfo(u'⏰ 10秒静止时间到，开始启动 ucar_camera...'.encode('utf-8'))
        self.usb_cam_process = launch_ucar_camera(self.usb_cam_process)
        if self.usb_cam_process:
            rospy.loginfo(u'✅ 特殊摄像头操作完成。'.encode('utf-8'))
            return {'status': 'success'}
        else:
            rospy.logerr(u'❌ 特殊摄像头启动失败。'.encode('utf-8'))
            return {'status': 'failed'}

    def _execute_point_action(self, point_idx):
        action_map = self.app_config.get('point_actions', {})
        action_name = action_map.get(point_idx)
        if not action_name: return None
        action_func_name = '_action_' + action_name
        action_func = getattr(self, action_func_name, None)
        if callable(action_func):
            return action_func(point_idx)
        else:
            rospy.logwarn((u"⚠️ 未找到动作 '%s' 对应的实现函数 %s。" % (action_name, action_func_name)).encode('utf-8'))
            return None
            
    # ==================== 主运行与导航逻辑 ====================

    # [MODIFIED] 重构主运行循环以处理新的交通灯和跳转逻辑
    def run(self):
        target_detected, traffic_light_logic_finished = False, False
        skip_to_point = None
        
        for idx, wp in enumerate(self.wps, 1):
            if skip_to_point and idx < skip_to_point:
                rospy.loginfo((u'⏭️ 跳过第%d个导航点' % idx).encode('utf-8'))
                continue
            skip_to_point = None

            rospy.loginfo((u'→ 处理 %d/%d (tag=%s)' % (idx, len(self.wps), wp['tag'])).encode('utf-8'))
            self.navigate(wp)

            action_name = self.app_config.get('point_actions', {}).get(idx)
            
            # 根据标志位，跳过不必要的重复检测
            if target_detected and action_name == 'detect_object':
                rospy.loginfo(u"ℹ️ 已找到目标，跳过此点的物体检测。".encode('utf-8'))
                continue
            if traffic_light_logic_finished and action_name == 'handle_traffic_light':
                rospy.loginfo(u"ℹ️ 交通灯逻辑已完成（已找到绿灯），跳过此点的交通灯检测。".encode('utf-8'))
                continue

            result = self._execute_point_action(idx)

            if result:
                # 逻辑分支1: 处理物体识别成功
                if result.get('target_found'):
                    target_detected = True
                    # 从配置文件动态加载跳转目标
                    jump_target = self.app_config.get('navigation_flow', {}).get('object_detection_jump_target')
                    if jump_target:
                        skip_to_point = jump_target
                        rospy.loginfo((u'⏭️ 目标已找到，从配置加载跳转目标，将跳转到第%d个导航点' % jump_target).encode('utf-8'))
                
                # [NEW LOGIC] 逻辑分支2: 处理交通灯绿灯
                elif result.get('status') == 'green_light':
                    jump_target = result.get('jump_target')
                    if jump_target:
                        # 设置跳转点
                        skip_to_point = jump_target
                        # 标记交通灯逻辑完成，以跳过下一个路口检测
                        traffic_light_logic_finished = True
                        rospy.loginfo((u'⏭️ 绿灯通过，将跳转到配置文件指定的导航点: %d' % jump_target).encode('utf-8'))

        rospy.loginfo(u"🏁 所有导航点处理完毕。".encode('utf-8'))
        self.print_task_summary()

    def play_traffic_light_audio(self, point_idx):
        audio_cfg = self.app_config.get('audio', {})
        filename = audio_cfg.get('traffic_light_map', {}).get(point_idx)
        if filename:
            audio_dir = audio_cfg.get('traffic_light_dir', '')
            audio_path = os.path.join(audio_dir, filename)
            if os.path.exists(audio_path):
                subprocess.Popen(['aplay', audio_path])
                rospy.loginfo((u"🔊 正在播放交通灯语音: %s" % audio_path).encode('utf-8'))
            else: rospy.logwarn((u"交通灯语音文件不存在: %s" % audio_path).encode('utf-8'))

    def print_task_summary(self):
        rospy.loginfo((u"=" * 50 + u"\n📋 任务摘要\n" + u"=" * 50).encode('utf-8'))
        procurement_task = rospy.get_param('/procurement_task', 'N/A').strip().lower()
        rospy.loginfo((u"🎯 任务类别: %s" % procurement_task).encode('utf-8'))
        target_classes = self.object_detector.category_items.get(procurement_task, [])
        rospy.loginfo((u"📝 目标列表: %s" % target_classes).encode('utf-8'))
        detected_objects = rospy.get_param("/detected_objects", [])
        rospy.loginfo((u"✅ 已检测到: %s" % detected_objects).encode('utf-8'))
        if target_classes:
            try:
                success_rate = len(set(detected_objects) & set(target_classes)) / float(len(target_classes)) * 100
                rospy.loginfo((u"📊 成功率: %.1f%%" % success_rate).encode('utf-8'))
            except ZeroDivisionError: pass
        rospy.loginfo((u"=" * 50).encode('utf-8'))

    def navigate(self, wp):
        goal = self.build_goal(wp)
        self.mb.send_goal(goal)
        if not self.wait_for_mb():
            rospy.logerr((u'❌ 粗导航失败 (tag=%s)' % wp['tag']).encode('utf-8'))
            return
        rospy.loginfo(u'✔ 粗导航完成'.encode('utf-8'))
        if wp.get('tag', '').lower() == 'frame':
            rospy.loginfo(u"...开始精细调整...".encode('utf-8'))
            self.refine(goal)

    def wait_for_mb(self):
        start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < self.tuning_cfg['max_nav_time']:
            state = self.mb.get_state()
            if state == GoalStatus.SUCCEEDED: return True
            if state in [GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                 rospy.logerr((u"MoveBase导航失败，状态: %d" % state).encode('utf-8'))
                 return False
            rospy.sleep(0.5)
        self.mb.cancel_goal()
        rospy.logerr(u"导航超时！".encode('utf-8'))
        return False

    def refine(self, goal):
        t0, rate, phase = rospy.Time.now(), rospy.Rate(20), 'trans'
        for pid in (self.pid_x, self.pid_y, self.pid_yaw): pid.reset()
        while (rospy.Time.now() - t0).to_sec() < self.tuning_cfg['refine_timeout']:
            dx_r, dy_r, yaw_err, dist = self.compute_errors(goal)
            if dist >= 999: break
            if phase == 'trans':
                if dist <= self.tuning_cfg['pos_tol']:
                    phase, _ = 'rot', [pid.reset() for pid in (self.pid_x, self.pid_y, self.pid_yaw)]
                    rospy.loginfo(u'✅ 平移完成 → 开始旋转'.encode('utf-8'))
                    continue
                cmd = self.cmd_trans(dx_r, dy_r)
            else:
                if abs(yaw_err) <= self.tuning_cfg['yaw_tol']:
                    rospy.loginfo(u'✅ 旋转完成'.encode('utf-8'))
                    break
                cmd = self.cmd_rot(yaw_err)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def compute_errors(self, goal):
        try:
            self.tf.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.2))
            (t, r) = self.tf.lookupTransform('map', 'base_link', rospy.Time(0))
            dx, dy = goal.target_pose.pose.position.x - t[0], goal.target_pose.pose.position.y - t[1]
            dist = math.hypot(dx, dy)
            yaw_curr, yaw_goal = q_to_yaw(Quaternion(*r)), q_to_yaw(goal.target_pose.pose.orientation)
            yaw_err = ang_diff(yaw_goal, yaw_curr)
            dx_r, dy_r = math.cos(yaw_curr) * dx + math.sin(yaw_curr) * dy, -math.sin(yaw_curr) * dx + math.cos(yaw_curr) * dy
            return dx_r, dy_r, yaw_err, dist
        except Exception as e:
            rospy.logwarn((u"计算误差时TF变换失败: %s" % str(e)).encode('utf-8'))
            return 0, 0, 0, 999
    
    def cmd_trans(self, dx_r, dy_r):
        dt, d = 1.0 / 20.0, math.hypot(dx_r, dy_r)
        v = max(-self.tuning_cfg['max_lin'], min(self.tuning_cfg['max_lin'], self.pid_x.step(d, dt)))
        ux, uy = (dx_r / d, dy_r / d) if d > 1e-4 else (0, 0)
        cmd = Twist()
        if abs(v) > self.tuning_cfg.get('min_lin', 0.1):
            cmd.linear.x, cmd.linear.y = v * ux, v * uy
        return cmd

    def cmd_rot(self, yaw_err):
        dt = 1.0 / 20.0
        w = max(-self.tuning_cfg['max_ang'], min(self.tuning_cfg['max_ang'], self.pid_yaw.step(yaw_err, dt)))
        cmd = Twist()
        if abs(w) > self.tuning_cfg.get('min_ang', 0.1): cmd.angular.z = w
        return cmd
    
    @staticmethod
    def build_goal(wp):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x, g.target_pose.pose.position.y = wp['x'], wp['y']
        g.target_pose.pose.orientation.z, g.target_pose.pose.orientation.w = wp['qz'], wp['qw']
        return g

# ---------------- main ---------------- #
if __name__ == '__main__':
    rospy.init_node('navigator_v175_configurable_tl', anonymous=False)
    root = os.path.dirname(os.path.abspath(__file__))
    usb_cam_process = None

    try:
        # launch_navigation() # 按需取消注释
        if not check_camera_status(): rospy.logwarn(u"⚠️ 摄像头状态检查失败，但仍将尝试启动。".encode('utf-8'))

        usb_cam_process = ensure_camera_is_running(None)
        if usb_cam_process is None: rospy.logwarn(u"⚠️ 默认摄像头启动失败，相关功能可能无法使用。".encode('utf-8'))

        rospy.loginfo(u"系统初始化完成，即将开始执行导航任务...".encode('utf-8'))
        
        nav = Navigator(
            csv_path=os.path.join(root, 'points.csv'),
            tuning_path=os.path.join(root, 'tuning.yaml'),
            config_path=os.path.join(root, 'config3.yaml')
        )
        
        if hasattr(nav, 'app_config') and nav.app_config:
            nav.usb_cam_process = usb_cam_process
            nav.run()
            usb_cam_process = nav.usb_cam_process
        else:
            rospy.logerr(u"Navigator 对象初始化失败，程序退出。".encode('utf-8'))

    except rospy.ROSInterruptException:
        rospy.loginfo(u"程序被中断。".encode('utf-8'))
    except Exception as e:
        rospy.logerr((u"发生未知顶层错误: %s" % str(e)).encode('utf-8'))
        import traceback
        traceback.print_exc()
    finally:
        # 确保在程序退出时，所有子进程都能被正确关闭
        if 'nav' in locals() and hasattr(nav, 'rosbridge_process'):
            stop_process(nav.rosbridge_process, u"最后的 rosbridge 进程")
        if 'usb_cam_process' in locals():
            stop_process(usb_cam_process, u"最后的摄像头进程")
        rospy.loginfo(u"所有任务完成，程序退出。".encode('utf-8'))