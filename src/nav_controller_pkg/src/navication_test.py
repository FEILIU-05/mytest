#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
navigation_controller.py  – V1.5.6 (Final Integrated)
----------------------------------
* 集成内置交通灯图像识别逻辑，不再依赖外部话题。
* 修复 Python 2 兼容性问题 (subprocess.TimeoutExpired)。
* 为第6个导航点创建专用的 ucar_camera 启动函数。
* 重构摄像头进程管理，实现安全的启动与重启逻辑。
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
import threading # 导入线程库
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

# ==================== 全局语音路径配置 ====================
QR_AUDIO_DIR = u"/home/ucar/Desktop/ucar/src/nav_controller_pkg/voice_packge/采购任务_wav"
OBJECT_AUDIO_DIR = "/home/ucar/Desktop/ucar/src/nav_controller_pkg/voice_packge/取得物品"
QR_AUDIO_MAP = {
    "Vegetable": "vegetable.wav",
    "Fruit":     "fruit.wav", 
    "Dessert":   "dessert.wav"
}
TRAFFIC_LIGHT_AUDIO_DIR = "/home/ucar/Desktop/ucar/src/nav_controller_pkg/voice_packge/路口"
TRAFFIC_LIGHT_AUDIO_MAP = {
    7: "1.wav",
    8: "2.wav"
}
# ======================================================

# ==================== (重构后) 进程与节点控制 ====================

def stop_process(process, process_name="进程"):
    """
    一个健壮和通用的进程关闭函数 (Python 2 兼容版)。
    它会先尝试优雅地关闭进程，如果超时则强制终止。
    """
    if process is None or process.poll() is not None:
        rospy.loginfo("ℹ️ %s 无需关闭（不存在或已结束）。", process_name)
        return

    rospy.loginfo("🔪 准备关闭 %s...", process_name)
    try:
        process.send_signal(signal.SIGINT)
        
        timeout_sec = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if process.poll() is not None:
                rospy.loginfo("✅ %s 已成功关闭。", process_name)
                return
            time.sleep(0.1)

        rospy.logwarn("⚠️ %s 关闭超时，将强制终止！", process_name)
        process.kill()
        process.wait()
        rospy.loginfo("✅ %s 已被强制终止。", process_name)
        
    except Exception as e:
        rospy.logerr("❌ 关闭 %s 时发生未知错误: %s", process_name, str(e))

def ensure_camera_is_running(current_process):
    """
    确保默认摄像头(usb_cam)正在运行。
    如果已有进程在运行，会先将其关闭，然后再启动一个新的。
    """
    rospy.loginfo("🔄 准备启动/重启默认摄像头...")
    stop_process(current_process, "旧摄像头进程")

    rospy.loginfo("🚀 正在启动新的 usb_cam 节点...")
    if not os.path.exists('/dev/video0'):
        rospy.logerr("❌ 摄像头设备 /dev/video0 不存在，无法启动。")
        return None
    
    try:
        command = ['roslaunch', 'usb_cam', 'usb_cam-test.launch']
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(3) 
        if process.poll() is None:
            rospy.loginfo("✅ 默认摄像头进程已成功启动。")
            return process
        else:
            _, stderr = process.communicate()
            rospy.logerr("❌ 默认摄像头启动失败，错误: %s", stderr.decode())
            return None
            
    except Exception as e:
        rospy.logerr("❌ 启动 usb_cam-test.launch 时发生异常: %s", str(e))
        return None

def launch_ucar_camera(current_process):
    """
    启动 ucar_camera 的 launch 文件，用于特定任务阶段。
    此函数会先关闭传入的旧进程，然后启动新的 ucar_camera 进程。
    """
    rospy.loginfo("📷 准备启动专用的 ucar_camera...")
    stop_process(current_process, "旧摄像头进程")

    rospy.loginfo("🚀 正在启动 roslaunch ucar_camera usb_cam-test.launch...")
    command = ['roslaunch', 'ucar_camera', 'usb_cam-test.launch']
    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(3) 
        if process.poll() is None:
            rospy.loginfo("✅ ucar_camera 已成功启动。")
            return process
        else:
            _, stderr = process.communicate()
            rospy.logerr("❌ ucar_camera 启动失败，错误: %s", stderr.decode())
            return None
            
    except Exception as e:
        rospy.logerr("❌ 启动 ucar_camera launch 文件时发生异常: %s", str(e))
        return None


def launch_navigation():
    rospy.loginfo("🚀 即将启动导航系统...")
    command = ['roslaunch', 'nav_controller_pkg', 'nav.launch']
    try:
        subprocess.Popen(command)
        rospy.loginfo("已发送 nav.launch 启动命令。等待 10 秒...")
        time.sleep(10)
        rospy.loginfo("✅ 导航系统应已准备就绪。")
    except Exception as e:
        rospy.logerr("❌ 启动 nav.launch 失败: %s", str(e))

def check_camera_status():
    rospy.loginfo("🔍 检查摄像头状态...")
    camera_devices = ['/dev/video0', '/dev/video1', '/dev/video2']
    available_devices = []
    
    for device in camera_devices:
        if os.path.exists(device):
            try:
                import stat
                device_stat = os.stat(device)
                permissions = "可读" if (device_stat.st_mode & stat.S_IRUSR) else "不可读"
                rospy.loginfo("📷 发现摄像头设备: %s (%s)", device, permissions)
                available_devices.append(device)
            except Exception as e:
                rospy.logwarn("⚠️ 无法检查设备 %s: %s", device, str(e))
        else:
            rospy.loginfo("❌ 摄像头设备 %s 不存在", device)
    
    if not available_devices:
        rospy.logerr("❌ 未找到可用的摄像头设备")
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
        rospy.loginfo("初始化内置交通灯检测器...")
        
        self.min_area = 500
        self.aspect_ratio_range = (0.5, 2.0)
        self.confidence_threshold = 800

        self.bridge = CvBridge()
        self.current_status = "unknown"
        self.status_lock = threading.Lock()

        # 注意: 确保在第6个点之后，这个话题是由 ucar_camera 发布的
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


class QRCodeDetector:
    """二维码检测器"""
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.latest_frame = None
        self.qr_audio_map = QR_AUDIO_MAP
        self.audio_dir = QR_AUDIO_DIR
        self.consecutive_threshold = 1
        self.last_qr_data = None
        self.detection_count = 0
        self.qr_detected = False
        self.qr_data = None

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("图像转换错误: %s" % str(e))

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

    def detect_qrcode(self, gray_img):
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
        """检测二维码，返回是否检测到以及检测到的数据"""
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

            rospy.loginfo("检测到二维码: '%s', 连续第 %d 次" % (data.encode('utf-8'), self.detection_count))

            if self.detection_count >= self.consecutive_threshold:
                rospy.loginfo("✅ 成功识别二维码: '%s'" % data.encode('utf-8'))
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
                try:
                    subprocess.Popen(['aplay', audio_path.encode('utf-8')])
                    rospy.loginfo("🔊 正在播放语音: %s" % audio_path.encode('utf-8'))
                except Exception as e:
                    rospy.logwarn("语音播放失败: %s" % str(e))
            else:
                rospy.logwarn("语音文件不存在: %s" % audio_path.encode('utf-8'))
        else:
            rospy.logwarn("未找到 '%s' 对应的语音文件" % data.encode('utf-8'))


class ObjectDetector:
    """物体检测器"""
    def __init__(self):
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
        self.focal_y = None
        self.center_x = None
        self.center_y = None
        self.camera_info_received = False
        
        self.target_category = ""
        self.target_classes = []
        self.last_seen_time = rospy.Time(0)
        self.bearing_rad = 0.0
        self.vision_bearing_rad = 0.0
        self.lidar_bearing_rad = None
        self.target_dist = None
        self.closest_dist = None
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
        self.target_height_m = 0.08
        
        self.max_lat = 0.18
        self.max_lin = 0.25
        self.max_ang = 0.6
        self.stop_min = 0.35
        self.stop_max = 0.45
        self.target_distance = (self.stop_min + self.stop_max) / 2.0
        
        self.target_dist_filtered = None
        self.vision_dist = None
        self.lidar_dist_raw = None
        self.dist_filter_alpha = 0.3
        
        self.angle_last_ = 0.0
        self.offset_last = 0.0
        self.steering_cumulative = 0.0
        self.offset_cumulative = 0.0
        self.w_threshold = 155
        self.angle_ = 0.0
        self.box_width = None
        
        self.pid_angle = PIDController(
            Kp=self.angle_Kp, Ki=self.angle_Ki, Kd=self.angle_Kd,
            output_min=-self.max_ang, output_max=self.max_ang, integral_max=5.0
        )
        self.pid_lateral = PIDController(
            Kp=self.lateral_Kp, Ki=self.lateral_Ki, Kd=self.lateral_Kd,
            output_min=-self.max_lat, output_max=self.max_lat, integral_max=3.0
        )
        self.pid_forward = PIDController(
            Kp=self.forward_Kp, Ki=self.forward_Ki, Kd=self.forward_Kd,
            output_min=-self.max_lin, output_max=self.max_lin, integral_max=2.0
        )
        
        self.search_direction = 1.0
        self.just_detected = False
        self.stable_track_time = rospy.Time(0)
        self.reached = False
        self.ok_counter = 0
        
        self.detected_objects = []
        self.audio_dir = OBJECT_AUDIO_DIR
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
            rospy.loginfo("[任务更新] 当前目标大类：{0} → 具体物品列表：{1}".format(category, self.target_classes))

    def camera_info_cb(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = msg.K
            self.focal_x = msg.K[0]
            self.focal_y = msg.K[4]
            self.center_x = msg.K[2]
            self.center_y = msg.K[5]
            
            self.img_w = msg.width
            self.img_h = msg.height
            
            if self.focal_x > 0:
                self.cam_hfov = 2.0 * math.atan(self.img_w / (2.0 * self.focal_x))
            
            self.camera_info_received = True
            rospy.loginfo("📷 相机信息已接收")

    def calculate_accurate_bearing(self, center_x):
        if self.camera_info_received and self.focal_x is not None:
            pixel_offset = center_x - self.center_x
            bearing_rad = math.atan(pixel_offset / self.focal_x)
            return bearing_rad
        else:
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            return raw_offset * (self.cam_hfov / 2.0)

    def calculate_vision_distance(self):
        if not self.camera_info_received or self.box_width is None:
            return None
        
        if self.focal_x is None or self.target_width_m <= 0:
            return None
            
        if self.box_width > 10:
            vision_dist = (self.focal_x * self.target_width_m) / self.box_width
            if 0.1 < vision_dist < 5.0:
                return vision_dist
        return None

    def fuse_angles(self, vision_angle, lidar_angle):
        if lidar_angle is None:
            return vision_angle
        
        time_since_detection = (rospy.Time.now() - self.stable_track_time).to_sec()
        angle_diff = abs(vision_angle - lidar_angle)
        
        if angle_diff > math.radians(30):
            return vision_angle
        
        if time_since_detection < 2.0:
            weight_lidar = 0.3
            weight_vision = 0.7
        elif angle_diff < math.radians(10):
            weight_lidar = 0.7
            weight_vision = 0.3
        else:
            weight_lidar = 0.5
            weight_vision = 0.5
            
        fused_angle = weight_lidar * lidar_angle + weight_vision * vision_angle
        return fused_angle

    def find_closest_object_angle(self, scan):
        min_dist = float('inf')
        closest_angle = None
        search_range = math.radians(45)
        cluster_angles = []
        cluster_distances = []
        
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > search_range:
                continue
            
            dist = scan.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and 0.1 < dist < 3.0:
                cluster_angles.append(angle)
                cluster_distances.append(dist)
                
                if dist < min_dist:
                    min_dist = dist
                    closest_angle = angle
        
        if closest_angle is not None and min_dist < 2.0:
            nearby_angles = []
            nearby_weights = []
            
            for angle, dist in zip(cluster_angles, cluster_distances):
                if abs(angle - closest_angle) < math.radians(5) and abs(dist - min_dist) < 0.2:
                    nearby_angles.append(angle)
                    weight = 1.0 / (dist + 0.1)
                    nearby_weights.append(weight)
            
            if nearby_angles:
                total_weight = sum(nearby_weights)
                if total_weight > 0:
                    weighted_angle = sum(a * w for a, w in zip(nearby_angles, nearby_weights)) / total_weight
                    self.closest_dist = min_dist
                    return weighted_angle
        
        return None

    def yolo_cb(self, msg):
        if not self.target_classes:
            return

        best_box = None
        best_prob = 0.0
        detected_object = None
        
        for box in msg.bounding_boxes:
            cls_name = box.Class.lower()
            if cls_name in self.target_classes and box.probability > best_prob:
                best_prob = box.probability
                best_box = box
                detected_object = cls_name

        if best_box:
            center_x = (best_box.xmin + best_box.xmax) / 2.0
            self.box_width = best_box.xmax - best_box.xmin
            
            raw_bearing = self.calculate_accurate_bearing(center_x)
            
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            self.offset_norm = 0.7 * self.offset_norm + 0.3 * raw_offset
            self.vision_bearing_rad = 0.7 * self.vision_bearing_rad + 0.3 * raw_bearing
            
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 0.5:
                self.just_detected = True
                self.stable_track_time = rospy.Time.now()
                rospy.loginfo("🎯 重新检测到目标，开始稳定跟踪")
            
            self.last_seen_time = rospy.Time.now()
            self.bearing_rad = self.fuse_angles(self.vision_bearing_rad, self.lidar_bearing_rad)
            
            self.current_detected_object = detected_object
            
            rospy.loginfo_throttle(3.0, "检测到 [{0}] 置信度 {1:.2f}".format(best_box.Class, best_prob))

    def lidar_cb(self, scan):
        if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
            self.target_dist = None
            self.lidar_bearing_rad = None
            return

        self.lidar_bearing_rad = self.find_closest_object_angle(scan)
        
        target_angle = self.vision_bearing_rad
        if self.lidar_bearing_rad is not None:
            target_angle = self.lidar_bearing_rad
            
        ang = max(scan.angle_min, min(scan.angle_max, target_angle))
        index = int((ang - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        
        valid_distances = []
        search_radius = 3
        
        for i in range(max(0, index - search_radius), min(len(scan.ranges), index + search_radius + 1)):
            dist = scan.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and 0.05 < dist < 5.0:
                valid_distances.append(dist)
        
        if valid_distances:
            valid_distances.sort()
            n = len(valid_distances)
            if n % 2 == 0:
                self.lidar_dist_raw = (valid_distances[n//2-1] + valid_distances[n//2]) / 2.0
            else:
                self.lidar_dist_raw = valid_distances[n//2]
        else:
            self.lidar_dist_raw = None
            
        self.vision_dist = self.calculate_vision_distance()
        fused_dist = None
        
        if self.lidar_dist_raw is not None and self.vision_dist is not None:
            dist_diff = abs(self.lidar_dist_raw - self.vision_dist)
            if dist_diff < 0.5:
                fused_dist = 0.7 * self.lidar_dist_raw + 0.3 * self.vision_dist
            else:
                fused_dist = self.lidar_dist_raw
        elif self.lidar_dist_raw is not None:
            fused_dist = self.lidar_dist_raw
        elif self.vision_dist is not None:
            fused_dist = self.vision_dist
        
        if fused_dist is not None:
            if self.target_dist_filtered is None:
                self.target_dist_filtered = fused_dist
            else:
                self.target_dist_filtered = (1 - self.dist_filter_alpha) * self.target_dist_filtered + \
                                          self.dist_filter_alpha * fused_dist
            self.target_dist = self.target_dist_filtered
        else:
            self.target_dist = None

    def play_audio_for_object(self, object_name):
        audio_path = os.path.join(self.audio_dir, "{0}.wav".format(object_name))
        if os.path.exists(audio_path):
            try:
                subprocess.Popen(['aplay', audio_path])
                rospy.loginfo("🔊 正在播放语音: %s" % audio_path)
            except Exception as e:
                rospy.logwarn("语音播放失败: %s" % str(e))
        else:
            rospy.logwarn("语音文件不存在: %s" % audio_path)

    def run_object_detection(self):
        rospy.loginfo("🔍 开始物体检测和微调...")
        self.reset_detector_state()
        
        rate = rospy.Rate(20)
        last_time = rospy.Time.now()
        #超时时间
        detection_timeout = 10
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if dt <= 0:
                dt = 0.05
            last_time = current_time
            
            if (current_time - start_time).to_sec() > detection_timeout:
                rospy.logwarn("⏰ 物体检测超时")
                break
            
            self.update_task_from_param()
            
            if self.reached:
                rate.sleep()
                continue

            twist = Twist()
            
            if self.target_dist is not None and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
                if self.just_detected:
                    if (self.search_direction > 0 and self.bearing_rad < -math.radians(15)) or \
                       (self.search_direction < 0 and self.bearing_rad > math.radians(15)):
                        transition_factor = min(1.0, (rospy.Time.now() - self.stable_track_time).to_sec() / 1.0)
                        smoothed_bearing = self.search_direction * math.radians(10) * (1 - transition_factor) + \
                                         self.bearing_rad * transition_factor
                    else:
                        smoothed_bearing = self.bearing_rad
                        self.just_detected = False
                else:
                    smoothed_bearing = self.bearing_rad
                
                self.angle_ = smoothed_bearing
                twist.angular.z = self.pid_angle.compute(self.angle_, dt)
                
                if abs(self.angle_) < math.radians(5):
                    twist.linear.y = self.pid_lateral.compute(self.offset_norm, dt)
                else:
                    twist.linear.y = 0.0
                    self.pid_lateral.reset()
                
                forward_speed = 0.0
                if abs(self.angle_) < math.radians(12) and abs(self.offset_norm) < 0.2:
                    dist_error = self.target_dist - self.target_distance
                    forward_speed = self.pid_forward.compute(dist_error, dt)
                    
                    if self.target_dist < 0.6: forward_speed *= 0.95
                    if self.target_dist < 0.4: forward_speed *= 0.85
                    
                    if abs(forward_speed) > 0 and abs(forward_speed) < 0.05:
                        forward_speed = 0.05 if forward_speed > 0 else -0.05
                else:
                    self.pid_forward.reset()
                    
                twist.linear.x = forward_speed

                angle_ok = abs(self.angle_) < math.radians(5)
                dist_ok = self.stop_min < self.target_dist < self.stop_max
                center_ok = abs(self.offset_norm) < 0.10
                
                if angle_ok and dist_ok and center_ok:
                    self.ok_counter += 1
                else:
                    self.ok_counter = 0

                if self.ok_counter >= 4:
                    self.reached = True
                    rospy.loginfo("✅ 连续 4 帧满足阈值，已安全到位并锁定！")
                    self.cmd_pub.publish(Twist())
                    rate.sleep()
                    break

                rospy.loginfo_throttle(1.0, "[PID控制] 角误差: {0:.1f}deg, 距离: {1:.2f}m, 偏移: {2:.3f}".format(
                    math.degrees(self.angle_), self.target_dist, self.offset_norm))

            else:
                search_speed = 0.35
                twist.angular.z = self.search_direction * search_speed
                twist.linear.x = 0.0
                
                self.pid_angle.reset()
                self.pid_lateral.reset()
                self.pid_forward.reset()
                self.ok_counter = 0
                
                if twist.angular.z != 0:
                    self.search_direction = twist.angular.z / abs(twist.angular.z)
                
                rospy.logwarn_throttle(2.0, "未找到目标，正在搜索... 方向: {0}".format('右转' if self.search_direction > 0 else '左转'))

            self.cmd_pub.publish(twist)
            rate.sleep()

        rospy.loginfo("物体检测循环已停止。")
        self.cmd_pub.publish(Twist())
        
        if self.reached and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
            return True, self.current_detected_object
        
        return False, None

    def reset_detector_state(self):
        self.reached = False
        self.ok_counter = 0
        self.just_detected = False
        self.current_detected_object = None
        self.last_seen_time = rospy.Time(0)
        self.stable_track_time = rospy.Time(0)
        
        self.pid_angle.reset()
        self.pid_lateral.reset()
        self.pid_forward.reset()
        
        self.bearing_rad = 0.0
        self.vision_bearing_rad = 0.0
        self.lidar_bearing_rad = None
        self.target_dist = None
        self.closest_dist = None
        self.offset_norm = 0.0
        self.angle_ = 0.0
        self.box_width = None
        self.target_dist_filtered = None
        self.vision_dist = None
        self.lidar_dist_raw = None
        
        rospy.loginfo("🔄 检测器状态已重置")


class Navigator(object):
    def __init__(self, csv_path, yaml_path):
        self.tf = tf.TransformListener()
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("正在等待 move_base action 服务器...")
        self.mb.wait_for_server()
        rospy.loginfo("✅ move_base action 服务器已连接。")

        self.cfg = self.load_yaml(yaml_path)
        self.wps = self.load_csv(csv_path)

        self.pid_x = PID(**self.cfg['pid_lin'])
        self.pid_y = PID(**self.cfg['pid_lin'])
        self.pid_yaw = PID(**self.cfg['pid_yaw'])
        
        self.qr_detector = QRCodeDetector()
        self.object_detector = ObjectDetector()
        self.traffic_light_detector = TrafficLightDetector()
        
        self.detected_objects = []
        self.usb_cam_process = None

    def load_yaml(self, path):
        defaults = {
            'pos_tol': 0.04,
            'yaw_tol_deg': 2.0,
            'max_nav_time': 300,
            'refine_timeout': 15.0,
            'pid_lin': {'kp': 1.2, 'ki': 0.0, 'kd': 0.05},
            'pid_yaw': {'kp': 1.8, 'ki': 0.0, 'kd': 0.1},
            'max_lin': 0.35, 'max_ang': 1.5,
            'min_lin': 0.10, 'min_ang': 0.25
        }
        try:
            cfg = yaml.safe_load(open(path)) or {}
        except Exception:
            rospy.logwarn('⚠️ 无法读取 %s，使用默认参数', path)
            cfg = {}
        for k, v in defaults.items():
            if k not in cfg:
                cfg[k] = v
        cfg['yaw_tol'] = math.radians(cfg['yaw_tol_deg'])
        return cfg

    @staticmethod
    def load_csv(path):
        wps = []
        for row in csv.reader(open(path)):
            if not row:
                continue
            if row[0].lower() == 'tag' or len(row) < 5:
                continue
            wps.append({
                'tag': row[0],
                'x': float(row[1]),
                'y': float(row[2]),
                'qz': float(row[3]),
                'qw': float(row[4])
            })
        rospy.loginfo('已加载 %d 个路径点', len(wps))
        return wps

    def run(self):
        target_detected = False
        traffic_light_detected = False
        skip_to_point = None
        
        for idx, wp in enumerate(self.wps, 1):
            if skip_to_point is not None:
                if idx < skip_to_point:
                    rospy.loginfo('⏭️ 跳过第%d个导航点', idx)
                    continue
                else:
                    rospy.loginfo('🎯 到达跳转目标点第%d个导航点', idx)
                    skip_to_point = None
            
            rospy.loginfo('→ 处理 %d/%d  (tag=%s)', idx, len(self.wps), wp['tag'])
            self.navigate(wp)
            
            if idx == 1:
                rospy.loginfo('📍 到达第一个导航点，开始二维码识别...')
                self.detect_qr_code_at_first_point()
            
            elif idx in [3, 4, 5] and not target_detected:
                rospy.loginfo('📍 到达第%d个导航点，开始物体识别...', idx)
                detected = self.detect_object_at_point(idx)
                
                if detected:
                    rospy.loginfo('🎯 在第%d个导航点检测到目标物体，将直接跳转到第6个导航点', idx)
                    target_detected = True
                    skip_to_point = 6
                    rospy.loginfo('⏭️ 设置跳转到第6个导航点')
                    self.print_task_summary()
                else:
                    rospy.loginfo('❌ 在第%d个导航点未检测到目标物体，继续前往下一个导航点', idx)


            elif idx == 6:
                rospy.loginfo('📍 到达第6个导航点，执行特殊摄像头操作...')
                rospy.loginfo('🛑 机器人已停止，将在此处静止10秒...')
                self.cmd_pub.publish(Twist())
                time.sleep(10)
                
                rospy.loginfo('⏰ 10秒静止时间到，开始启动 ucar_camera...')
                self.usb_cam_process = launch_ucar_camera(self.usb_cam_process)
                rospy.loginfo('✅ 特殊摄像头操作完成，继续导航...')

            elif idx in [7, 8] and not traffic_light_detected:
                rospy.loginfo('🚦 到达第%d个导航点，开始内置交通灯识别...', idx)
                green_detected = self.detect_traffic_light_at_point(idx)
                
                if green_detected:
                    rospy.loginfo('🟢 在第%d个导航点识别到绿灯', idx)
                    self.play_traffic_light_audio(idx)
                    
                    if idx == 7:
                        rospy.loginfo('⏭️ 第7个导航点识别到绿灯，将跳过后续导航点')
                        traffic_light_detected = True
                        skip_to_point = len(self.wps) + 1 # 跳转到最后一个点之后，确保结束循环
                        rospy.loginfo('⏭️ 设置跳转到任务结束')
                    else:
                        rospy.loginfo('✅ 第8个导航点识别到绿灯，继续正常导航')
                else:
                    rospy.loginfo('🔴 在第%d个导航点未识别到绿灯，前往下一个导航点', idx)

    def detect_qr_code_at_first_point(self):
        rospy.loginfo('🔍 开始二维码识别，请将二维码对准摄像头...')
        rate = rospy.Rate(10)
        timeout = 60
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn('⏰ 二维码识别超时')
                break
                
            detected, qr_data = self.qr_detector.detect_qr_code()
            if detected:
                rospy.set_param("/procurement_task", qr_data.lower())
                rospy.loginfo("已设置ROS参数: /procurement_task = %s" % qr_data.lower().encode('utf-8'))
                self.qr_detector.play_audio_for_qr(qr_data)
                rospy.loginfo('✅ 二维码识别完成，继续导航...')
                time.sleep(2)
                break
            rate.sleep()

    def detect_object_at_point(self, point_idx):
        rospy.loginfo('🔍 开始物体识别，请将目标物体对准摄像头...')
        detected, object_name = self.object_detector.run_object_detection()
        
        if detected and object_name:
            procurement_task = rospy.get_param('/procurement_task', '').strip().lower()
            target_classes = self.object_detector.category_items.get(procurement_task, [])
            
            if object_name in target_classes:
                if object_name not in self.detected_objects:
                    self.detected_objects.append(object_name)
                    rospy.loginfo('✅ 检测到目标物体: %s', object_name)
                    
                    # === 修改点: 播放语音前，确保车辆完全静止 ===
                    rospy.loginfo("🛑 播报前停止0.5秒...")
                    self.cmd_pub.publish(Twist()) # 发布零速指令
                    time.sleep(0.5) # 等待0.5秒

                    self.object_detector.play_audio_for_object(object_name)
                    rospy.set_param("/detected_objects", self.detected_objects)
                    rospy.loginfo('📝 已记录物体: %s', object_name)
                    rospy.loginfo('📋 当前检测到的物体列表: %s', self.detected_objects)
                    return True
                else:
                    rospy.loginfo('⚠️ 物体 %s 已经检测过了', object_name)
                    return False
            else:
                rospy.loginfo('⚠️ 检测到的物体 %s 不在目标类别中', object_name)
                return False
        else:
            rospy.loginfo('❌ 未检测到目标物体，继续前往下一个导航点')
            return False

    def detect_traffic_light_at_point(self, point_idx):
        rospy.loginfo('🛑 机器人已停止，将在此处静止10秒进行交通灯识别...')
        self.cmd_pub.publish(Twist())
        
        detection_time = 10.0
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        green_count = 0
        red_count = 0
        
        rospy.loginfo('⏰ 开始10秒交通灯检测，实时从图像处理结果中采样...')
        
        last_log_time = 0
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed_time >= detection_time:
                rospy.logwarn('⏰ 10秒交通灯识别时间到')
                break
            
            current_status = self.traffic_light_detector.get_status()
            
            if current_status == 'green':
                green_count += 1
            elif current_status == 'red':
                red_count += 1

            if int(elapsed_time) > last_log_time:
                last_log_time = int(elapsed_time)
                rospy.loginfo('📊 [%.1fs] 实时统计 - 绿灯:%d次, 红灯:%d次',
                              elapsed_time, green_count, red_count)
            
            if green_count >= 30:
                rospy.loginfo('✅ 绿灯信号稳定，提前确认！')
                break

            rate.sleep()
        
        rospy.loginfo('📋 10秒检测完成，最终统计: 绿灯 %d 次, 红灯 %d 次', green_count, red_count)
        
        if green_count > red_count and green_count >= 20:
            rospy.loginfo('✅ 根据统计结果，判定为绿灯！')
            return True
        else:
            rospy.loginfo('❌ 未检测到足够的绿灯信号，判定为红灯或无信号')
            return False

    def play_traffic_light_audio(self, point_idx):
        if point_idx in TRAFFIC_LIGHT_AUDIO_MAP:
            filename = TRAFFIC_LIGHT_AUDIO_MAP[point_idx]
            audio_path = os.path.join(TRAFFIC_LIGHT_AUDIO_DIR, filename)
            if os.path.exists(audio_path):
                try:
                    subprocess.Popen(['aplay', audio_path])
                    rospy.loginfo("🔊 正在播放交通灯语音: %s" % audio_path)
                except Exception as e:
                    rospy.logwarn("交通灯语音播放失败: %s" % str(e))
            else:
                rospy.logwarn("交通灯语音文件不存在: %s" % audio_path)
        else:
            rospy.logwarn("未找到第%d个导航点对应的交通灯语音文件" % point_idx)

    def print_task_summary(self):
        rospy.loginfo("=" * 50)
        rospy.loginfo("📋 任务摘要")
        rospy.loginfo("=" * 50)
        procurement_task = rospy.get_param('/procurement_task', '').strip().lower()
        rospy.loginfo("🎯 当前任务类别: %s", procurement_task)
        target_classes = self.object_detector.category_items.get(procurement_task, [])
        rospy.loginfo("📝 目标物体列表: %s", target_classes)
        detected_objects = rospy.get_param("/detected_objects", [])
        rospy.loginfo("✅ 已检测到的物体: %s", detected_objects)
        if target_classes:
            success_rate = len(detected_objects) / float(len(target_classes)) * 100
            rospy.loginfo("📊 检测成功率: %.1f%% (%d/%d)", success_rate, len(detected_objects), len(target_classes))
        else:
            rospy.loginfo("📊 检测成功率: 0%% (无目标物体)")
        rospy.loginfo("=" * 50)

    def navigate(self, wp):
        goal = self.build_goal(wp)
        self.mb.send_goal(goal)
        if not self.wait_for_mb():
            rospy.logerr('❌ 粗导航失败')
            return
        rospy.loginfo('✔ 粗导航完成')
        if wp['tag'].lower() == 'frame':
            self.refine(goal)

    def wait_for_mb(self):
        start = rospy.Time.now()
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.mb.get_state() == GoalStatus.SUCCEEDED:
                return True
            if (rospy.Time.now() - start).to_sec() > self.cfg['max_nav_time']:
                self.mb.cancel_goal()
                return False
            r.sleep()

    def refine(self, goal):
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        phase = 'trans'
        for pid in (self.pid_x, self.pid_y, self.pid_yaw):
            pid.reset()

        while (rospy.Time.now() - t0).to_sec() < self.cfg['refine_timeout']:
            dx_r, dy_r, yaw_err, dist = self.compute_errors(goal)
            if phase == 'trans':
                if dist <= self.cfg['pos_tol']:
                    phase = 'rot'
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    rospy.loginfo('✅ 平移完成 %.3fm → 开始旋转', dist)
                    continue
                cmd = self.cmd_trans(dx_r, dy_r)
            else:
                if abs(yaw_err) <= self.cfg['yaw_tol']:
                    rospy.loginfo('✅ 旋转完成 %.2f°', math.degrees(yaw_err))
                    break
                cmd = self.cmd_rot(yaw_err)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def compute_errors(self, goal):
        try:
            self.tf.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.2))
            (t, r) = self.tf.lookupTransform('map', 'base_link', rospy.Time(0))
        except Exception:
            return 0, 0, 0, 999

        dx = goal.target_pose.pose.position.x - t[0]
        dy = goal.target_pose.pose.position.y - t[1]
        dist = math.hypot(dx, dy)
        yaw_curr = q_to_yaw(Quaternion(*r))
        yaw_goal = q_to_yaw(goal.target_pose.pose.orientation)
        yaw_err = ang_diff(yaw_goal, yaw_curr)
        dx_r = math.cos(yaw_curr) * dx + math.sin(yaw_curr) * dy
        dy_r = -math.sin(yaw_curr) * dx + math.cos(yaw_curr) * dy
        msg = '误差: {:.3f}m  {:.2f}°  [{}]'.format(
            dist, math.degrees(yaw_err),
            'trans' if dist > self.cfg['pos_tol'] else 'rot')
        rospy.loginfo_throttle(1, msg)
        return dx_r, dy_r, yaw_err, dist

    def cmd_trans(self, dx_r, dy_r):
        dt = 1.0 / 20.0
        d = math.hypot(dx_r, dy_r)
        v = self.pid_x.step(d, dt)
        ux, uy = dx_r / d, dy_r / d if d > 1e-4 else (0, 0)
        cmd = Twist()
        if abs(v) > self.cfg['min_lin']:
            v = max(-self.cfg['max_lin'], min(self.cfg['max_lin'], v))
            cmd.linear.x = v * ux
            cmd.linear.y = v * uy
        return cmd

    def cmd_rot(self, yaw_err):
        dt = 1.0 / 20.0
        w = self.pid_yaw.step(yaw_err, dt)
        cmd = Twist()
        if abs(w) > self.cfg['min_ang']:
            cmd.angular.z = max(-self.cfg['max_ang'], min(self.cfg['max_ang'], w))
        return cmd

    @staticmethod
    def build_goal(wp):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = wp['x']
        g.target_pose.pose.position.y = wp['y']
        g.target_pose.pose.orientation.z = wp['qz']
        g.target_pose.pose.orientation.w = wp['qw']
        return g


# ---------------- main ---------------- #
if __name__ == '__main__':
    rospy.init_node('navigator_v156', anonymous=False)
    root = os.path.dirname(os.path.abspath(__file__))
    usb_cam_process = None
    
    try:
        # launch_navigation()
        
        if not check_camera_status():
            rospy.logwarn("⚠️ 摄像头状态检查失败，但仍将尝试启动。")
        
        # 初始启动默认摄像头
        usb_cam_process = ensure_camera_is_running(None)
        if usb_cam_process is None:
            rospy.logwarn("⚠️ 默认摄像头启动失败，相关功能可能无法使用。")

        rospy.loginfo("系统初始化完成，即将开始执行导航任务...")
        nav = Navigator(os.path.join(root, 'points.csv'),
                        os.path.join(root, 'tuning.yaml'))
        
        nav.usb_cam_process = usb_cam_process
        nav.run()
        
        usb_cam_process = nav.usb_cam_process
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")
    except Exception as e:
        rospy.logerr("发生未知错误: %s", str(e))
        import traceback
        traceback.print_exc()
    finally:
        stop_process(usb_cam_process, "最后的摄像头进程")
        rospy.loginfo("所有任务完成，程序退出。")