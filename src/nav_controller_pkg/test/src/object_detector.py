#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import division
import math
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
from utils import PIDController # 导入PIDController

class ObjectDetector(object):
    """
    物体检测器，用于识别目标物体并进行精细调整。
    此模块仅负责检测和姿态控制，不负责音频播放。
    """
    def __init__(self, obj_config):
        # 物体类别映射
        self.category_items = {
            "fruit":     ["apple", "banana", "watermelon"],
            "vegetable": ["chilli", "tomato", "potato"],
            "dessert":   ["milk", "cake", "cola"],
        }
        
        # 摄像头参数
        self.img_w = obj_config.get('image_width', 480)
        self.img_h = obj_config.get('image_height', 320)
        self.cam_hfov = math.radians(obj_config.get('camera_hfov', 124.8))
        self.camera_matrix = None
        self.focal_x = None
        self.focal_y = None
        self.center_x = None
        self.center_y = None
        self.camera_info_received = False
        
        # 目标与状态
        self.target_category = ""
        self.target_classes = []
        self.last_seen_time = rospy.Time(0)
        self.just_detected = False
        self.stable_track_time = rospy.Time(0)
        self.reached = False
        self.ok_counter = 0
        self.current_detected_object = None

        # 控制参数
        self.angle_Kp = obj_config.get('angle_Kp', 0.9)
        self.angle_Ki = obj_config.get('angle_Ki', 0.08)
        self.angle_Kd = obj_config.get('angle_Kd', 0.25)
        self.lateral_Kp = obj_config.get('lateral_Kp', 0.5)
        self.lateral_Ki = obj_config.get('lateral_Ki', 0.0)
        self.lateral_Kd = obj_config.get('lateral_Kd', 0.08)
        self.forward_Kp = obj_config.get('forward_Kp', 0.8)
        self.forward_Ki = obj_config.get('forward_Ki', 0.05)
        self.forward_Kd = obj_config.get('forward_Kd', 0.22)
        
        self.max_lat = obj_config.get('max_lateral_speed', 0.18)
        self.max_lin = obj_config.get('max_linear_speed', 0.25)
        self.max_ang = obj_config.get('max_angular_speed', 0.6)
        self.stop_min = obj_config.get('stop_distance_min', 0.35)
        self.stop_max = obj_config.get('stop_distance_max', 0.45)
        self.target_distance = (self.stop_min + self.stop_max) / 2.0
        
        self.target_width_m = obj_config.get('target_width_meters', 0.05)
        self.dist_filter_alpha = obj_config.get('distance_filter_alpha', 0.3)

        # 实时测量值
        self.bearing_rad = 0.0
        self.vision_bearing_rad = 0.0
        self.lidar_bearing_rad = None
        self.target_dist = None
        self.closest_dist = None # From find_closest_object_angle
        self.offset_norm = 0.0
        self.angle_ = 0.0 # Current angular error to target
        self.box_width = None # Object bounding box width in pixels
        self.target_dist_filtered = None
        self.vision_dist = None
        self.lidar_dist_raw = None
        
        # PID控制器实例
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
        
        self.search_direction = 1.0 # 初始搜索方向 (1.0 for right, -1.0 for left)
        
        # ROS 发布订阅
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_cb)
        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        # 摄像头信息可能由 usb_cam 或 ucar_camera 发布，这里暂时订阅 usb_cam
        # 实际使用时，需要确保在启动 ucar_camera 后切换订阅话题或使用 /camera_info
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_cb) 
        
        self.update_task_from_param(force=True)

    def update_task_from_param(self, force=False):
        """从ROS参数服务器更新目标任务类别"""
        category = rospy.get_param('/procurement_task', '').strip().lower()
        if category != self.target_category or force:
            self.target_category = category
            self.target_classes = self.category_items.get(category, [])
            rospy.loginfo("[任务更新] 当前目标大类：{0} → 具体物品列表：{1}".format(category, self.target_classes))

    def camera_info_cb(self, msg):
        """摄像头信息回调，获取内参"""
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
        """计算目标相对于相机中心的精确偏航角 (弧度)"""
        if self.camera_info_received and self.focal_x is not None:
            pixel_offset = center_x - self.center_x
            bearing_rad = math.atan(pixel_offset / self.focal_x)
            return bearing_rad
        else:
            # Fallback if camera info not available (less accurate)
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            return raw_offset * (self.cam_hfov / 2.0)

    def calculate_vision_distance(self):
        """根据目标宽度和相机焦距计算视觉距离"""
        if not self.camera_info_received or self.box_width is None:
            return None
        
        if self.focal_x is None or self.target_width_m <= 0:
            return None
            
        if self.box_width > 10: # Avoid division by small numbers
            vision_dist = (self.focal_x * self.target_width_m) / self.box_width
            if 0.1 < vision_dist < 5.0: # Filter out unreasonable distances
                return vision_dist
        return None

    def fuse_angles(self, vision_angle, lidar_angle):
        """融合视觉和激光雷达的角度信息"""
        if lidar_angle is None:
            return vision_angle
        
        time_since_detection = (rospy.Time.now() - self.stable_track_time).to_sec()
        angle_diff_abs = abs(vision_angle - lidar_angle)
        
        # If angles differ significantly, trust vision (might be a false LIDAR detection)
        if angle_diff_abs > math.radians(30):
            return vision_angle
        
        # Dynamic weighting based on tracking stability and angle difference
        if time_since_detection < 2.0: # Just detected, favor vision more
            weight_lidar = 0.3
            weight_vision = 0.7
        elif angle_diff_abs < math.radians(10): # Angles are close, favor LIDAR more for stability
            weight_lidar = 0.7
            weight_vision = 0.3
        else: # Default balanced weighting
            weight_lidar = 0.5
            weight_vision = 0.5
            
        fused_angle = weight_lidar * lidar_angle + weight_vision * vision_angle
        return fused_angle

    def find_closest_object_angle(self, scan):
        """在激光雷达扫描中寻找最近物体所在的中心角度"""
        min_dist = float('inf')
        closest_angle = None
        search_range = math.radians(45) # Search within +/- 45 degrees
        cluster_angles = []
        cluster_distances = []
        
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > search_range:
                continue
            
            dist = scan.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and 0.1 < dist < 3.0: # Filter valid distances
                cluster_angles.append(angle)
                cluster_distances.append(dist)
                
                if dist < min_dist:
                    min_dist = dist
                    closest_angle = angle
        
        if closest_angle is not None and min_dist < 2.0: # Only consider if a close object is found
            nearby_angles = []
            nearby_weights = []
            
            # Find points in a small cluster around the closest point
            for angle, dist in zip(cluster_angles, cluster_distances):
                if abs(angle - closest_angle) < math.radians(5) and abs(dist - min_dist) < 0.2: # Small angle and distance diff
                    nearby_angles.append(angle)
                    weight = 1.0 / (dist + 0.1) # Weight by inverse distance
                    nearby_weights.append(weight)
            
            if nearby_angles:
                total_weight = sum(nearby_weights)
                if total_weight > 0:
                    weighted_angle = sum(a * w for a, w in zip(nearby_angles, nearby_weights)) / total_weight
                    self.closest_dist = min_dist # Store the closest distance from LIDAR
                    return weighted_angle
        
        return None

    def yolo_cb(self, msg):
        """YOLOv4/Darknet ROS 边界框回调函数"""
        if not self.target_classes: # No target classes set, skip detection
            return

        best_box = None
        best_prob = 0.0
        detected_object = None
        
        # Iterate through detected boxes to find the best match for the target category
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
            
            # Normalize offset for lateral control
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            self.offset_norm = 0.7 * self.offset_norm + 0.3 * raw_offset # Simple EMA filter
            self.vision_bearing_rad = 0.7 * self.vision_bearing_rad + 0.3 * raw_bearing # Simple EMA filter
            
            # Detect first time or re-detection after a break
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 0.5:
                self.just_detected = True
                self.stable_track_time = rospy.Time.now()
                rospy.loginfo("🎯 重新检测到目标，开始稳定跟踪")
            
            self.last_seen_time = rospy.Time.now()
            self.bearing_rad = self.fuse_angles(self.vision_bearing_rad, self.lidar_bearing_rad)
            
            self.current_detected_object = detected_object # Store the actual detected object name
            
            rospy.loginfo_throttle(3.0, "检测到 [{0}] 置信度 {1:.2f}".format(best_box.Class.encode('utf-8', errors='ignore'), best_prob))
        else:
            # If no object is detected, reset detection flag
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.5:
                self.current_detected_object = None

    def lidar_cb(self, scan):
        """激光雷达扫描回调函数"""
        # If no object is currently tracked by vision, clear LIDAR target
        if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
            self.target_dist = None
            self.lidar_bearing_rad = None
            return

        self.lidar_bearing_rad = self.find_closest_object_angle(scan)
        
        # Use vision bearing as a hint for LIDAR distance if LIDAR angle is not found or unreliable
        target_angle = self.vision_bearing_rad
        if self.lidar_bearing_rad is not None:
            target_angle = self.lidar_bearing_rad
            
        # Get LIDAR distance at the target angle
        ang = max(scan.angle_min, min(scan.angle_max, target_angle))
        index = int((ang - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        
        valid_distances = []
        search_radius = 3 # Look at a small window around the target angle
        
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
            
        # Fuse vision and LIDAR distances
        self.vision_dist = self.calculate_vision_distance()
        fused_dist = None
        
        if self.lidar_dist_raw is not None and self.vision_dist is not None:
            dist_diff = abs(self.lidar_dist_raw - self.vision_dist)
            if dist_diff < 0.5: # If they are relatively close, fuse them
                fused_dist = 0.7 * self.lidar_dist_raw + 0.3 * self.vision_dist
            else: # If they differ significantly, prefer LIDAR
                fused_dist = self.lidar_dist_raw
        elif self.lidar_dist_raw is not None:
            fused_dist = self.lidar_dist_raw
        elif self.vision_dist is not None:
            fused_dist = self.vision_dist
        
        # Apply EMA filter to the fused distance
        if fused_dist is not None:
            if self.target_dist_filtered is None:
                self.target_dist_filtered = fused_dist
            else:
                self.target_dist_filtered = (1 - self.dist_filter_alpha) * self.target_dist_filtered + \
                                          self.dist_filter_alpha * fused_dist
            self.target_dist = self.target_dist_filtered
        else:
            self.target_dist = None

    def run_object_detection(self):
        """
        运行物体检测和微调循环。
        此方法不再播放音频，而是返回检测结果。
        """
        rospy.loginfo("🔍 开始物体检测和微调...")
        self.reset_detector_state()
        
        rate = rospy.Rate(20)
        last_time = rospy.Time.now()
        detection_timeout = 10.0 # Timeout for the whole detection process
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if dt <= 0: # Avoid zero or negative dt
                dt = 0.05
            last_time = current_time
            
            if (current_time - start_time).to_sec() > detection_timeout:
                rospy.logwarn("⏰ 物体检测超时")
                self.cmd_pub.publish(Twist())
                break
            
            self.update_task_from_param() # Periodically update target from ROS param
            
            if self.reached: # If target reached, stop and exit loop
                self.cmd_pub.publish(Twist()) # Ensure robot is stopped
                rate.sleep()
                break

            twist = Twist()
            
            # Only control if object is recently seen
            if self.target_dist is not None and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
                # Smooth angle on initial detection to avoid sharp turns
                if self.just_detected:
                    if (self.search_direction > 0 and self.bearing_rad < -math.radians(15)) or \
                       (self.search_direction < 0 and self.bearing_rad > math.radians(15)):
                        # If current angle is opposite to search direction, gradually transition
                        transition_factor = min(1.0, (rospy.Time.now() - self.stable_track_time).to_sec() / 1.0)
                        smoothed_bearing = self.search_direction * math.radians(10) * (1 - transition_factor) + \
                                         self.bearing_rad * transition_factor
                    else:
                        smoothed_bearing = self.bearing_rad
                        self.just_detected = False # Once smoothly aligned, disable just_detected
                else:
                    smoothed_bearing = self.bearing_rad
                
                self.angle_ = smoothed_bearing
                twist.angular.z = self.pid_angle.compute(self.angle_, dt)
                
                # Lateral control (linear.y) only when angle is sufficiently aligned
                if abs(self.angle_) < math.radians(5):
                    twist.linear.y = self.pid_lateral.compute(self.offset_norm, dt)
                else:
                    twist.linear.y = 0.0
                    self.pid_lateral.reset() # Reset integral when not active
                
                forward_speed = 0.0
                # Forward motion only when aligned and centered
                if abs(self.angle_) < math.radians(12) and abs(self.offset_norm) < 0.2:
                    dist_error = self.target_dist - self.target_distance
                    forward_speed = self.pid_forward.compute(dist_error, dt)
                    
                    # Reduce speed as it gets closer
                    if self.target_dist < 0.6: forward_speed *= 0.95
                    if self.target_dist < 0.4: forward_speed *= 0.85
                    
                    # Ensure minimum movement if PID output is too small but non-zero
                    if abs(forward_speed) > 0 and abs(forward_speed) < 0.05:
                        forward_speed = 0.05 if forward_speed > 0 else -0.05
                else:
                    self.pid_forward.reset() # Reset integral when not active
                    
                twist.linear.x = forward_speed

                # Check if all conditions for "reached" are met
                angle_ok = abs(self.angle_) < math.radians(5)
                dist_ok = self.stop_min < self.target_dist < self.stop_max
                center_ok = abs(self.offset_norm) < 0.10
                
                if angle_ok and dist_ok and center_ok:
                    self.ok_counter += 1
                else:
                    self.ok_counter = 0

                if self.ok_counter >= 4: # Require consecutive "OK" frames
                    self.reached = True
                    rospy.loginfo("✅ 连续 4 帧满足阈值，已安全到位并锁定！")
                    self.cmd_pub.publish(Twist()) # Final stop command
                    rate.sleep()
                    break

                rospy.loginfo_throttle(1.0, "[PID控制] 角误差: {0:.1f}deg, 距离: {1:.2f}m, 偏移: {2:.3f}".format(
                    math.degrees(self.angle_), self.target_dist, self.offset_norm))

            else: # Object not seen, enter search mode
                search_speed = 0.35
                twist.angular.z = self.search_direction * search_speed
                twist.linear.x = 0.0 # No forward movement during search
                
                # Reset PIDs during search
                self.pid_angle.reset()
                self.pid_lateral.reset()
                self.pid_forward.reset()
                self.ok_counter = 0
                
                # Update search direction
                if twist.angular.z != 0:
                    self.search_direction = twist.angular.z / abs(twist.angular.z)
                
                rospy.logwarn_throttle(2.0, "未找到目标，正在搜索... 方向: {0}".format('右转' if self.search_direction > 0 else '左转'))

            self.cmd_pub.publish(twist)
            rate.sleep()

        rospy.loginfo("物体检测循环已停止。")
        self.cmd_pub.publish(Twist()) # Ensure robot is stopped on exit
        
        if self.reached and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
            return True, self.current_detected_object
        
        return False, None

    def reset_detector_state(self):
        """重置检测器内部状态，准备下一次检测"""
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