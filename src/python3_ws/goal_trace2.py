#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
目标跟踪节点（Vision + LiDAR）
---------------------------------
功能：
1. 订阅 darknet_ros 的 /darknet_ros/bounding_boxes，筛选目标类别物体；
2. 利用图像中心偏移估算方位角，结合 /scan 读取该方位距离；
3. 通过 /cmd_vel 直接控制底盘转向和前进，直至与目标相距 0.1~0.15 m 并正对；
4. 若失去目标，则原地缓慢旋转搜索。

运行环境：Ubuntu 18.04 + ROS Melodic (Python2.7) + JetPack 4.2.2
"""

import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, CameraInfo
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
# 如需使用 TF 做精确相机→雷达坐标转换，可取消下两行注释
# import tf2_ros
# import tf_transformations

# ---------- 目标类别与具体物品映射 ----------
CATEGORY_ITEMS = {
    "fruit":     ["apple", "banana", "watermelon"],  # 水果类
    "vegetable": ["chilli", "tomato", "potato"],      # 蔬菜类
    "dessert":   ["milk", "cake", "cola"],           # 甜品类
}

# ---------- PID控制器类 ----------
class PIDController(object):
    """完整的PID控制器实现"""
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, output_min=None, output_max=None, integral_max=None):
        # PID增益
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # 控制输出限幅
        self.output_min = output_min
        self.output_max = output_max
        # 积分限幅，防止积分饱和
        self.integral_max = integral_max if integral_max is not None else 10.0
        # 积分和前一误差初始化
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """重置积分和前一误差"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, dt=None):
        """
        计算PID控制输出
        Args:
            error: 当前误差
            dt: 时间间隔（如果为None则使用固定值）
        Returns:
            控制输出
        """
        if dt is None or dt <= 0:
            dt = 0.1  # 默认10Hz
        
        # P项
        P_out = self.Kp * error
        
        # I项：误差积分（添加积分限幅）
        self.integral += error * dt
        # 限制积分项，防止积分饱和
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        I_out = self.Ki * self.integral
        
        # D项：误差微分
        D_out = 0.0
        if dt > 1e-6:
            D_out = self.Kd * ((error - self.prev_error) / dt)
        
        # 保存当前误差用于下一次微分计算
        self.prev_error = error
        
        # PID总输出
        output = P_out + I_out + D_out
        
        # 控制输出限幅（防止超出最大速度）
        if self.output_max is not None and output > self.output_max:
            output = self.output_max
            # 防止积分项持续增大导致超调，达到限幅时适当削减积分
            if self.integral != 0:
                self.integral = self.integral * 0.95
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
            if self.integral != 0:
                self.integral = self.integral * 0.95
                
        return output

class TargetFollower(object):
    """核心类：封装订阅、逻辑、发布等功能"""
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('goal_trace', anonymous=False)

        # -------- 相机与算法参数 --------
        self.img_w = 480                    # YOLO 输入图像宽（像素）
        self.img_h = 320                    # YOLO 输入图像高（像素）
        self.cam_hfov = math.radians(124.8) # 相机水平视场角 (弧度) - 默认值
        
        # -------- 相机标定参数（从camera_info获取）--------
        self.camera_matrix = None           # 相机内参矩阵
        self.focal_x = None                 # X方向焦距
        self.focal_y = None                 # Y方向焦距  
        self.center_x = None                # 光学中心X坐标
        self.center_y = None                # 光学中心Y坐标
        self.camera_info_received = False   # 是否已接收到相机信息

        # -------- 目标类别相关 --------
        self.target_category = ""   # 当前任务大类（fruit / vegetable / dessert）
        self.target_classes = []    # 该大类对应 YOLO 具体类别列表
        self.update_task_from_param(force=True)  # 初始化一次

        # -------- 目标实时信息 --------
        self.last_seen_time = rospy.Time(0) # 最近一次看到目标的时间
        self.bearing_rad = 0.0             # 融合后的朝向误差 (弧度，右正)
        self.vision_bearing_rad = 0.0      # 视觉计算的角度
        self.lidar_bearing_rad = None      # 激光雷达计算的角度
        self.target_dist = None            # 由雷达测得的距离 (米)
        self.closest_dist = None           # 激光雷达检测到的最近距离
        self.offset_norm = 0.0             # 图像中心偏移归一化值 (-1~1)

        # -------- 控制参数（可调）--------
        # PID控制器参数
        self.angle_Kp = rospy.get_param('~angle_Kp', 0.9)
        self.angle_Ki = rospy.get_param('~angle_Ki', 0.08)
        self.angle_Kd = rospy.get_param('~angle_Kd', 0.25)
        self.lateral_Kp = rospy.get_param('~lateral_Kp', 0.5)
        self.lateral_Ki = rospy.get_param('~lateral_Ki', 0.0)
        self.lateral_Kd = rospy.get_param('~lateral_Kd', 0.08)
        self.forward_Kp = rospy.get_param('~forward_Kp', 0.8)
        self.forward_Ki = rospy.get_param('~forward_Ki', 0.05)
        self.forward_Kd = rospy.get_param('~forward_Kd', 0.22)
        
        self.target_width_m = rospy.get_param('~target_width', 0.05)
        self.target_height_m = rospy.get_param('~target_height', 0.08)
        
        self.k_lat = 0.6
        self.speed_gain_ = 0.6
        
        # -------- 运动限制 --------
        self.max_lat = 0.18
        self.max_lin = 0.25
        self.max_ang = 0.6
        self.stop_min = 0.35
        self.stop_max = 0.45
        self.target_distance = (self.stop_min + self.stop_max) / 2.0

        # -------- 距离估算相关 --------
        self.target_dist_filtered = None
        self.vision_dist = None
        self.lidar_dist_raw = None
        self.dist_filter_alpha = 0.3
        self.dist_samples = []
        self.max_samples = 5

        # -------- 运动状态相关变量 --------
        self.angle_last_ = 0.0
        self.offset_last = 0.0
        self.steering_cumulative = 0.0
        self.offset_cumulative = 0.0
        self.w_threshold = 155
        self.angle_ = 0.0
        self.box_width = None

        # -------- PID控制器实例 --------
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

        # -------- ROS 通讯 --------
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_cb)
        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_cb)

        rospy.loginfo("目标跟踪节点已启动，等待检测并移动...")

        self.reached = False
        self.ok_counter = 0
        
        # -------- 状态管理 --------
        self.search_direction = 1.0
        self.just_detected = False
        self.stable_track_time = rospy.Time(0)

    def update_task_from_param(self, force=False):
        category = rospy.get_param('/procurement_task', '').strip().lower()
        if category != self.target_category or force:
            self.target_category = category
            self.target_classes = CATEGORY_ITEMS.get(category, [])
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
            rospy.loginfo("📷 相机信息已接收:")
            rospy.loginfo("  图像尺寸: {0}x{1}".format(self.img_w, self.img_h))
            rospy.loginfo("  焦距: fx={0:.1f}, fy={1:.1f}".format(self.focal_x, self.focal_y))
            rospy.loginfo("  光学中心: cx={0:.1f}, cy={1:.1f}".format(self.center_x, self.center_y))
            rospy.loginfo("  计算得出水平视场角: {0:.1f}°".format(math.degrees(self.cam_hfov)))

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
            rospy.loginfo_throttle(3.0, "激光雷达角度差异过大: {0:.1f}°, 使用视觉角度".format(math.degrees(angle_diff)))
            return vision_angle
        
        if time_since_detection < 2.0:
            weight_lidar = 0.3
            weight_vision = 0.7
            rospy.loginfo_throttle(1.0, "刚检测到目标，主要使用视觉角度")
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
            # --- 修正点 1: 替换 math.isfinite ---
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
        for box in msg.bounding_boxes:
            cls_name = box.Class.lower()
            if cls_name in self.target_classes and box.probability > best_prob:
                best_prob = box.probability
                best_box = box

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
            
            cam_info_status = "✅使用相机内参" if self.camera_info_received else "⚠️使用默认参数"
            rospy.loginfo_throttle(3.0, "检测到 [{0}] 置信度 {1:.2f} ({2})".format(best_box.Class, best_prob, cam_info_status))
            rospy.loginfo_throttle(3.0, "图像偏移: {0:.3f}, 视觉角度: {1:.1f}°".format(self.offset_norm, math.degrees(self.vision_bearing_rad)))
            if self.lidar_bearing_rad is not None:
                angle_diff = abs(self.vision_bearing_rad - self.lidar_bearing_rad)
                rospy.loginfo_throttle(3.0, "激光角度: {0:.1f}°, 差异: {1:.1f}°, 融合角度: {2:.1f}°".format(
                    math.degrees(self.lidar_bearing_rad), math.degrees(angle_diff), math.degrees(self.bearing_rad)))
            else:
                rospy.loginfo_throttle(3.0, "激光角度: N/A, 融合角度: {0:.1f}°".format(math.degrees(self.bearing_rad)))

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
            # --- 修正点 2: 替换 math.isfinite ---
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
                rospy.logwarn_throttle(3.0, "视觉距离差异过大: LiDAR={0:.2f}m vs Vision={1:.2f}m".format(self.lidar_dist_raw, self.vision_dist))
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
            
        if self.closest_dist is not None and self.target_dist is not None:
            dist_diff = abs(self.closest_dist - self.target_dist)
            if dist_diff > 0.3:
                rospy.logwarn("距离不一致: 最近点{0:.2f}m vs 目标点{1:.2f}m".format(self.closest_dist, self.target_dist))

       # ===============================================================
    # 主控制循环 (已修正)
    # ===============================================================
    def run(self):
        rate = rospy.Rate(20)  # 20 Hz 发送控制
        last_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if dt <= 0:
                dt = 0.05  # 对应20Hz
            last_time = current_time
            
            self.update_task_from_param()  # 每循环检查一次参数是否改变
            
            # --- 关键修改：如果已经到达，则直接跳过后续所有逻辑 ---
            # 这个标志位确保一旦break，即使有后续的循环也不会再执行
            if self.reached:
                rate.sleep()
                continue

            twist = Twist()
            
            # 若距离合法，进入跟踪 / 停止逻辑
            if self.target_dist is not None and (rospy.Time.now() - self.last_seen_time).to_sec() < 1.0:
                # 刚检测到目标时的平滑过渡
                if self.just_detected:
                    if (self.search_direction > 0 and self.bearing_rad < -math.radians(15)) or \
                       (self.search_direction < 0 and self.bearing_rad > math.radians(15)):
                        transition_factor = min(1.0, (rospy.Time.now() - self.stable_track_time).to_sec() / 1.0)
                        smoothed_bearing = self.search_direction * math.radians(10) * (1 - transition_factor) + \
                                         self.bearing_rad * transition_factor
                        rospy.loginfo("平滑转换中... 因子: {0:.2f}".format(transition_factor))
                    else:
                        smoothed_bearing = self.bearing_rad
                        self.just_detected = False
                else:
                    smoothed_bearing = self.bearing_rad
                
                # 1. PID控制角速度
                self.angle_ = smoothed_bearing
                twist.angular.z = self.pid_angle.compute(self.angle_, dt)
                
                # 2. PID控制侧向速度
                if abs(self.angle_) < math.radians(5):
                    twist.linear.y = self.pid_lateral.compute(self.offset_norm, dt)
                else:
                    twist.linear.y = 0.0
                    self.pid_lateral.reset()
                
                # 3. PID控制前进速度
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

                # 判定是否到达
                angle_ok = abs(self.angle_) < math.radians(5)
                dist_ok = self.stop_min < self.target_dist < self.stop_max
                center_ok = abs(self.offset_norm) < 0.10
                
                if angle_ok and dist_ok and center_ok:
                    self.ok_counter += 1
                else:
                    self.ok_counter = 0

                # --- 关键修改：使用 break 彻底退出循环 ---
                if self.ok_counter >= 4:
                    self.reached = True # 设置标志位
                    rospy.loginfo("✅ 连续 4 帧满足阈值，已安全到位并锁定！")
                    rospy.loginfo("正在停止运动控制...")
                    self.cmd_pub.publish(Twist()) # 发布一次零速度指令
                    rate.sleep() # 等待一小会儿确保指令发出
                    break # 彻底退出 while 循环

                rospy.loginfo_throttle(1.0, "[PID控制] 角误差: {0:.1f}deg, 距离: {1:.2f}m, 偏移: {2:.3f}".format(
                    math.degrees(self.angle_), self.target_dist, self.offset_norm))

            else:
                # 未检测到目标 / 距离无效 —— 原地旋转搜索
                search_speed = 0.35
                twist.angular.z = self.search_direction * search_speed
                twist.linear.x = 0.0
                
                self.pid_angle.reset()
                self.pid_lateral.reset()
                self.pid_forward.reset()
                self.ok_counter = 0 # 丢失目标时重置计数器
                
                if twist.angular.z != 0:
                    self.search_direction = twist.angular.z / abs(twist.angular.z)
                
                rospy.logwarn_throttle(2.0, "未找到目标，正在搜索... 方向: {0}".format('右转' if self.search_direction > 0 else '左转'))

            # 发布最终的速度指令
            self.cmd_pub.publish(twist)
            rate.sleep()

        # 当循环结束后（因为break），再发布一次停止指令，确保万无一失
        rospy.loginfo("控制循环已停止。机器人将保持静止。")
        self.cmd_pub.publish(Twist())

if __name__ == '__main__':
    try:
        node = TargetFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass