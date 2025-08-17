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

运行环境：Ubuntu 18.04 + ROS Melodic (Python3.6.9) + JetPack 4.2.2
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
class PIDController:
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

class TargetFollower:
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
        # 角度控制PID参数 (进一步提高以增强响应速度)
        self.angle_Kp = rospy.get_param('~angle_Kp', 0.9)  # 提高从0.7到0.9
        self.angle_Ki = rospy.get_param('~angle_Ki', 0.08) # 提高从0.06到0.08
        self.angle_Kd = rospy.get_param('~angle_Kd', 0.25) # 提高从0.22到0.25
        # 侧向控制PID参数  
        self.lateral_Kp = rospy.get_param('~lateral_Kp', 0.5)  # 降低从0.7到0.5，减少横向震荡
        self.lateral_Ki = rospy.get_param('~lateral_Ki', 0.0)
        self.lateral_Kd = rospy.get_param('~lateral_Kd', 0.08) # 降低从0.12到0.08，平稳横向控制
        # 前进控制PID参数 (进一步提高增益增强动力响应)
        self.forward_Kp = rospy.get_param('~forward_Kp', 0.8)  # 提高从0.6到0.8
        self.forward_Ki = rospy.get_param('~forward_Ki', 0.05) # 提高从0.03到0.05
        self.forward_Kd = rospy.get_param('~forward_Kd', 0.22) # 提高从0.18到0.22
        
        # 目标物理尺寸（用于视觉测距，米）
        self.target_width_m = rospy.get_param('~target_width', 0.05)   # 默认5cm宽
        self.target_height_m = rospy.get_param('~target_height', 0.08) # 默认8cm高
        
        # 原有控制增益（作为备份）
        self.k_lat = 0.6   # 侧向速度比例增益
        self.speed_gain_ = 0.6  # 速度增益系数
        
        # -------- 运动限制 --------
        self.max_lat = 0.18  # 最大侧向速度 (降低从0.30到0.18，提高精度)
        self.max_lin = 0.25  # 最大前进速度 (提高从0.15到0.25)
        self.max_ang = 0.6   # 最大角速度 (提高从0.4到0.6)
        # 最近停止距离（米），用于 LiDAR 测距停止区域
        self.stop_min = 0.35  # 最近停止距离
        self.stop_max = 0.45  # 最远停止距离
        self.target_distance = (self.stop_min + self.stop_max) / 2.0  # 目标距离

        # -------- 距离估算相关 --------
        self.target_dist_filtered = None  # 滤波后的目标距离
        self.vision_dist = None           # 视觉估算距离
        self.lidar_dist_raw = None        # 激光雷达原始距离
        self.dist_filter_alpha = 0.3     # 距离滤波系数
        self.dist_samples = []            # 距离样本缓存
        self.max_samples = 5              # 最大样本数

        # -------- 运动状态相关变量 --------
        # 最近一次角度误差与偏移量，用于微分项
        self.angle_last_ = 0.0
        self.offset_last = 0.0
        # 累积误差，可用于积分控制（目前未启用，可扩展）
        self.steering_cumulative = 0.0
        self.offset_cumulative = 0.0
        # 目标框宽度阈值，用于纯视觉距离判定（像素）
        self.w_threshold = 155
        # 当前角度误差（弧度）
        self.angle_ = 0.0
        # 当前检测框宽度
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

        self.reached = False       # 是否已到达目标
        self.ok_counter = 0        # 连续满足计数
        
        # -------- 状态管理 --------
        self.search_direction = 1.0    # 搜索方向 (1.0=右转, -1.0=左转)
        self.just_detected = False     # 是否刚检测到目标
        self.stable_track_time = rospy.Time(0)  # 稳定跟踪开始时间

    # ===============================================================
    # 参数服务器轮询：定期读取 /procurement_task
    # ===============================================================
    def update_task_from_param(self, force=False):
        """
        读取 /procurement_task，更新 self.target_category 与 self.target_classes
        比赛只给‘大类’，因此直接在 CATEGORY_ITEMS 字典里取对应子列表
        """
        category = rospy.get_param('/procurement_task', '').strip().lower()
        if category != self.target_category or force:
            self.target_category = category
            self.target_classes = CATEGORY_ITEMS.get(category, [])  # 若未找到则置空
            rospy.loginfo(f"[任务更新] 当前目标大类：{category} → 具体物品列表：{self.target_classes}")

    # ===============================================================
    # 相机信息回调：获取相机内参，计算更准确的角度
    # ===============================================================
    def camera_info_cb(self, msg: CameraInfo):
        """
        处理相机信息，获取准确的相机内参
        """
        if not self.camera_info_received:
            # 保存相机内参
            self.camera_matrix = msg.K  # 3x3相机内参矩阵（展开为9个元素的列表）
            self.focal_x = msg.K[0]     # fx: 水平方向焦距
            self.focal_y = msg.K[4]     # fy: 垂直方向焦距  
            self.center_x = msg.K[2]    # cx: 光学中心X坐标
            self.center_y = msg.K[5]    # cy: 光学中心Y坐标
            
            # 更新图像尺寸（以相机信息为准）
            self.img_w = msg.width
            self.img_h = msg.height
            
            # 根据焦距计算更准确的水平视场角
            # HFOV = 2 * arctan(width / (2 * fx))
            self.cam_hfov = 2.0 * math.atan(self.img_w / (2.0 * self.focal_x))
            
            self.camera_info_received = True
            rospy.loginfo("📷 相机信息已接收:")
            rospy.loginfo(f"  图像尺寸: {self.img_w}x{self.img_h}")
            rospy.loginfo(f"  焦距: fx={self.focal_x:.1f}, fy={self.focal_y:.1f}")
            rospy.loginfo(f"  光学中心: cx={self.center_x:.1f}, cy={self.center_y:.1f}")
            rospy.loginfo(f"  计算得出水平视场角: {math.degrees(self.cam_hfov):.1f}°")

    def calculate_accurate_bearing(self, center_x):
        """
        使用相机内参计算更准确的方位角
        Args:
            center_x: 目标在图像中的X坐标
        Returns:
            方位角（弧度），右侧为正
        """
        if self.camera_info_received and self.focal_x is not None:
            # 使用相机内参进行准确计算
            # bearing = arctan((center_x - cx) / fx)
            pixel_offset = center_x - self.center_x
            bearing_rad = math.atan(pixel_offset / self.focal_x)
            return bearing_rad
        else:
            # 回退到原有方法
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            return raw_offset * (self.cam_hfov / 2.0)

    def calculate_vision_distance(self):
        """
        使用目标框大小和相机内参计算视觉测距
        Returns:
            视觉估算的距离（米），如果无法计算返回None
        """
        if not self.camera_info_received or self.box_width is None:
            return None
        
        if self.focal_x is None or self.target_width_m <= 0:
            return None
            
        # 使用目标宽度进行距离估算: distance = (focal_length * real_width) / pixel_width
        if self.box_width > 10:  # 确保检测框足够大
            vision_dist = (self.focal_x * self.target_width_m) / self.box_width
            # 合理性检查：距离应该在0.1-5米之间
            if 0.1 < vision_dist < 5.0:
                return vision_dist
        return None

    # ===============================================================
    # 角度融合方法
    # ===============================================================
    def fuse_angles(self, vision_angle, lidar_angle):
        """
        融合视觉和激光雷达的角度信息
        Args:
            vision_angle: 视觉计算的角度 (弧度)
            lidar_angle: 激光雷达计算的角度 (弧度，可为None)
        Returns:
            融合后的角度 (弧度)
        """
        if lidar_angle is None:
            return vision_angle
        
        # 检查是否刚检测到目标（1秒内）
        time_since_detection = (rospy.Time.now() - self.stable_track_time).to_sec()
        
        angle_diff = abs(vision_angle - lidar_angle)
        
        # 如果角度差异过大，可能是激光雷达检测错误
        if angle_diff > math.radians(30):
            rospy.loginfo_throttle(3.0, f"激光雷达角度差异过大: {math.degrees(angle_diff):.1f}°, 使用视觉角度")
            return vision_angle
        
        # 动态调整融合权重
        if time_since_detection < 2.0:  # 刚检测到目标的2秒内，主要信任视觉
            weight_lidar = 0.3
            weight_vision = 0.7
            rospy.loginfo_throttle(1.0, "刚检测到目标，主要使用视觉角度")
        elif angle_diff < math.radians(10):  # 角度差异小且稳定跟踪
            weight_lidar = 0.7
            weight_vision = 0.3
        else:  # 角度差异较大
            weight_lidar = 0.5
            weight_vision = 0.5
            
        fused_angle = weight_lidar * lidar_angle + weight_vision * vision_angle
        return fused_angle

    def find_closest_object_angle(self, scan):
        """
        从激光雷达数据中找到最近物体的角度
        Args:
            scan: LaserScan消息
        Returns:
            最近物体的角度 (弧度)，如果没找到返回None
        """
        min_dist = float('inf')
        closest_angle = None
        
        # 在前方±45度范围内寻找最近点
        search_range = math.radians(45)
        
        # 用于聚类的参数
        cluster_angles = []
        cluster_distances = []
        
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > search_range:
                continue
                
            dist = scan.ranges[i]
            if math.isfinite(dist) and 0.1 < dist < 3.0:  # 合理距离范围
                cluster_angles.append(angle)
                cluster_distances.append(dist)
                
                if dist < min_dist:
                    min_dist = dist
                    closest_angle = angle
        
        # 如果找到了有效点，进行进一步验证
        if closest_angle is not None and min_dist < 2.0:
            # 在最近点附近进行小范围平均，减少噪声影响
            nearby_angles = []
            nearby_weights = []
            
            for angle, dist in zip(cluster_angles, cluster_distances):
                if abs(angle - closest_angle) < math.radians(5) and abs(dist - min_dist) < 0.2:
                    nearby_angles.append(angle)
                    # 距离越近，权重越高
                    weight = 1.0 / (dist + 0.1)
                    nearby_weights.append(weight)
            
            if nearby_angles:
                # 加权平均
                total_weight = sum(nearby_weights)
                weighted_angle = sum(a * w for a, w in zip(nearby_angles, nearby_weights)) / total_weight
                self.closest_dist = min_dist
                return weighted_angle
        
        return None

    # ===============================================================
    # YOLO 回调：筛选最高置信度目标 → 计算朝向误差
    # ===============================================================
    def yolo_cb(self, msg: BoundingBoxes):
        """处理 YOLO 检测框，更新 bearing_rad"""
        # 若目标列表为空，直接返回
        if not self.target_classes:
            return

        # 找到匹配类别中置信度最高的框
        best_box = None
        best_prob = 0.0
        for box in msg.bounding_boxes:
            cls_name = box.Class.lower()
            if cls_name in self.target_classes and box.probability > best_prob:
                best_prob = box.probability
                best_box = box

        # 若找到了目标框，计算水平偏移 → 角度
        if best_box:
            # 计算检测框的中心和宽度，中心偏移用于角度误差，宽度用于距离判断
            center_x = (best_box.xmin + best_box.xmax) / 2.0
            self.box_width = best_box.xmax - best_box.xmin
            
            # 使用改进的角度计算方法
            raw_bearing = self.calculate_accurate_bearing(center_x)
            
            # 计算归一化偏移量（用于侧向控制）
            raw_offset = (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            # 一阶低通滤波，缓和跳变
            self.offset_norm = 0.7 * self.offset_norm + 0.3 * raw_offset
            
            # 对角度也进行滤波
            self.vision_bearing_rad = 0.7 * self.vision_bearing_rad + 0.3 * raw_bearing
            
            # 检查是否是新检测到的目标
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 0.5:
                self.just_detected = True
                self.stable_track_time = rospy.Time.now()
                rospy.loginfo("🎯 重新检测到目标，开始稳定跟踪")
            
            self.last_seen_time = rospy.Time.now()
            
            # 融合视觉和激光雷达角度
            self.bearing_rad = self.fuse_angles(self.vision_bearing_rad, self.lidar_bearing_rad)
            
            # 打印调试信息（大幅减少输出频率，提高性能）
            cam_info_status = "✅使用相机内参" if self.camera_info_received else "⚠️使用默认参数"
            rospy.loginfo_throttle(3.0, f"检测到 [{best_box.Class}] 置信度 {best_prob:.2f} ({cam_info_status})")
            rospy.loginfo_throttle(3.0, f"图像偏移: {self.offset_norm:.3f}, 视觉角度: {math.degrees(self.vision_bearing_rad):.1f}°")
            if self.lidar_bearing_rad is not None:
                angle_diff = abs(self.vision_bearing_rad - self.lidar_bearing_rad)
                rospy.loginfo_throttle(3.0, f"激光角度: {math.degrees(self.lidar_bearing_rad):.1f}°, "
                             f"差异: {math.degrees(angle_diff):.1f}°, 融合角度: {math.degrees(self.bearing_rad):.1f}°")
            else:
                rospy.loginfo_throttle(3.0, f"激光角度: N/A, 融合角度: {math.degrees(self.bearing_rad):.1f}°")

    # ===============================================================
    # 雷达回调：根据 bearing_rad 读取对应距离 + 计算最准确角度
    # ===============================================================
    def lidar_cb(self, scan: LaserScan):
        """在雷达数据中找到 bearing_rad 对应的距离，并计算最准确的目标角度"""
        # 若近期没看到目标，则不处理
        if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
            self.target_dist = None
            self.lidar_bearing_rad = None
            return

        # 1. 从激光雷达找到最近物体的角度（更准确的角度估计）
        self.lidar_bearing_rad = self.find_closest_object_angle(scan)
        
        # 2. 使用当前最佳角度估计来获取距离
        if self.lidar_bearing_rad is not None:
            # 使用激光雷达角度
            target_angle = self.lidar_bearing_rad
        else:
            # 回退到视觉角度
            target_angle = self.vision_bearing_rad
            
        # 将角度限制到激光有效角度范围内
        ang = max(scan.angle_min, min(scan.angle_max, target_angle))
        index = int((ang - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        
        # 改进：多点平均距离计算，减少噪声影响
        valid_distances = []
        search_radius = 3  # 在目标索引附近搜索的点数
        
        for i in range(max(0, index - search_radius), 
                      min(len(scan.ranges), index + search_radius + 1)):
            dist = scan.ranges[i]
            if math.isfinite(dist) and 0.05 < dist < 5.0:  # 合理距离范围
                valid_distances.append(dist)
        
        # 计算有效距离的中位数（比平均值更鲁棒）
        if valid_distances:
            valid_distances.sort()
            n = len(valid_distances)
            if n % 2 == 0:
                self.lidar_dist_raw = (valid_distances[n//2-1] + valid_distances[n//2]) / 2.0
            else:
                self.lidar_dist_raw = valid_distances[n//2]
        else:
            self.lidar_dist_raw = None
            
        # 距离融合和滤波
        self.vision_dist = self.calculate_vision_distance()
        fused_dist = None
        
        # 多传感器距离融合
        if self.lidar_dist_raw is not None and self.vision_dist is not None:
            # 如果两个距离差异很大，优先信任激光雷达
            dist_diff = abs(self.lidar_dist_raw - self.vision_dist)
            if dist_diff < 0.5:  # 差异小于50cm，进行融合
                fused_dist = 0.7 * self.lidar_dist_raw + 0.3 * self.vision_dist
            else:
                fused_dist = self.lidar_dist_raw  # 差异大，优先激光雷达
                rospy.logwarn_throttle(3.0, f"视觉距离差异过大: LiDAR={self.lidar_dist_raw:.2f}m vs Vision={self.vision_dist:.2f}m")
        elif self.lidar_dist_raw is not None:
            fused_dist = self.lidar_dist_raw
        elif self.vision_dist is not None:
            fused_dist = self.vision_dist
        
        # 距离滤波：使用低通滤波器平滑距离变化
        if fused_dist is not None:
            if self.target_dist_filtered is None:
                self.target_dist_filtered = fused_dist
            else:
                self.target_dist_filtered = (1 - self.dist_filter_alpha) * self.target_dist_filtered + \
                                          self.dist_filter_alpha * fused_dist
            self.target_dist = self.target_dist_filtered
        else:
            self.target_dist = None
            
        # 验证距离一致性
        if self.closest_dist is not None and self.target_dist is not None:
            dist_diff = abs(self.closest_dist - self.target_dist)
            if dist_diff > 0.3:  # 距离差异过大，可能检测错误
                rospy.logwarn(f"距离不一致: 最近点{self.closest_dist:.2f}m vs 目标点{self.target_dist:.2f}m")

    # ===============================================================
    # 主控制循环
    # ===============================================================
    def run(self):
        rate = rospy.Rate(20)  # 0 Hz 发送控制
        last_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if dt <= 0:
                dt = 0.1  # 默认10Hz
            last_time = current_time
            
            self.update_task_from_param()  # 每循环检查一次参数是否改变
            twist = Twist()
            
            # === 修改点：如果已到位，持续输出零速度
            if self.reached:
                self.cmd_pub.publish(twist)
                rate.sleep()
                continue

            # 若距离合法，进入跟踪 / 停止逻辑
            if self.target_dist is not None:
                # 刚检测到目标时的平滑过渡
                if self.just_detected:
                    # 如果搜索方向与目标方向相反，先减速然后平滑转向
                    if (self.search_direction > 0 and self.bearing_rad < -math.radians(15)) or \
                       (self.search_direction < 0 and self.bearing_rad > math.radians(15)):
                        # 搜索方向与目标方向相反，需要平滑转换
                        transition_factor = min(1.0, (rospy.Time.now() - self.stable_track_time).to_sec() / 1.0)
                        smoothed_bearing = self.search_direction * math.radians(10) * (1 - transition_factor) + \
                                         self.bearing_rad * transition_factor
                        rospy.loginfo(f"平滑转换中... 因子: {transition_factor:.2f}")
                    else:
                        smoothed_bearing = self.bearing_rad
                        self.just_detected = False  # 方向一致，可以正常跟踪
                else:
                    smoothed_bearing = self.bearing_rad
                
                # 1. 使用PID控制角速度
                self.angle_ = smoothed_bearing
                omega = self.pid_angle.compute(self.angle_, dt)
                twist.angular.z = omega
                
                # 2. 使用PID控制侧向速度：放宽侧向移动条件
                if abs(self.angle_) < math.radians(5):  # 放宽从3度到5度
                    lateral_speed = self.pid_lateral.compute(self.offset_norm, dt)
                    twist.linear.y = lateral_speed
                else:
                    twist.linear.y = 0.0
                    self.pid_lateral.reset()  # 重置侧向PID
                
                # 3. 使用PID控制前进速度
                forward_speed = 0.0
                # 放宽前进条件：让机器人更容易开始移动
                if abs(self.angle_) < math.radians(12) and abs(self.offset_norm) < 0.2:  # 放宽条件
                    # 计算距离误差
                    dist_error = self.target_dist - self.target_distance
                    forward_speed = self.pid_forward.compute(dist_error, dt)
                    
                    # 调整的安全限制：提高近距离速度，保持足够动力
                    if self.target_dist < 0.6:
                        forward_speed *= 0.95  # 近距离时降低到95%速度（从85%提高）
                    if self.target_dist < 0.4:
                        forward_speed *= 0.85  # 很近距离时降低到85%速度（从75%提高）
                    
                    # 确保最小速度，避免动力不足
                    if abs(forward_speed) > 0 and abs(forward_speed) < 0.05:
                        forward_speed = 0.05 if forward_speed > 0 else -0.05
                else:
                    self.pid_forward.reset()  # 重置前进PID
                    
                twist.linear.x = forward_speed

                # === 修改点：连帧判定停止
                angle_ok = abs(self.angle_) < math.radians(5)
                # 判定前后距离
                dist_ok = self.stop_min < self.target_dist < self.stop_max
                center_ok = abs(self.offset_norm) < 0.10
                
                if angle_ok and dist_ok and center_ok:
                    self.ok_counter += 1
                else:
                    self.ok_counter = 0

                if self.ok_counter >= 4:
                    twist = Twist()
                    self.reached = True
                    # 重置所有PID控制器
                    self.pid_angle.reset()
                    self.pid_lateral.reset()
                    self.pid_forward.reset()
                    rospy.loginfo("✅ 连续 4 帧满足阈值，已安全到位并锁定！")
                    break  # 跳出循环，保持停止状态
                    
                # 打印控制调试信息（降低频率提高性能）
                rospy.loginfo_throttle(3.0, f"[PID控制] 角度误差: {math.degrees(self.angle_):.1f}°, "
                                     f"距离: {self.target_dist:.2f}m, 偏移: {self.offset_norm:.3f}")
                if self.vision_dist is not None:
                    rospy.loginfo_throttle(3.0, f"[距离融合] LiDAR: {self.lidar_dist_raw:.2f}m, "
                                         f"Vision: {self.vision_dist:.2f}m, 融合: {self.target_dist:.2f}m")
            else:
                # 未检测到目标 / 距离无效 —— 原地旋转搜索
                search_speed = 0.35  # 提高搜索速度从0.2到0.35
                twist.angular.z = self.search_direction * search_speed
                twist.linear.x = 0.0
                
                # 重置PID控制器
                self.pid_angle.reset()
                self.pid_lateral.reset()
                self.pid_forward.reset()
                
                # 记录搜索方向，用于状态转换
                self.search_direction = twist.angular.z / abs(twist.angular.z) if twist.angular.z != 0 else self.search_direction
                
                rospy.logwarn_throttle(3.0, f"未找到目标，正在搜索... 方向: {'右转' if self.search_direction > 0 else '左转'}")

            # 发布速度指令
            self.cmd_pub.publish(twist)
            rate.sleep()

# --------------------------- 主函数入口 ---------------------------
if __name__ == '__main__':
    try:
        node = TargetFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass
