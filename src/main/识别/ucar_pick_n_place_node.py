#!/usr/bin/env python3
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
from sensor_msgs.msg import LaserScan
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

class TargetFollower:
    """核心类：封装订阅、逻辑、发布等功能"""
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('target_follower', anonymous=False)

        # -------- 相机与算法参数 --------
        self.img_w = 480                    # YOLO 输入图像宽（像素）
        self.img_h = 320                    # YOLO 输入图像高（像素）
        self.cam_hfov = math.radians(124.8) # 相机水平视场角 (弧度)

        # -------- 目标类别相关 --------
        self.target_category = "vegetable"   # 当前任务大类（fruit / vegetable / dessert）
        self.target_classes = []    # 该大类对应 YOLO 具体类别列表
        self.update_task_from_param(force=True)  # 初始化一次

        # -------- 目标实时信息 --------
        self.last_seen_time = rospy.Time(0) # 最近一次看到目标的时间
        self.bearing_rad = 0.0             # 朝向误差 (弧度，右正)
        self.target_dist = None            # 由雷达测得的距离 (米)
        self.offset_norm = 0.0             # 图像中心偏移归一化值 (-1~1)

        # -------- 控制参数（可调）--------
        self.k_ang = 1.0   # 角速度比例增益
        self.k_lin = 0.5   # 线速度比例增益
        # -------- 运动限制 --------
        self.max_lin = 0.22
        self.max_ang = 0.6
        self.stop_min = 0.35  # 最近停止距离
        self.stop_max = 0.50  # 最远停止距离

        # -------- ROS 通讯 --------
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_cb)
        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)

        rospy.loginfo("目标跟踪节点已启动，等待检测并移动...")

        self.reached = False       # 是否已到达目标
        self.ok_counter = 0        # 连续满足计数

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
            center_x = (best_box.xmin + best_box.xmax) / 2.0
            raw_offset= (center_x - self.img_w / 2.0) / (self.img_w / 2.0)
            self.offset_norm = 0.7 * self.offset_norm + 0.3 * raw_offset  # 平滑处理
            self.bearing_rad = -self.offset_norm * (self.cam_hfov / 2.0)           # 右侧为负角
            self.last_seen_time = rospy.Time.now()
            # 打印调试信息
            rospy.loginfo(f"检测到 [{best_box.Class}] 置信度 {best_prob:.2f}, 方位角 {math.degrees(self.bearing_rad):.1f}°")

    # ===============================================================
    # 雷达回调：根据 bearing_rad 读取对应距离
    # ===============================================================
    def lidar_cb(self, scan: LaserScan):
        """在雷达数据中找到 bearing_rad 对应的距离"""
        # 若近期没看到目标，则不处理
        if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
            self.target_dist = None
            return

        # 将 bearing 限制到激光有效角度范围内
        ang = max(scan.angle_min, min(scan.angle_max, self.bearing_rad))
        index = int((ang - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        dist = scan.ranges[index]
        # 过滤无效测距（Inf、NaN 或过小）
        if math.isfinite(dist) and dist > 0.05:
            self.target_dist = dist
        else:
            self.target_dist = None

    # ===============================================================
    # 主控制循环
    # ===============================================================
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz 发送控制
        while not rospy.is_shutdown():
            self.update_task_from_param()  # 每循环检查一次参数是否改变
            twist = Twist()
            # === 修改点：如果已到位，持续输出零速度
            if self.reached:
                self.cmd_pub.publish(twist)
                rate.sleep()
                continue

            # 若距离合法，进入跟踪 / 停止逻辑
            if self.target_dist is not None:
                # 1. 角度闭环
                twist.angular.z = max(-self.max_ang,
                                      min(self.max_ang, self.k_ang * self.bearing_rad))

                # 3. 距离闭环（仅在角度误差 < 10° |offset|<0.15 时前进）
                if abs(self.bearing_rad) < math.radians(10) and abs(self.offset_norm) < 0.18:
                    dist_err = self.target_dist - (self.stop_min + self.stop_max) / 2.0
                    twist.linear.x = max(-self.max_lin, min(
                        self.max_lin, 
                        self.k_lin * dist_err))
                else:
                    twist.linear.x = 0.0

                # === 修改点：连帧判定停止
                angle_ok = abs(self.bearing_rad) < math.radians(5)
                dist_ok = self.stop_min < self.target_dist < self.stop_max
                if angle_ok and dist_ok:
                    self.ok_counter += 1
                else:
                    self.ok_counter = 0

                if self.ok_counter >= 5:
                    twist = Twist()
                    self.reached = True
                    rospy.loginfo("✅ 连续 5 帧满足阈值，已安全到位并锁定！")
                    break  # 跳出循环，保持停止状态
            else:
                # 未检测到目标 / 距离无效 —— 原地慢速旋转搜索
                twist.angular.z = 0.4
                twist.linear.x = 0.0
                rospy.logwarn_throttle(2.0, "未找到目标，正在原地搜索...")

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
