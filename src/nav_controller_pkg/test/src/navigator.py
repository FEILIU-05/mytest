#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import division
import os
import csv
import math
import yaml
import rospy
import tf
import actionlib
import time
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

# 导入其他模块
from utils import q_to_yaw, ang_diff, PID # 导入PID用于精调
from process_manager import ProcessManager
from audio_manager import AudioManager
from qr_code_detector import QRCodeDetector
from object_detector import ObjectDetector
from traffic_light_detector import TrafficLightDetector
from config_manager import ConfigManager # 导入ConfigManager

class Navigator(object):
    def __init__(self, config_manager, process_manager, audio_manager, 
                 qr_detector, object_detector, traffic_light_detector):
        
        self.config_manager = config_manager
        self.process_manager = process_manager
        self.audio_manager = audio_manager
        self.qr_detector = qr_detector
        self.object_detector = object_detector
        self.traffic_light_detector = traffic_light_detector

        self.tf = tf.TransformListener()
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("正在等待 move_base action 服务器...")
        self.mb.wait_for_server()
        rospy.loginfo("✅ move_base action 服务器已连接。")

        # 从 ConfigManager 获取配置
        self.tuning_cfg = self.config_manager.tuning_cfg
        self.app_config = self.config_manager.app_config
        self.wps = self.config_manager.wps

        self.pid_x = PID(**self.tuning_cfg['pid_lin'])
        self.pid_y = PID(**self.tuning_cfg['pid_lin'])
        self.pid_yaw = PID(**self.tuning_cfg['pid_yaw'])

        self.detected_objects = [] # 用于存储检测到的物体

    # ==================== 模块化动作函数 ====================

    def _action_detect_qr(self, point_idx):
        """动作：在指定点识别二维码"""
        rospy.loginfo('🔍 [动作] 开始二维码识别...')
        rate = rospy.Rate(10)
        timeout = 60 # 二维码识别超时
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn('⏰ 二维码识别超时')
                return {'status': 'timeout'}

            detected, qr_data = self.qr_detector.detect_qr_code()
            if detected:
                # 设置ROS参数，由其他节点订阅，同时本地保存任务信息
                rospy.set_param("/procurement_task", qr_data.lower())
                rospy.loginfo("已设置ROS参数: /procurement_task = %s" % qr_data.lower().encode('utf-8'))
                
                # 播放音频
                self.audio_manager.play_audio_for_qr(qr_data)
                
                rospy.loginfo('✅ 二维码识别完成，继续导航...')
                time.sleep(2) # 播放音频留出时间
                return {'status': 'success', 'qr_data': qr_data}
            rate.sleep()
        return {'status': 'failed'}

    def _action_detect_object(self, point_idx):
        """动作：在指定点识别物体"""
        rospy.loginfo('🔍 [动作] 到达第%d点, 开始物体识别...', point_idx)
        detected, object_name = self.object_detector.run_object_detection()

        if detected and object_name:
            procurement_task = rospy.get_param('/procurement_task', '').strip().lower()
            target_classes = self.object_detector.category_items.get(procurement_task, [])

            # 检查检测到的物体是否在当前任务类别中
            if object_name in target_classes:
                # 检查是否是重复检测
                if object_name not in self.detected_objects:
                    self.detected_objects.append(object_name)
                    rospy.loginfo('✅ 检测到目标物体: %s', object_name.encode('utf-8', errors='ignore'))
                    
                    rospy.loginfo("🛑 播报前停止0.5秒...")
                    self.cmd_pub.publish(Twist()) # 停止机器人
                    time.sleep(0.5)

                    self.audio_manager.play_audio_for_object(object_name)
                    rospy.set_param("/detected_objects", self.detected_objects) # 更新ROS参数
                    rospy.loginfo('📝 已记录物体: %s. 当前列表: %s', 
                                  object_name.encode('utf-8', errors='ignore'), 
                                  str(self.detected_objects).encode('utf-8', errors='ignore'))
                    return {'status': 'success', 'object_name': object_name, 'target_found': True}
                else:
                    rospy.loginfo('⚠️ 物体 %s 已经检测过了', object_name.encode('utf-8', errors='ignore'))
                    return {'status': 'duplicate'}
            else:
                rospy.loginfo('⚠️ 检测到的物体 %s 不在当前任务类别中', object_name.encode('utf-8', errors='ignore'))
                return {'status': 'wrong_category'}
        else:
            rospy.loginfo('❌ 在第%d点未检测到目标物体，继续前往下一个导航点', point_idx)
            return {'status': 'not_found'}

    def _action_launch_special_camera(self, point_idx):
        """动作：执行特殊摄像头操作（启动 ucar_camera）"""
        rospy.loginfo('📷 [动作] 到达第%d个导航点，执行特殊摄像头操作...', point_idx)
        rospy.loginfo('🛑 机器人已停止，将在此处静止10秒...')
        self.cmd_pub.publish(Twist())
        time.sleep(10)

        rospy.loginfo('⏰ 10秒静止时间到，开始启动 ucar_camera...')
        # 调用 ProcessManager 启动 ucar_camera
        if self.process_manager.launch_ucar_camera():
            rospy.loginfo('✅ 特殊摄像头操作完成，继续导航...')
            return {'status': 'success'}
        else:
            rospy.logerr('❌ 特殊摄像头启动失败！')
            return {'status': 'failed', 'reason': 'camera_launch_failed'}

    def _action_handle_traffic_light(self, point_idx):
        """动作：在指定点处理交通灯"""
        rospy.loginfo('🚦 [动作] 到达第%d个导航点，开始内置交通灯识别...', point_idx)
        rospy.loginfo('🛑 机器人已停止，将在此处静止10秒进行交通灯识别...')
        self.cmd_pub.publish(Twist())

        detection_time = 10.0
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        green_count, red_count, last_log_time = 0, 0, 0

        rospy.loginfo('⏰ 开始10秒交通灯检测...')
        while (rospy.Time.now() - start_time).to_sec() < detection_time and not rospy.is_shutdown():
            current_status = self.traffic_light_detector.get_status()
            if current_status == 'green': green_count += 1
            elif current_status == 'red': red_count += 1

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if int(elapsed) > last_log_time:
                last_log_time = int(elapsed)
                rospy.loginfo('📊 [%.1fs] 实时统计 - 绿灯:%d, 红灯:%d', elapsed, green_count, red_count)
            
            if green_count >= 30: # If green is stably detected early
                rospy.loginfo('✅ 绿灯信号稳定，提前确认！')
                break
            rate.sleep()

        rospy.loginfo('📋 10秒检测完成，最终统计: 绿灯 %d 次, 红灯 %d 次', green_count, red_count)

        if green_count > red_count and green_count >= 20: # Threshold for green light decision
            rospy.loginfo('✅ 根据统计结果，判定为绿灯！')
            self.audio_manager.play_traffic_light_audio(point_idx)
            return {'status': 'green_light'}
        else:
            rospy.loginfo('❌ 未检测到足够的绿灯信号，判定为红灯或无信号')
            return {'status': 'red_light'}

    def _execute_point_action(self, point_idx):
        """根据配置执行导航点对应的动作"""
        action_map = self.app_config.get('point_actions', {})
        action_name = action_map.get(point_idx)

        if not action_name:
            return None # 此点无特定动作

        # 使用 getattr 动态调用对应的动作函数
        action_func_name = '_action_' + action_name
        action_func = getattr(self, action_func_name, None)
        if callable(action_func):
            rospy.loginfo("⚙️ 正在执行动作: %s (导航点 %d)", action_name, point_idx)
            return action_func(point_idx)
        else:
            rospy.logwarn("⚠️ 未找到与动作 '%s' (对应函数 %s) 对应的实现函数。", action_name, action_func_name)
            return None
            
    # ==================== 主运行与导航逻辑 ====================

    def run(self):
        """主任务流程"""
        target_detected = False # 标志：是否已完成物体识别任务
        traffic_light_logic_finished = False # 标志：是否已完成交通灯逻辑
        skip_to_point = None # 用于流程跳转

        # 初始化 ROS 参数
        rospy.set_param("/procurement_task", "")
        rospy.set_param("/detected_objects", [])
        self.detected_objects = []

        # 确保初始摄像头启动
        # This is now handled in main.py, but we can re-check or log here
        if self.process_manager.get_current_cam_process() is None:
            rospy.logwarn("初始默认摄像头未启动，部分功能可能受影响。")

        for idx, wp in enumerate(self.wps, 1):
            if skip_to_point is not None and idx < skip_to_point:
                rospy.loginfo('⏭️ 跳过第%d个导航点 (跳转目标: %s)', idx, skip_to_point)
                continue
            skip_to_point = None # 重置跳转标志

            rospy.loginfo('----------------------------------------')
            rospy.loginfo('→ 处理 %d/%d (tag=%s)', idx, len(self.wps), wp['tag'].encode('utf-8', errors='ignore'))
            rospy.loginfo('----------------------------------------')

            self.navigate(wp) # 执行导航到当前路径点

            # 如果某些逻辑已经完成，则跳过后续相关检查
            action_name_at_point = self.app_config.get('point_actions', {}).get(idx)
            if target_detected and action_name_at_point == 'detect_object':
                rospy.loginfo("ℹ️ 已找到目标物体，跳过此点的物体检测动作。")
                continue
            if traffic_light_logic_finished and action_name_at_point == 'handle_traffic_light':
                rospy.loginfo("ℹ️ 交通灯逻辑已完成，跳过此点的交通灯检测动作。")
                continue

            # 执行该点配置的动作
            result = self._execute_point_action(idx)

            # 根据动作结果控制流程
            if result:
                if result.get('target_found'): # 物体检测成功
                    target_detected = True
                    skip_to_point = 6 # 跳转到第6个点（假设这是下一阶段的开始点）
                    rospy.loginfo('⏭️ 物体检测成功，设置跳转到第6个导航点')
                    self.print_task_summary()
                
                if result.get('status') == 'green_light': # 交通灯绿灯
                    if idx == 7: # 假设交通灯在第7个点处理
                        traffic_light_logic_finished = True
                        skip_to_point = len(self.wps) + 1 # 结束所有后续导航
                        rospy.loginfo('⏭️ 绿灯通过，设置跳转到任务结束')
                elif result.get('status') == 'red_light': # 交通灯红灯
                    if idx == 7:
                        rospy.logwarn('⛔️ 交通灯为红灯，任务可能需要在此等待或停止。')
                        # 根据需求决定在此是等待还是停止程序
                        # 例如：可以进入一个循环等待直到绿灯或超时
                        # for simplicity here, we just log and continue for now
                        pass

        rospy.loginfo("🏁 所有导航点处理完毕。")
        self.print_task_summary()

    def print_task_summary(self):
        """打印最终任务摘要"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("📋 任务摘要")
        rospy.loginfo("=" * 50)
        procurement_task = rospy.get_param('/procurement_task', 'N/A').strip().lower()
        rospy.loginfo("🎯 当前任务类别: %s", procurement_task.encode('utf-8', errors='ignore'))
        
        target_classes = self.object_detector.category_items.get(procurement_task, [])
        rospy.loginfo("📝 目标物体列表: %s", str(target_classes).encode('utf-8', errors='ignore'))
        
        detected_objects = rospy.get_param("/detected_objects", [])
        rospy.loginfo("✅ 已检测到的物体: %s", str(detected_objects).encode('utf-8', errors='ignore'))
        
        if target_classes:
            try:
                success_rate = len(detected_objects) / float(len(target_classes)) * 100
                rospy.loginfo("📊 检测成功率: %.1f%% (%d/%d)", success_rate, len(detected_objects), len(target_classes))
            except ZeroDivisionError:
                 rospy.loginfo("📊 检测成功率: N/A (目标列表为空)")
        else:
            rospy.loginfo("📊 检测成功率: 0%% (无目标或任务未开始)")
        rospy.loginfo("=" * 50)

    def navigate(self, wp):
        """执行粗导航和精细调整"""
        goal = self.build_goal(wp)
        self.mb.send_goal(goal)
        if not self.wait_for_mb():
            rospy.logerr('❌ 粗导航失败 (tag=%s)', wp['tag'].encode('utf-8', errors='ignore'))
            # 在这里可以添加失败处理逻辑，例如重试、跳过或终止
            # 对于关键路径点，可能需要 raise exception 或 rospy.signal_shutdown
            return # For now, simply return

        rospy.loginfo('✔ 粗导航完成')
        # 仅对 'frame' 标签的点进行精调 (这与原始代码逻辑一致)
        if wp.get('tag', '').lower() == 'frame':
            rospy.loginfo("...开始精细调整...")
            self.refine(goal)

    def wait_for_mb(self):
        """等待 move_base 完成导航，或超时"""
        start = rospy.Time.now()
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            state = self.mb.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            if state in [GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                 rospy.logerr("MoveBase导航失败，状态: %d", state)
                 return False
            if (rospy.Time.now() - start).to_sec() > self.tuning_cfg['max_nav_time']:
                self.mb.cancel_goal()
                rospy.logerr("导航超时！")
                return False
            r.sleep()
        return False

    def refine(self, goal):
        """使用PID控制器进行位置和姿态的精细调整"""
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        phase = 'trans' # 'trans' for translation, 'rot' for rotation
        for pid in (self.pid_x, self.pid_y, self.pid_yaw):
            pid.reset()

        while (rospy.Time.now() - t0).to_sec() < self.tuning_cfg['refine_timeout']:
            dx_r, dy_r, yaw_err, dist = self.compute_errors(goal)
            
            if dist >= 999: # Indicates TF transform failed in compute_errors
                rospy.logerr("精调时TF变换失败，已终止精调。")
                break

            if phase == 'trans':
                if dist <= self.tuning_cfg['pos_tol']:
                    phase = 'rot'
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw): # Reset PIDs for new phase
                        pid.reset()
                    rospy.loginfo('✅ 平移完成 %.3fm → 开始旋转', dist)
                    continue
                cmd = self.cmd_trans(dx_r, dy_r)
            else: # phase == 'rot'
                if abs(yaw_err) <= self.tuning_cfg['yaw_tol']:
                    rospy.loginfo('✅ 旋转完成 %.2f°', math.degrees(yaw_err))
                    break
                cmd = self.cmd_rot(yaw_err)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist()) # Stop robot after refine

    def compute_errors(self, goal):
        """计算当前位置与目标位置的误差"""
        try:
            # Wait for transform to be available
            self.tf.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.2))
            (t, r) = self.tf.lookupTransform('map', 'base_link', rospy.Time(0))
        except Exception as e:
            rospy.logwarn("计算误差时TF变换失败: %s", str(e))
            return 0, 0, 0, 999 # Return a high distance to indicate failure

        dx = goal.target_pose.pose.position.x - t[0]
        dy = goal.target_pose.pose.position.y - t[1]
        dist = math.hypot(dx, dy)
        yaw_curr = q_to_yaw(Quaternion(*r))
        yaw_goal = q_to_yaw(goal.target_pose.pose.orientation)
        yaw_err = ang_diff(yaw_goal, yaw_curr) # Signed angular error
        
        # Transform position error into robot's frame
        dx_r = math.cos(yaw_curr) * dx + math.sin(yaw_curr) * dy
        dy_r = -math.sin(yaw_curr) * dx + math.cos(yaw_curr) * dy
        
        current_phase = 'trans' if dist > self.tuning_cfg['pos_tol'] else 'rot'
        msg = '误差: {:.3f}m  {:.2f}°  [{}]'.format(
            dist, math.degrees(yaw_err), current_phase)
        rospy.loginfo_throttle(1, msg)
        return dx_r, dy_r, yaw_err, dist
    
    def cmd_trans(self, dx_r, dy_r):
        """计算平移控制指令"""
        dt = 1.0 / 20.0 # Assuming 20 Hz loop rate for PID dt
        d = math.hypot(dx_r, dy_r)
        v = self.pid_x.step(d, dt) # Use pid_x for linear velocity magnitude
        
        # Compute unit vector towards target
        ux, uy = (dx_r / d, dy_r / d) if d > 1e-4 else (0, 0)
        
        cmd = Twist()
        if abs(v) > self.tuning_cfg.get('min_lin', 0.05): # Apply minimum linear speed
            v = max(-self.tuning_cfg['max_lin'], min(self.tuning_cfg['max_lin'], v))
            cmd.linear.x = v * ux
            cmd.linear.y = v * uy
        return cmd

    def cmd_rot(self, yaw_err):
        """计算旋转控制指令"""
        dt = 1.0 / 20.0
        w = self.pid_yaw.step(yaw_err, dt)
        cmd = Twist()
        if abs(w) > self.tuning_cfg.get('min_ang', 0.05): # Apply minimum angular speed
            cmd.angular.z = max(-self.tuning_cfg['max_ang'], min(self.tuning_cfg['max_ang'], w))
        return cmd
    
    @staticmethod
    def build_goal(wp):
        """根据路径点数据构建 MoveBaseGoal"""
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = wp['x']
        g.target_pose.pose.position.y = wp['y']
        g.target_pose.pose.orientation.z = wp['qz']
        g.target_pose.pose.orientation.w = wp['qw']
        return g