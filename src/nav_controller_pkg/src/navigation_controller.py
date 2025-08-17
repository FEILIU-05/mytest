#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
navigation_controller.py  – V1.5.1
----------------------------------
* 修复 loginfo_throttle 调用方式
"""

from __future__ import division
import os 
import csv 
import math
import yaml
import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


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


class Navigator(object):
    def __init__(self, csv_path, yaml_path):
        rospy.init_node('navigator_v151')
        self.tf = tf.TransformListener()
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mb.wait_for_server()

        self.cfg = self.load_yaml(yaml_path)
        self.wps = self.load_csv(csv_path)

        self.pid_x = PID(**self.cfg['pid_lin'])
        self.pid_y = PID(**self.cfg['pid_lin'])
        self.pid_yaw = PID(**self.cfg['pid_yaw'])

    # ---------- YAML ----------
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

    # ---------- CSV ----------
    @staticmethod
    def load_csv(path):
        wps = []
        for row in csv.reader(open(path)):
            if not row:                         # ← 先排除空行
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

    # ---------- 主循环 ----------
    def run(self):
        for idx, wp in enumerate(self.wps, 1):
            rospy.loginfo('→ 处理 %d/%d  (tag=%s)', idx, len(self.wps), wp['tag'])
            self.navigate(wp)

    # ---------- 单点 ----------
    def navigate(self, wp):
        goal = self.build_goal(wp)
        self.mb.send_goal(goal)
        if not self.wait_for_mb():
            rospy.logerr('❌ 粗导航失败')
            return
        rospy.loginfo('✔ 粗导航完成')
        if wp['tag'].lower() == 'frame':
            self.refine(goal)

    # ---------- 等待 move_base ----------
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

    # ---------- 微调 ----------
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

    # ---------- 误差 ----------
    def compute_errors(self, goal):
        try:
            self.tf.waitForTransform('map', 'base_link',
                                     rospy.Time(0), rospy.Duration(0.2))
            (t, r) = self.tf.lookupTransform('map', 'base_link', rospy.Time(0))
        except Exception:
            return 0, 0, 0, 999

        dx = goal.target_pose.pose.position.x - t[0]
        dy = goal.target_pose.pose.position.y - t[1]
        dist = math.hypot(dx, dy)

        yaw_curr = q_to_yaw(Quaternion(*r))
        yaw_goal = q_to_yaw(goal.target_pose.pose.orientation)
        yaw_err = ang_diff(yaw_goal, yaw_curr)

        # 投影到机器人坐标
        dx_r = math.cos(yaw_curr) * dx + math.sin(yaw_curr) * dy
        dy_r = -math.sin(yaw_curr) * dx + math.cos(yaw_curr) * dy

        msg = '误差: {:.3f}m  {:.2f}°  [{}]'.format(
            dist, math.degrees(yaw_err),
            'trans' if dist > self.cfg['pos_tol'] else 'rot')
        rospy.loginfo_throttle(1, msg)
        return dx_r, dy_r, yaw_err, dist

    # ---------- 速度命令 ----------
    def cmd_trans(self, dx_r, dy_r):
        dt = 1.0 / 20.0
        # 1. 误差模长
        d = math.hypot(dx_r, dy_r)
        v = self.pid_x.step(d, dt)           # 用同一 PID 控距离
        # 2. 方向单位向量
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
            cmd.angular.z = max(-self.cfg['max_ang'],
                                min(self.cfg['max_ang'], w))
        return cmd

    # ---------- goal ----------
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
    root = os.path.dirname(os.path.abspath(__file__))
    try:
        Navigator(os.path.join(root, 'points2.csv'),
                  os.path.join(root, 'tuning.yaml')).run()
    except rospy.ROSInterruptException:
        pass
