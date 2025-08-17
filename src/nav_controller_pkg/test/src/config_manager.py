#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import csv
import math
import yaml
import rospy

class ConfigManager(object):
    """
    统一管理所有配置文件加载和参数获取。
    """
    def __init__(self, root_dir):
        # root_dir 在此版本中将不再被用于构建文件路径，但可以保留以防未来需要
        self.root_dir = root_dir 
        self.app_config = {}
        self.tuning_cfg = {}
        self.wps = []

    def get_default_tuning(self):
        """返回默认的 tuning.yaml 参数"""
        return {
            'pos_tol': 0.04, 'yaw_tol_deg': 2.0, 'max_nav_time': 300,
            'refine_timeout': 15.0,
            'pid_lin': {'kp': 1.2, 'ki': 0.0, 'kd': 0.05},
            'pid_yaw': {'kp': 1.8, 'ki': 0.0, 'kd': 0.1},
            'max_lin': 0.35, 'max_ang': 1.5,
            'min_lin': 0.10, 'min_ang': 0.25
        }

    def _load_yaml(self, path, defaults=None):
        """通用 YAML 加载函数"""
        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            if defaults:
                for k, v in defaults.items():
                    if k not in cfg:
                        cfg[k] = v
                if 'yaw_tol_deg' in cfg:
                    cfg['yaw_tol'] = math.radians(cfg['yaw_tol_deg'])
            rospy.loginfo("✅ 成功加载配置文件: %s", os.path.basename(path))
            return cfg
        except Exception as e:
            rospy.logwarn('⚠️ 无法读取 %s: %s。', path, str(e))
            if defaults:
                rospy.logwarn('↳ 将使用默认参数。')
                if 'yaw_tol_deg' in defaults:
                    defaults['yaw_tol'] = math.radians(defaults['yaw_tol_deg'])
                return defaults
            return None

    def _load_csv(self, path):
        """加载路径点CSV文件"""
        wps = []
        try:
            with open(path, 'r') as f:
                for row in csv.reader(f):
                    if not row or row[0].lower() == 'tag' or len(row) < 5:
                        continue
                    wps.append({
                        'tag': row[0], 'x': float(row[1]), 'y': float(row[2]),
                        'qz': float(row[3]), 'qw': float(row[4])
                    })
            rospy.loginfo('✅ 已从 %s 加载 %d 个路径点', os.path.basename(path), len(wps))
        except Exception as e:
            rospy.logerr("❌ 加载路径点文件 %s 失败: %s", path, str(e))
        return wps

    def load_all_configs(self):
        """加载所有必需的配置文件"""
        # === START OF MODIFICATION ===
        # 请将 "/path/to/your/test_package" 替换为您 ROS 包的实际绝对路径
        # 例如，如果您的包在 /home/user/catkin_ws/src/test，那么这里就是 /home/user/catkin_ws/src/test
        base_path = "/path/to/your/test_package" 

        config_path = os.path.join(base_path, 'config', 'config.yaml')
        tuning_path = os.path.join(base_path, 'config', 'tuning.yaml')
        csv_path = os.path.join(base_path, 'config', 'points.csv')
        # === END OF MODIFICATION ===

        self.app_config = self._load_yaml(config_path)
        if not self.app_config:
            rospy.logerr("❌ 主配置文件 config.yaml 加载失败，无法继续！")
            return False

        self.tuning_cfg = self._load_yaml(tuning_path, self.get_default_tuning())
        self.wps = self._load_csv(csv_path)
        
        return True