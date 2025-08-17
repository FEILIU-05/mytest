#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import rospy
import time

# 导入所有新的模块
from config_manager import ConfigManager
from process_manager import ProcessManager
from audio_manager import AudioManager
from qr_code_detector import QRCodeDetector
from object_detector import ObjectDetector
from traffic_light_detector import TrafficLightDetector
from navigator import Navigator

if __name__ == '__main__':
    rospy.init_node('navigator_v160_refactored_main', anonymous=False)
    
    # 获取脚本所在的根目录
    root_dir = os.path.dirname(os.path.abspath(__file__))

    # --- 1. 初始化管理器和检测器 ---
    
    # 配置管理器
    config_manager = ConfigManager(root_dir)
    if not config_manager.load_all_configs():
        rospy.logerr("❌ 配置加载失败，程序退出。")
        rospy.signal_shutdown("Config load failed")
        exit(1) # Exit with an error code

    # 进程管理器
    process_manager = ProcessManager()

    # 音频管理器 (需要传入 audio_config 部分)
    audio_manager = AudioManager(config_manager.app_config.get('audio', {}))

    # 感知检测器
    qr_detector = QRCodeDetector()
    
    # ObjectDetector 构造函数现在接收一个 obj_config 字典
    object_detector_config = config_manager.app_config.get('object_detector', {})
    object_detector = ObjectDetector(object_detector_config)

    traffic_light_detector = TrafficLightDetector()

    # --- 2. 初始系统检查与启动 ---

    # 检查摄像头状态（可选，但建议在启动前检查）
    if not process_manager.check_camera_status():
        rospy.logwarn("⚠️ 摄像头状态检查失败，但仍将尝试启动默认摄像头。")

    # 初始启动默认摄像头
    if not process_manager.ensure_camera_is_running():
        rospy.logwarn("⚠️ 默认摄像头启动失败，相关功能可能无法使用。")
    
    # 可以在这里选择性启动导航系统
    # if not process_manager.launch_navigation():
    #     rospy.logwarn("⚠️ 导航系统启动失败。")

    rospy.loginfo("系统初始化完成，即将开始执行导航任务...")

    # --- 3. 初始化并运行 Navigator ---
    try:
        # 将所有依赖注入到 Navigator
        nav = Navigator(
            config_manager=config_manager,
            process_manager=process_manager,
            audio_manager=audio_manager,
            qr_detector=qr_detector,
            object_detector=object_detector,
            traffic_light_detector=traffic_light_detector
        )
        
        nav.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")
    except Exception as e:
        rospy.logerr("发生未知顶层错误: %s", str(e))
        import traceback
        traceback.print_exc()
    finally:
        # 确保在程序结束时关闭所有由 ProcessManager 管理的进程
        process_manager.stop_process(process_manager.get_current_cam_process(), "最后的摄像头进程")
        rospy.loginfo("所有任务完成，程序退出。")