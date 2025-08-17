#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import subprocess
import signal
import time
import rospy

class ProcessManager(object):
    """
    负责启动、停止和管理外部ROS进程/节点。
    """
    def __init__(self):
        self._usb_cam_process = None

    def stop_process(self, process, process_name="进程"):
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

    def ensure_camera_is_running(self):
        """
        确保默认摄像头(usb_cam)正在运行。
        如果已有进程在运行，会先将其关闭，然后再启动一个新的。
        """
        rospy.loginfo("🔄 准备启动/重启默认摄像头...")
        self.stop_process(self._usb_cam_process, "旧摄像头进程")

        rospy.loginfo("🚀 正在启动新的 usb_cam 节点...")
        if not os.path.exists('/dev/video0'):
            rospy.logerr("❌ 摄像头设备 /dev/video0 不存在，无法启动。")
            self._usb_cam_process = None
            return False

        try:
            command = ['roslaunch', 'usb_cam', 'usb_cam-test.launch']
            # 使用Popen而不是call，以便管理进程
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            time.sleep(3) # Give it some time to launch
            if process.poll() is None: # Process is still running
                rospy.loginfo("✅ 默认摄像头进程已成功启动。")
                self._usb_cam_process = process
                return True
            else:
                _, stderr = process.communicate()
                rospy.logerr("❌ 默认摄像头启动失败，错误: %s", stderr.decode('utf-8', errors='ignore'))
                self._usb_cam_process = None
                return False

        except Exception as e:
            rospy.logerr("❌ 启动 usb_cam-test.launch 时发生异常: %s", str(e))
            self._usb_cam_process = None
            return False

    def launch_ucar_camera(self):
        """
        启动 ucar_camera 的 launch 文件，用于特定任务阶段。
        此函数会先关闭传入的旧进程，然后启动新的 ucar_camera 进程。
        """
        rospy.loginfo("📷 准备启动专用的 ucar_camera...")
        self.stop_process(self._usb_cam_process, "旧摄像头进程")

        rospy.loginfo("🚀 正在启动 roslaunch ucar_camera usb_cam-test.launch...")
        command = ['roslaunch', 'ucar_camera', 'usb_cam-test.launch']
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            time.sleep(3) # Give it some time to launch
            if process.poll() is None: # Process is still running
                rospy.loginfo("✅ ucar_camera 已成功启动。")
                self._usb_cam_process = process
                return True
            else:
                _, stderr = process.communicate()
                rospy.logerr("❌ ucar_camera 启动失败，错误: %s", stderr.decode('utf-8', errors='ignore'))
                self._usb_cam_process = None
                return False

        except Exception as e:
            rospy.logerr("❌ 启动 ucar_camera launch 文件时发生异常: %s", str(e))
            self._usb_cam_process = None
            return False

    def launch_navigation(self):
        """
        启动导航系统。
        注意：这个通常会阻塞，或者需要另起线程，这里保留原样。
        """
        rospy.loginfo("🚀 即将启动导航系统...")
        command = ['roslaunch', 'nav_controller_pkg', 'nav.launch']
        try:
            # 使用 Popen，这样它不会阻塞当前进程
            subprocess.Popen(command)
            rospy.loginfo("已发送 nav.launch 启动命令。等待 10 秒...")
            time.sleep(10) # Give it time to initialize
            rospy.loginfo("✅ 导航系统应已准备就绪。")
            return True
        except Exception as e:
            rospy.logerr("❌ 启动 nav.launch 失败: %s", str(e))
            return False

    def check_camera_status(self):
        """
        检查系统是否有可用的摄像头设备。
        """
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

    def get_current_cam_process(self):
        """获取当前摄像头进程句柄"""
        return self._usb_cam_process

    def set_current_cam_process(self, process):
        """设置当前摄像头进程句柄"""
        self._usb_cam_process = process