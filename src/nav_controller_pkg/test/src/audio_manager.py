#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import subprocess
import rospy

class AudioManager(object):
    """
    负责统一管理和播放音频文件。
    """
    def __init__(self, audio_config):
        self.qr_audio_map = audio_config.get('qr_map', {})
        self.qr_audio_dir = audio_config.get('qr_dir', '')
        self.object_audio_dir = audio_config.get('object_dir', '')
        self.traffic_light_map = audio_config.get('traffic_light_map', {})
        self.traffic_light_dir = audio_config.get('traffic_light_dir', '')

    def _play_audio_file(self, audio_path):
        """私有方法：执行音频播放"""
        if not os.path.exists(audio_path):
            rospy.logwarn("⚠️ 音频文件不存在: %s", audio_path.encode('utf-8', errors='ignore'))
            return

        try:
            # Popen ensures the script continues immediately without waiting for aplay
            subprocess.Popen(['aplay', audio_path.encode('utf-8')])
            rospy.loginfo("🔊 正在播放语音: %s", audio_path.encode('utf-8', errors='ignore'))
        except Exception as e:
            rospy.logwarn("语音播放失败: %s", str(e))

    def play_audio_for_qr(self, qr_data):
        """播放与二维码数据关联的音频"""
        # Ensure qr_data is unicode for consistent map lookup
        data = unicode(qr_data, 'utf-8', errors='ignore') if isinstance(qr_data, str) else qr_data
        filename = self.qr_audio_map.get(data)
        if filename:
            audio_path = os.path.join(self.qr_audio_dir, unicode(filename, 'utf-8', errors='ignore'))
            self._play_audio_file(audio_path)
        else:
            rospy.logwarn("未找到 '%s' 对应的语音文件", data.encode('utf-8', errors='ignore'))

    def play_audio_for_object(self, object_name):
        """播放与物体名称关联的音频"""
        # Ensure object_name is unicode
        name = unicode(object_name, 'utf-8', errors='ignore') if isinstance(object_name, str) else object_name
        audio_path = os.path.join(self.object_audio_dir, "{0}.wav".format(name))
        self._play_audio_file(audio_path)

    def play_traffic_light_audio(self, point_idx):
        """播放与交通灯导航点关联的音频"""
        filename = self.traffic_light_map.get(point_idx)
        if filename:
            audio_path = os.path.join(self.traffic_light_dir, unicode(filename, 'utf-8', errors='ignore'))
            self._play_audio_file(audio_path)
        else:
            rospy.logwarn("未找到第%d个导航点对应的交通灯语音文件", point_idx)