#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess
import os

def play_audio(audio_file):
    """使用aplay播放指定的音频文件"""
    try:
        # 检查文件是否存在
        if not os.path.exists(audio_file):
            rospy.logerr(f"音频文件不存在: {audio_file}")
            return False
        
        # 使用aplay播放音频
        result = subprocess.run(
            ['aplay', '-q', audio_file],  # -q参数用于安静模式，减少输出
            capture_output=True,
            text=True,
            check=True
        )
        rospy.loginfo(f"成功播放音频: {audio_file}")
        return True
        
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"播放音频时出错: {e.stderr}")
        return False
    except Exception as e:
        rospy.logerr(f"发生未知错误: {str(e)}")
        return False

def audio_callback(msg):
    """处理接收到的消息并播放对应音频"""
    audio_file = msg.data
    
    # 如果路径不是绝对路径，假设音频文件在指定的目录中
    if not audio_file.startswith('/'):
        # 这里需要设置你的默认音频目录
        default_audio_dir = rospy.get_param('~audio_dir', '/home/ucar/Desktop/ucar/src/main/navacation/voice_packge/初始播报/小飞小飞快速出发执行任务_44100.wav')
        audio_file = os.path.join(default_audio_dir, audio_file)
    
    play_audio(audio_file)

def audio_player_node():
    """ROS音频播放节点"""
    # 初始化节点
    rospy.init_node('audio_player', anonymous=True)
    
    # 创建订阅者，订阅String类型的消息
    rospy.Subscriber('audio_play_topic', String, audio_callback)
    
    # 打印节点启动信息
    rospy.loginfo("音频播放节点已启动，等待音频文件路径...")
    
    # 进入循环等待消息
    rospy.spin()

if __name__ == '__main__':
    try:
        audio_player_node()
    except rospy.ROSInterruptException:
        pass