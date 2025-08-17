#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(x, y, qx, qy, qz, qw, comment=""):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    rospy.loginfo("发送目标: x=%.2f, y=%.2f, quat=[%.3f, %.3f, %.3f, %.3f], 注释: %s",
                  x, y, qx, qy, qz, qw, comment)
    client.send_goal(goal)
    
    # 等待移动完成，超时时间设为60秒
    success = client.wait_for_result(rospy.Duration(60.0))
    
    if success:
        result = client.get_result()
        rospy.loginfo("到达目标点: %s", comment)
    else:
        rospy.logwarn("未能按时到达目标点: %s", comment)
        client.cancel_goal()  # 取消目标
    
    rospy.sleep(1.0)  # 短暂停留后再前往下一个目标

def main():
    rospy.init_node('send_waypoints_quat')

    csv_path = '/home/ucar/Desktop/ucar/src/main/zhao/waypoint.csv'
    try:
        with open(csv_path, 'r') as f:
            reader = list(csv.reader(f))
            if not reader:
                rospy.logerr("CSV 文件为空")
                return

            # 直接发送所有航点（包括原来用于初始化的第一行）
            for row in reader:
                if len(row) < 6:
                    rospy.logwarn("跳过格式错误的行: %s", row)
                    continue
                x = float(row[0])
                y = float(row[1])
                qx = float(row[2])
                qy = float(row[3])
                qz = float(row[4])
                qw = float(row[5])
                comment = row[6] if len(row) > 6 else ""
                send_goal(x, y, qx, qy, qz, qw, comment)
                
    except IOError as e:
        rospy.logerr("无法打开CSV文件: %s", e)
    except Exception as e:
        rospy.logerr("发生错误: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")