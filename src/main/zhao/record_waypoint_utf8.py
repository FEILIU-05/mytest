#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf
import csv
from geometry_msgs.msg import PoseStamped

def get_current_pose():
    listener = tf.TransformListener()
    rospy.loginfo("等待 tf /map → /base_link ...")
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    try:
        (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        x, y = trans[0], trans[1]
        qx, qy, qz, qw = rot
        return x, y, qx, qy, qz, qw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF 获取失败: %s", e)
        return None, None, None, None, None, None

def append_to_csv(file_path, x, y, qx, qy, qz, qw, comment=""):
    with open(file_path, 'a') as csvfile:  # Python 2 不需要 newline='' 参数
        writer = csv.writer(csvfile)
        writer.writerow([round(x, 3), round(y, 3), qx, qy, qz, qw, comment])
    rospy.loginfo("已保存点: x=%.3f, y=%.3f, quaternion=[%.3f, %.3f, %.3f, %.3f], 注释: %s",
                  x, y, qx, qy, qz, qw, comment)

def main():
    rospy.init_node('record_waypoint_quat_node')
    csv_path = '/home/ucar/Desktop/ucar/src/main/zhao/waypoint.csv'

    while not rospy.is_shutdown():
        raw_input("按回车记录当前点...")  # 使用 raw_input() 替代 input()
        x, y, qx, qy, qz, qw = get_current_pose()
        if x is None:
            continue
        comment = raw_input("输入注释（可留空）：")  # 使用 raw_input() 替代 input()
        append_to_csv(csv_path, x, y, qx, qy, qz, qw, comment)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
