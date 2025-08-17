#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf
import csv

def get_current_pose():
    listener = tf.TransformListener()
    rospy.loginfo("等待 tf /map → /base_link ...")
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    try:
        (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        x, y = trans[0], trans[1]
        # rot = [qx, qy, qz, qw] → 只保留 qz, qw
        qz, qw = rot[2], rot[3]
        return x, y, qz, qw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF 获取失败: %s", e)
        return None, None, None, None

def append_to_csv(file_path, x, y, qz, qw, comment=""):
    with open(file_path, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['frame', round(x, 3), round(y, 3), round(qz, 6), round(qw, 6), comment])
    rospy.loginfo("✅ 已保存: frame, x=%.3f, y=%.3f, qz=%.6f, qw=%.6f, 注释=%s", x, y, qz, qw, comment)

def main():
    rospy.init_node('record_waypoint_for_nav')
    csv_path = '/home/ucar/Desktop/ucar/src/nav_controller_pkg/src/points2.csv'

    while not rospy.is_shutdown():
        raw_input("按回车记录当前点...")
        x, y, qz, qw = get_current_pose()
        if x is None:
            continue
        comment = raw_input("输入注释（可留空）：")
        append_to_csv(csv_path, x, y, qz, qw, comment)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
