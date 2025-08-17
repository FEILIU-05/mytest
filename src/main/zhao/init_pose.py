#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose(x, y, qx, qy, qz, qw):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1.0)  # 等待连接

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # 添加基本协方差
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.0685389

    pub.publish(msg)
    rospy.loginfo("已初始化初始位姿: x=%.2f, y=%.2f, quat=[%.3f, %.3f, %.3f, %.3f]",
                  x, y, qx, qy, qz, qw)

def main():
    rospy.init_node('init_pose_from_csv')

    csv_path = '/home/ucar/Desktop/ucar/src/main/zhao/waypoints_quat.csv'

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 6:
                rospy.logwarn("CSV 第一行格式不完整")
                return
            x = float(row[0])
            y = float(row[1])
            qx = float(row[2])
            qy = float(row[3])
            qz = float(row[4])
            qw = float(row[5])
            publish_initial_pose(x, y, qx, qy, qz, qw)
            break  # 只处理第一行

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
