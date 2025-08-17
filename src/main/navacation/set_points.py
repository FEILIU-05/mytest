#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    x = position.x
    y = position.y
    z = position.z
    w = orientation.w
    z2 = orientation.z

    rospy.loginfo("Position: x={}, y={}, z={}, orientation: z2={}, w={}".format(x, y, z, z2, w))

def main():
    rospy.init_node('print_robot_position', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
