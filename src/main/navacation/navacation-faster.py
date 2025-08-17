# ps -ef | grep vino
# 
# 为了让小车经过多个导航点时不停留花时间来原地调整方向，可以在设置目标点时，将朝向角度设置为与前一个目标点相同。
# 这样小车就可以直接从前一个目标点到达下一个目标点，而不需要在目标点处调整方向。
# 具体来说，可以在每个目标点的朝向角度中设置与前一个目标点相同的值，例如在代码中的第1个目标点后，可以将第2个目标点的朝向角度设置为-0.009
# 与第1个目标点相同。同样的，可以在后续的目标点中重复这个步骤，以便小车可以顺利地到达每个目标点。

import rospy
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from move_base_msgs.msg import MoveBaseActionResult

# 设置多个目标点
goal_list = [
    [2.979, -0.709, 0.000, 0.000, 0.000, 0.443, 0.897],
    [2.603, -5.321, 0.000, 0.000, 0.000, -0.405, 0.914],
    [4.689, -0.725, 0.000, 0.000, 0.000, 0.951, 0.309],
    [4.448, -5.476, 0.000, 0.000, 0.000, -0.403, 0.915],
    [0.945, -0.376, 0.000, 0.000, 0.000, -0.718, 0.696],
    [0.558, -5.256, 0.000, 0.000, 0.000, 0.977, -0.212],
    [0.756, -2.413, 0.000, 0.000, 0.000, 0.345, 0.939],
    [0.860, -0.377, 0.000, 0.000, 0.000, 0.700, 0.714],
    [-0.090, 0.090, 0.000, 0.000, 0.000,  1.000, 0.000]
]

goal_index = 0

def callback(data):
    global goal_index
    if data.status.status == 3:
        # 到达E点，发布D点
        '''
        if goal_index == 1:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = -210  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 2.1  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)

            # 设置下一个目标点D
            mypose = PoseStamped()
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1

        # 到达D点，发布点C
        elif goal_index == 2:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = 310  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 3.1  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)

            # 设置下一个目标点C
            mypose = PoseStamped()
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1

        # 到达C点，发布点B
        elif goal_index == 3:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = 290  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 2.9  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)

            # 设置下一个目标点B
            mypose = PoseStamped()
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1

        # 到达B点，发布点坡道前
        elif goal_index == 4:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = 290  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 2.9  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)

            # 设置下一个目标点C
            mypose = PoseStamped()
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1
            
        # 到达F前面
        '''
        if goal_index == 6:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = 120  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 1.2  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()
            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)

            # 设置下一个目标点C
            mypose = PoseStamped()
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1
        # 到达F前面
        elif goal_index == 7:
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = 100  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 1.0  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = -50  # 目标旋转角度 (度)
            target_angle_rad = target_angle * 3.14159 / 180.0
            # 设置旋转时间，计算旋转速度
            rotation_time = 0.5  # 旋转时间 (秒)
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed

            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)
            mypose = PoseStamped()
            # 设置下一个目标点
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1
        
        elif goal_index < len(goal_list):
            mypose = PoseStamped()
            # 设置下一个目标点
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal_list[goal_index][0]
            mypose.pose.position.y = goal_list[goal_index][1]
            mypose.pose.position.z = goal_list[goal_index][2]
            mypose.pose.orientation.x = goal_list[goal_index][3]
            mypose.pose.orientation.y = goal_list[goal_index][4]
            mypose.pose.orientation.z = goal_list[goal_index][5]
            mypose.pose.orientation.w = goal_list[goal_index][6]
            turtle_vel_pub.publish(mypose)
            goal_index += 1

        else:
            rospy.loginfo("所有的目标点都到了!")

if __name__ == '__main__':
    # 创建一个发布器，用于发布机器人的移动命令
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # 创建一个Twist消息实例
    twist = Twist()

    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')

    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)

    mypose = PoseStamped()
    # 设置下一个目标点
    mypose.header.frame_id = 'map'
    mypose.pose.position.x = goal_list[goal_index][0]
    mypose.pose.position.y = goal_list[goal_index][1]
    mypose.pose.position.z = goal_list[goal_index][2]
    mypose.pose.orientation.x = goal_list[goal_index][3]
    mypose.pose.orientation.y = goal_list[goal_index][4]
    mypose.pose.orientation.z = goal_list[goal_index][5]
    mypose.pose.orientation.w = goal_list[goal_index][6]
    turtle_vel_pub.publish(mypose)
    goal_index += 1

    rospy.spin()    















