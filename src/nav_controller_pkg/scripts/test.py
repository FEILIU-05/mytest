import rospy
from std_msgs.msg import String

def send_target():
    pub = rospy.Publisher('/target_data', String, queue_size=10)
    rospy.init_node('send_target_node', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        target = "牛奶"  # 小车识别二维码得到的目标
        pub.publish(target)
        rospy.loginfo(f"[小车] 已发布目标: {target}")
        rate.sleep()

def result_callback(msg):
    rospy.loginfo(f"[小车] 收到识别结果: {msg.data}")

if __name__ == '__main__':
    try:
        rospy.Subscriber('/recognition_result', String, result_callback)
        send_target()
    except rospy.ROSInterruptException:
        pass