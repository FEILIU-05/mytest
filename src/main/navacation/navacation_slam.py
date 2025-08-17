# ps -ef | grep vino
# 
# 为了让小车经过多个导航点时不停留花时间来原地调整方向，可以在设置目标点时，将朝向角度设置为与前一个目标点相同。
# 这样小车就可以直接从前一个目标点到达下一个目标点，而不需要在目标点处调整方向。
# 具体来说，可以在每个目标点的朝向角度中设置与前一个目标点相同的值，例如在代码中的第1个目标点后，可以将第2个目标点的朝向角度设置为-0.009
# 与第1个目标点相同。同样的，可以在后续的目标点中重复这个步骤，以便小车可以顺利地到达每个目标点。
import subprocess
import rospy
import time
import threading
import os
import re
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
import json

# 获取当前文件所在的绝对路径
current_file_path = os.path.abspath(__file__)

# 获取当前文件所在目录的路径
current_directory = os.path.dirname(current_file_path)

# 改变当前工作目录到当前文件所在的目录
os.chdir(current_directory)


# 播放器命令
player_cmd = 'aplay'  # Linux上的播放器命令

goal_w = 0 # 如果加点了之后就进行下个目标点，后面所有点位置加此权重

class_hash = {"teargas": 3, "spontoon": 1, "bulletproof_vest": 2, "first_aid_kit": 0} # 匹配kbfz数量和wp种类的哈希表

# B区域的变量
kbfz_num = 0 # 恐怖分子数量
cur_class = "" # 当前识别的种类
kbfz_flag = 2 # 判断是否进入恐怖分子识别区域 1是进入， 0是没有进入
kbfz_bbcs = 1 # 恐怖分子播报次数
kbfz_dec_size = 0 # 总识别次数
kbfz_dict = dict() # 识别的字典
kbfz_max_num = 0 # kbfz出现的最终结果

# 物品识别区域变量
wp_class = "" # 代表该区域识别到的种类
wp_list = []
wpbb_flag = False
turn_flag = False

# 图像宽度
image_width = 640



def run_roslaunch():
    subprocess.run(["roslaunch", "kcf_track", "kcf_track.launch"])


# 创建线程的函数
def threaded_play_audio(audio_path):
    thread = threading.Thread(target=play_audio, args=(audio_path,))
    thread.start()
    return thread

# 播放音频的函数
def play_audio(audio_path):
    os.system(f"{player_cmd} {audio_path}")

# 停止播放音频
def end_audio():
    os.system(f"pkill {player_cmd}")

def stop_audio():
    os.system("pkill -STOP -u your_username vlc")


def load_goal():
    global goal_list


    

# 小飞
# 点的列表
goal_list = []

def load_goal():
    global goal_list
    with open("points/goal_points.json", "r") as f:
         data = json.load(f)

    # 提取目标点并存储到列表中
    goal_list = data['goal_points']

goal_index = 0
def rotate_and_move(twist, target_angle_deg, rotation_time, pub):
    global goal_index
    global bb_msg
    # 设置旋转角度，将目标旋转角度转换为弧度
    target_angle_rad = target_angle_deg * 3.14159 / 180.0
    # 计算旋转速度
    angular_speed = target_angle_rad / rotation_time

    # 设置机器人的旋转速度
    twist.angular.z = angular_speed

    # 计算旋转的起始时间和发布频率
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)

    # 循环发布旋转命令，直到达到旋转时间
    while rospy.Time.now().to_sec() - t0 < rotation_time:
        pub.publish(twist)
        rate.sleep()

def wp_isbb_callback(msg):
    global wpbb_flag

    if str(msg.data) == "ok":
        wpbb_flag = True


    # 发布停止命令，使机器人停止旋转wp_class
def bb_msg_callback(msg):
    global kbfz_num
    global kbfz_flag
    global kbfz_dec_size
    global kbfz_max_num
    global kbfz_dict
    global cur_class
    global kbfz_bbcs
    global bb_msg
    global wp_class
    global goal_w
    global wp_list
    global goal_list
    global class_hash
    global wpbb_flag
    global turn_flag

    bb_msg = str(msg.data)

    if bb_msg not in "1234567890":
        wp_list = bb_msg.split(",")
        wp_class = wp_list[0]
        # print(f"收到信息：种类 {wp_class} 中心点 {wp_list[1]},{wp_list[2]}")

    else: 
        print(f"收到信息：{bb_msg}")
        kbfz_num = int(bb_msg)

        if not kbfz_dict.get(kbfz_num):
            kbfz_dict[kbfz_num] = 1
        else:
            kbfz_dict[kbfz_num] += 1




def callback(data):
    global goal_list
    global goal_index
    global bb_msg
    global kbfz_max_num
    global wp_class
    global goal_w
    global class_hash
    global wpbb_flag
    global turn_flag
    global image_width 

    if data.status.status == 3:
        if goal_index < len(goal_list):

            # 根据目标索引（goal_index）处理旋转和移动
            if goal_index == 2:
                ok_msg = String()
                ok_msg.data = "kbfz"
                
                time.sleep(2.0)
                ok_pub.publish(ok_msg)
                print(kbfz_dict)
                # 阻塞等待摄像头回传
                while(kbfz_dict == {}): continue
        
                # 对字典进行排序，按照值(value)排序，返回一个列表，列表中的元素是元组([(key, value), ...])
                kbfz_max_dict = sorted(kbfz_dict.items(), key=lambda x: x[1], reverse=True)
                print(kbfz_max_dict)
                kbfz_max_num = kbfz_max_dict[0][0] # 获取数量最大的键
                print(f"B区的kbfz识别数量 = {kbfz_dec_size}")
                print(kbfz_dict)
                #rotate_and_move(twist, -160, 1.2, pub)
                print("*"*5 + str(kbfz_max_num)  + "*"*5)
                print("*"*5 + str(kbfz_max_num)  + "*"*5)
                print("*"*5 + str(kbfz_max_num)  + "*"*5)
                print("*"*5 + str(kbfz_max_num) + "*"*5)
                print("*"*5 + str(kbfz_max_num)  + "*"*5)
                play_audio(f"voice_packge/恐怖分子数量播报/{kbfz_max_num}.wav")
                ok_msg = String()
                ok_msg.data = f"{kbfz_max_num}"
                ok_pub.publish(ok_msg)
                # rotate_and_move(twist, 45, 1.2, pub)
                # rotate_and_move(twist, -100, 1.2, pub)

            elif goal_index == 6:
                time.sleep(0.05)
                play_audio("voice_packge/物品取得播报/急救包.wav")
                #rotate_and_move(twist, -100, 1.2, pub)
                #rotate_and_move(twist, -100, 1.2, pub)
            elif goal_index == 7:

                gw_cs = 1
                # 旋转寻找目标逻辑
                while True:
                    # 寻找目标
                    while class_hash[wp_class] != kbfz_max_num:
                        twist.angular.z = -0.6
                        pub.publish(twist)
                        print(f"目标不在视野区域！开始寻找目标，当前转动速度为{twist.angular.z}")


                    # 判断是否为识别物体获取对应的参数
                    if class_hash[wp_class] == kbfz_max_num:
                        center_x = float(wp_list[1])
                        Max_rotation_speed = 0.75

                        # 让速度回归
                        if gw_cs == 1:
                            twist.angular.z = 0
                            pub.publish(twist)
                            print("找到目标！！！")
                            gw_cs = 0
             

                    if abs((float(image_width/2 - center_x))/(image_width/2)) > 0.1:
                        twist.angular.z = float(image_width/2 - center_x)/(image_width/2)
                        print(f"开始调整位置，当前转动速度为{twist.angular.z}")
                        pub.publish(twist)
                    else:
                        twist.angular.z = 0
                        pub.publish(twist)
                        # 找到目标
                        break
                    if twist.angular.z > Max_rotation_speed:
                        twist.angular.z = Max_rotation_speed
                        pub.publish(twist)
                    if twist.angular.z < -Max_rotation_speed:
                        twist.angular.z = -Max_rotation_speed
                        pub.publish(twist)
                
                # while class_hash[wp_class] != kbfz_max_num:
                #     twist.angular.z = -0.4
                #     pub.publish(twist)

                # twist.angular.z = 0
                # 使用线程运行roslaunch命令
                roslaunch_thread = threading.Thread(target=run_roslaunch)
                roslaunch_thread.start()

                while True:
                    if wpbb_flag:
                        break
                    
                # 播报逻辑
                if kbfz_max_num == 1:
                    play_audio("voice_packge/物品取得播报/警棍.wav")
                elif kbfz_max_num == 2:
                    play_audio("voice_packge/物品取得播报/防弹衣.wav")
                else:
                    play_audio("voice_packge/物品取得播报/催泪瓦斯.wav")


            elif goal_index == 12:
                time.sleep(0.1)
                print(f"wpbb_flag = {wpbb_flag},  wp_class = {wp_class}")
                play_audio("voice_packge/任务完成/任务完成.wav")
            
            

            goal = goal_list[goal_index]["coordinates"]
            mypose = PoseStamped()

            # 设置下一个目标点
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal[0]
            mypose.pose.position.y = goal[1]
            mypose.pose.position.z = goal[2]
            mypose.pose.orientation.x = goal[3]
            mypose.pose.orientation.y = goal[4]
            mypose.pose.orientation.z = goal[5]
            mypose.pose.orientation.w = goal[6]

            # 发布下一个目标点
            turtle_vel_pub.publish(mypose)
            goal_index += 1
        else:
            end_audio()
            rospy.loginfo("所有目标点均已到达！")
            return

# ... 余下的代码保持不变 ...


if __name__ == '__main__':

    # 创建订阅者订阅获取播报的信息
    rospy.Subscriber("/bb_msg", String, bb_msg_callback)
    # 创建订阅者订阅获取是否物品播报的信息
    rospy.Subscriber("/wp_isbb", String, wp_isbb_callback)

    # 创建一个发布器，用于发布机器人的移动命令
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # 创建一个判断是否剖播报的发布器，用于播报提醒
    ok_pub = rospy.Publisher('ok', String, queue_size=1)
    # 创建一个Twist消息实例
    twist = Twist()

    # 加载所有点
    load_goal()

    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')

    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)
    
    # 启动播报
    # play_audio("voice_packge/启动播报/原神.wav")
    # 启动音乐
    # thread = threaded_play_audio("/home/ucar/Desktop/bgm.wav")

    mypose = PoseStamped()

    # 设置下一个目标点
    mypose.header.frame_id = 'map'
    mypose.pose.position.x = goal_list[goal_index]["coordinates"][0]
    mypose.pose.position.y = goal_list[goal_index]["coordinates"][1]
    mypose.pose.position.z = goal_list[goal_index]["coordinates"][2]
    mypose.pose.orientation.x = goal_list[goal_index]["coordinates"][3]
    mypose.pose.orientation.y = goal_list[goal_index]["coordinates"][4]
    mypose.pose.orientation.z = goal_list[goal_index]["coordinates"][5]
    mypose.pose.orientation.w = goal_list[goal_index]["coordinates"][6]
    rospy.loginfo(f"正在前往下一个目标：{goal_list[goal_index]['description']}")
    turtle_vel_pub.publish(mypose)
    goal_index += 1
    rospy.spin()    
