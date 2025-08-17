#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <locale>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace cv;
using namespace std;

// 智能巡线区域处理系统
// 
// 功能特点：
// 1. 支持畸变校正和透视变换
// 2. 增强的HSV颜色空间线条检测 (远距离优化)
// 3. PID控制器巡线
// 4. 环岛检测与处理（直接通过模式）
// 5. 终点检测与自动停车
// 6. 优化的路径跟踪算法，适用于环岛赛道
// 7. 远距离线条检测与轨迹预测
//
// 环岛逻辑：
// - 检测进入环岛条件（黄色点数和宽度）
// - 采用直接通过策略
// - 检测退出环岛条件
//
// 注意：已移除直角转弯逻辑和绕一圈模式，专门针对直接通过环岛优化

// 图像处理类

class ImageProcessor {
public:
    ImageProcessor() : it_(nh_) {
        // 读取配置文件
        readParameters();

        // 初始化相机内参标志
        camera_info_received_ = false;
        enable_undistortion_ = true;
        camera_info_topic_ = "/ucar_camera/camera_info";  // 默认话题
        
        // 初始化透视变换相关
        enable_perspective_correction_ = false;
        perspective_initialized_ = false;
        
        // 尝试从配置文件加载相机内参
        // loadCameraParametersFromFile();
        
        // 打印相机校正状态
        if (img_debug_output_) {
            printCameraCalibrationStatus();
        }

        // 使用配置的话题名称订阅图像
        image_sub_ = it_.subscribe(image_topic_, 1, &ImageProcessor::imageCb, this);
        image_pub_ = it_.advertise("/img_follow", 1);
        
        // 订阅相机内参信息
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &ImageProcessor::cameraInfoCb, this);
    }

    virtual void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        Mat processed_image = processImage(cv_ptr->image);
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, processed_image).toImageMsg();
        image_pub_.publish(output_msg);
    }

    // 相机内参回调函数
    void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        if (!camera_info_received_) {
            // 提取相机内参矩阵
            camera_matrix_ = (Mat_<double>(3,3) << 
                msg->K[0], msg->K[1], msg->K[2],
                msg->K[3], msg->K[4], msg->K[5],
                msg->K[6], msg->K[7], msg->K[8]);
            
            // 提取畸变系数
            if (msg->D.size() >= 5) {
                dist_coeffs_ = (Mat_<double>(5,1) << 
                    msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4]);
            }
            
            // 图像尺寸
            image_size_ = Size(msg->width, msg->height);
            
            // 计算去畸变映射
            initUndistortRectifyMap(camera_matrix_, dist_coeffs_, Mat(), 
                                  camera_matrix_, image_size_, CV_16SC2, 
                                  map1_, map2_);
            
            camera_info_received_ = true;
            ROS_INFO("相机内参已接收并初始化");
        }
    }

    Mat processImage(const Mat& image) {
        Mat corrected_image = image.clone();
        
        // 如果已接收到相机内参且启用畸变校正，则进行畸变校正
        if (camera_info_received_ && enable_undistortion_) {
            remap(image, corrected_image, map1_, map2_, INTER_LINEAR);
            if (img_debug_output_) {
                printUTF8("已应用畸变校正，降低了图像畸变误差");
            }
        } else if (img_debug_output_) {
            printUTF8("未应用畸变校正 - 相机内参: " + 
                     string(camera_info_received_ ? "已接收" : "未接收") + 
                     ", 畸变校正: " + string(enable_undistortion_ ? "启用" : "禁用"));
        }
        
        // 如果启用透视变换校正
        Mat perspective_corrected = corrected_image.clone();
        if (enable_perspective_correction_) {
            if (!perspective_initialized_) {
                initializePerspectiveTransform(corrected_image.cols, corrected_image.rows);
            }
            warpPerspective(corrected_image, perspective_corrected, perspective_matrix_, 
                           Size(corrected_image.cols, corrected_image.rows));
            if (img_debug_output_) {
                printUTF8("已应用透视变换校正，获得鸟瞰图效果");
            }
        }

        // 将RGB图像转换到HSV色彩空间
        Mat imgHSV;
        cvtColor(perspective_corrected, imgHSV, COLOR_BGR2HSV);

        // 在HSV色彩空间中进行直方图均衡化
        vector<Mat> hsvSplit;
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, imgHSV);

        // 对图像进行阈值处理，仅保留HSV值在指定范围内的像素
        Mat grayImage;
        inRange(imgHSV, Scalar(LowH_, LowS_, LowV_), Scalar(HighH_, HighS_, HighV_), grayImage);

        // 应用额外的图像增强以保留远处的线条信息
        // 1. 对二值化图像进行膨胀操作，连接断开的远处线条
        Mat dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        dilate(grayImage, grayImage, dilate_element, Point(-1, -1), 1);
        
        // 进行开运算，去除小的噪点 - 使用较小的核保留远处细线
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));  // 减小核大小保留细节
        morphologyEx(grayImage, grayImage, MORPH_OPEN, element);

        // 进行闭运算，连接相邻的区域 - 使用稍大的核连接断开的线段
        Mat close_element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(grayImage, grayImage, MORPH_CLOSE, close_element); 
        
        // 创建BGR图像以画出黄色线
        Mat colorImage;
        cvtColor(grayImage, colorImage, COLOR_GRAY2BGR);

        // 一开始初始化像素矩阵，进行边界255处理
        for (int y = 0; y < grayImage.rows; y++) {
            // 底部1处理
            if (y == grayImage.rows - 1) {
                for (int x = 2; x < grayImage.cols - 2; x++) {
                    grayImage.at<uchar>(y, x) = 0;
                }
            }
            // 注意：移除了左右边界255处理，因为改进后的代码不再需要这个处理
        }

        // 记录中心点位置的x
        unordered_map<int, int> left_points;
        unordered_map<int, int> right_points;
        // 记录中心点位置的x，一直更新保持连贯性质
        int center_x = grayImage.cols / 2;

        // 考虑范围 - 大幅扩大处理区域，让黄线能画得更远
        int img_range = grayImage.rows / 6;  // 从图像上六分之一开始处理，覆盖更大范围

        // 后面方便计算center_x的中值
        int total_center_x = 0;
        // 补线长度
        int bx_size = 0;
        
        // 判断当前帧是否检测到路径端点
        bool is_end = false;

        for (int y = grayImage.rows - 1; y >= img_range; y--) 
        {
            int mid = center_x;
            int leftBoundary = -1;
            int rightBoundary = -1;

            // 使用局部变量保存 center_x
            int local_center_x = center_x;

            // 开始统计
            total_center_x += center_x;
            
            if (grayImage.at<uchar>(y, local_center_x) == 255) {
                if (img_debug_output_) printUTF8("检测到路径端点");
                is_end = true;
                break;  // 简化处理，直接跳出循环
            }

            // 从中心向左遍历找到左边界点 - 优化远处线条检测
            for (int x = mid; x >= 0; x--) {
                if (x + 1 < grayImage.cols && x - 1 >= 0) {
                    // 标准边界检测：白->白->黑
                    if (grayImage.at<uchar>(y, x) == 255 && grayImage.at<uchar>(y, x - 1) == 255 && grayImage.at<uchar>(y, x + 1) == 0) {
                        leftBoundary = x;
                        left_points[y] = x;
                        break;
                    }
                    // 远处细线检测：白->黑 (更宽松的条件)
                    else if (y < grayImage.rows / 2 && 
                             grayImage.at<uchar>(y, x) == 255 && 
                             grayImage.at<uchar>(y, x + 1) == 0) {
                        leftBoundary = x;
                        left_points[y] = x;
                        break;
                    }
                }
            }

            // 从中心向右遍历找到右边界点 - 优化远处线条检测
            for (int x = mid; x < grayImage.cols; x++) {
                if (x + 1 < grayImage.cols && x - 1 >= 0) {
                    // 标准边界检测：黑->白->白
                    if (grayImage.at<uchar>(y, x) == 255 && grayImage.at<uchar>(y, x + 1) == 255 && grayImage.at<uchar>(y, x - 1) == 0) {
                        rightBoundary = x;
                        right_points[y] = x;
                        break;
                    }
                    // 远处细线检测：黑->白 (更宽松的条件)
                    else if (y < grayImage.rows / 2 && 
                             grayImage.at<uchar>(y, x) == 255 && 
                             grayImage.at<uchar>(y, x - 1) == 0) {
                        rightBoundary = x;
                        right_points[y] = x;
                        break;
                    }
                }
            }

            bool hasLeft = left_points.find(y) != left_points.end();
            bool hasRight = right_points.find(y) != right_points.end();
            if (hasLeft && hasRight) {
                center_x = (leftBoundary + rightBoundary) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else if (!hasLeft && hasRight) {
                center_x = (rightBoundary + 1) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else if (hasLeft && !hasRight) {
                center_x = (leftBoundary + grayImage.cols - 2) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else {
                // 两边都不存在，补线
                center_x = (grayImage.cols - 1) / 2;
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
                bx_size ++;
            }

            // 扩展中线
            for (int x = center_x, i = x - 1, j = x + 1; i >= center_x - center_p_ && j <= center_x + center_p_; i--, j++) {
                if (i >= 0 && i < colorImage.cols) {
                    colorImage.at<Vec3b>(y, i) = Vec3b(0, 255, 255); // 使用黄色
                }
                if (j >= 0 && j < colorImage.cols) {
                    colorImage.at<Vec3b>(y, j) = Vec3b(0, 255, 255); // 使用黄色
                }
            }
        }
        if (img_debug_output_) {
            printUTF8("left_size：" + to_string(left_points.size()) + " right_size：" + to_string(right_points.size()) + " is_stop：" + to_string(is_stop));
        }
        
        // 优化的终点检测逻辑：适合环岛赛道
        // 当检测到路径端点且左右边界点都很少时，认为到达终点
        if (is_end && left_points.size() <= end_dist_ && right_points.size() <= end_dist_) {
            is_stop = true;
            if (img_debug_output_) {
                printUTF8("检测到终点：路径结束且边界点不足");
            }
        }

        return colorImage;
    }

protected:

    // 用于打印
    void printUTF8(const string& str) {
        // 设置输出流的locale为UTF-8
        try {
            cout.imbue(locale("C.UTF-8"));
        } catch (const exception& e) {
            // 如果C.UTF-8不可用，尝试其他UTF-8 locale
            try {
                cout.imbue(locale("en_US.UTF-8"));
            } catch (const exception& e2) {
                // 使用默认locale
                cout.imbue(locale(""));
            }
        }
        cout << str << endl;
    }

    // 判定是否停止
    bool is_stop = false;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    // 相机内参相关
    ros::Subscriber camera_info_sub_;
    bool camera_info_received_;
    bool enable_undistortion_;
    Mat camera_matrix_;
    Mat dist_coeffs_;
    Size image_size_;
    Mat map1_, map2_;
    string camera_info_topic_;
    
    // 透视变换相关
    bool enable_perspective_correction_;
    Mat perspective_matrix_;
    bool perspective_initialized_;

    // 配置参数
    int range_y_;
    int center_p_;
    int boundary_check_width_;
    bool img_debug_output_;
    int end_dist_;
    string image_topic_;
    int LowH_;
    int HighH_;
    int LowS_;
    int HighS_;
    int LowV_;
    int HighV_;

    void readParameters() {
        ifstream file("/home/ucar/Desktop/ucar/src/line_follower2/config/image.cfg");
        if (!file.is_open()) {
            ROS_ERROR("无法打开图像处理配置文件");
            return;
        }

        string line;
        while (getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            istringstream iss(line);
            string key;
            string value;

            if (getline(iss, key, '=') && getline(iss, value)) {
                // 去除可能的尾随注释
                size_t commentPos = value.find('#');
                if (commentPos != string::npos) {
                    value = value.substr(0, commentPos);
                }

                // 去除首尾空白
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                // 参数赋值
                try {
                    if (key == "range_y") range_y_ = stoi(value);
                    else if (key == "center_p") center_p_ = stoi(value);
                    else if (key == "boundary_check_width") boundary_check_width_ = stoi(value);
                    else if (key == "img_debug_output") img_debug_output_ = (value == "true" || value == "1");
                    else if (key == "image_topic") image_topic_ = value;
                    else if (key == "end_dist") end_dist_ = stoi(value);
                    else if (key == "LowH") LowH_ = stoi(value);
                    else if (key == "HighH") HighH_ = stoi(value);
                    else if (key == "LowS") LowS_ = stoi(value);
                    else if (key == "HighS") HighS_ = stoi(value);
                    else if (key == "LowV") LowV_ = stoi(value);
                    else if (key == "HighV") HighV_ = stoi(value);
                    else {
                        ROS_WARN_STREAM("未知参数: " << key << " = " << value);
                    }
                }
                catch (const exception& e) {
                    ROS_ERROR_STREAM("解析参数 " << key << " 时出错: " << e.what());
                }
            }
        }
    }
    
    // 初始化透视变换矩阵（鸟瞰图变换）
    void initializePerspectiveTransform(int img_width, int img_height) {
        if (perspective_initialized_) return;
        
        // 定义透视变换的源点和目标点
        // 这些点需要根据实际相机安装位置和角度进行调整
        vector<Point2f> src_points = {
            Point2f(img_width * 0.1, img_height * 0.6),   // 左上
            Point2f(img_width * 0.9, img_height * 0.6),   // 右上
            Point2f(img_width * 0.0, img_height * 1.0),   // 左下
            Point2f(img_width * 1.0, img_height * 1.0)    // 右下
        };
        
        vector<Point2f> dst_points = {
            Point2f(img_width * 0.2, 0),                  // 左上
            Point2f(img_width * 0.8, 0),                  // 右上
            Point2f(img_width * 0.2, img_height),         // 左下
            Point2f(img_width * 0.8, img_height)          // 右下
        };
        
        perspective_matrix_ = getPerspectiveTransform(src_points, dst_points);
        perspective_initialized_ = true;
        
        if (img_debug_output_) {
            printUTF8("透视变换矩阵已初始化");
        }
    }
    
    // 打印相机校正状态
    void printCameraCalibrationStatus() {
        printUTF8("============================相机校正状态============================");
        printUTF8("相机内参状态: " + string(camera_info_received_ ? "已加载" : "未加载"));
        printUTF8("畸变校正: " + string(enable_undistortion_ ? "启用" : "禁用"));
        printUTF8("透视变换: " + string(enable_perspective_correction_ ? "启用" : "禁用"));
        printUTF8("相机信息话题: " + camera_info_topic_);
        if (camera_info_received_) {
            printUTF8("图像尺寸: " + to_string(image_size_.width) + "x" + to_string(image_size_.height));
        }
        printUTF8("================================================================");
    }
};

// PID速度控制器 巡线类

class LineFollower : public ImageProcessor {
public:
    LineFollower() : ImageProcessor() {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        // 设置locale支持UTF-8，使用更健壮的方式
        try {
            locale::global(locale("C.UTF-8"));
        } catch (const exception& e) {
            try {
                locale::global(locale("en_US.UTF-8"));
            } catch (const exception& e2) {
                try {
                    locale::global(locale(""));
                } catch (const exception& e3) {
                    ROS_WARN("无法设置UTF-8 locale，中文显示可能有问题");
                }
            }
        }

        // 读取参数
        readParameters();

        // 初始化自适应PID参数
        adaptive_Kp_ = Kp_;
        adaptive_Ki_ = Ki_;
        adaptive_Kd_ = Kd_;

        // 打印读取的参数
        printParameters();
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) override {
        ImageProcessor::imageCb(msg);
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        Mat processed_image = processImage(cv_ptr->image);
        followLine(processed_image);
    }

private:
    ros::Publisher cmd_vel_pub_;
    double max_linear_speed_;
    double max_angular_speed_;
    double Kp_, Ki_, Kd_;
    double adaptive_Kp_, adaptive_Ki_, adaptive_Kd_;
    double deadzone_;
    double integral_limit_;  // 积分限幅
    double error_threshold_;  // 积分分离阈值
    double soft_limit_lower_, soft_limit_upper_;  // 软限幅范围
    double filter_coefficient_;  // 输入滤波系数
    bool pid_debug_output_;
    string end_audio_msg_;      // 结束播报
    
    // 环岛状态枚举
    enum RoundaboutState {
        NOT_IN_ROUNDABOUT = 0,
        ENTERING = 1,
        STRAIGHT_PHASE = 2,  // 新增：先直行阶段
        TURNING_PHASE = 3,   // 新增：转向阶段
        EXITING = 4
    };
    
    // 环岛相关变量
    RoundaboutState roundabout_state_ = NOT_IN_ROUNDABOUT;
    int roundabout_counter_ = 0; // 防抖用
    int straight_phase_counter_ = 0; // 直行阶段计数器
    
    // 环岛配置参数
    double roundabout_turn_bias_ = 20.0; // 环岛内的转向偏移量
    int straight_phase_duration_ = 15;   // 直行阶段持续帧数
    double gradual_turn_factor_ = 0.3;   // 渐进转向系数
    
    const int roundabout_entry_threshold_ = 780;  // 进入环岛需要黄色点数目大于780
    const int roundabout_exit_threshold_ = 850;   // 离开环岛时，黄色点数目阈值
    const int roundabout_min_width_ = 160;        // 进入环岛时宽度阈值
    const int roundabout_exit_min_width_ = 220;   // 离开环岛时宽度阈值


    // 读取参数配置
    void readParameters() {
        ifstream file("/home/ucar/Desktop/ucar/src/line_follower2/config/pid.cfg");
        if (!file.is_open()) {
            ROS_ERROR("无法打开PID配置文件");
            return;
        }

        string line;
        while (getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            istringstream iss(line);
            string key;
            string value;

            if (getline(iss, key, '=') && getline(iss, value)) {
                // 去除可能的尾随注释
                size_t commentPos = value.find('#');
                if (commentPos != string::npos) {
                    value = value.substr(0, commentPos);
                }

                // 去除首尾空白
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                // 参数赋值
                try {
                    if (key == "max_linear_speed") max_linear_speed_ = stod(value);
                    else if (key == "max_angular_speed") max_angular_speed_ = stod(value);
                    else if (key == "Kp") Kp_ = stod(value);
                    else if (key == "Ki") Ki_ = stod(value);
                    else if (key == "Kd") Kd_ = stod(value);
                    else if (key == "adaptive_Kp") adaptive_Kp_ = stod(value);
                    else if (key == "adaptive_Ki") adaptive_Ki_ = stod(value);
                    else if (key == "adaptive_Kd") adaptive_Kd_ = stod(value);
                    else if (key == "deadzone") deadzone_ = stod(value);
                    else if (key == "integral_limit") integral_limit_ = stod(value);
                    else if (key == "error_threshold") error_threshold_ = stod(value);
                    else if (key == "soft_limit_lower") soft_limit_lower_ = stod(value);
                    else if (key == "soft_limit_upper") soft_limit_upper_ = stod(value);
                    else if (key == "filter_coefficient") filter_coefficient_ = stod(value);
                    else if (key == "pid_debug_output") pid_debug_output_ = (value == "true" || value == "1");
                    else if (key == "end_audio_msg") end_audio_msg_= value;
                    else if (key == "roundabout_turn_bias") roundabout_turn_bias_ = stod(value);
                    else if (key == "straight_phase_duration") straight_phase_duration_ = stoi(value);
                    else if (key == "gradual_turn_factor") gradual_turn_factor_ = stod(value);
                    else {
                        ROS_WARN_STREAM("未知参数: " << key << " = " << value);
                    }
                } 
                catch (const exception& e) {
                    ROS_ERROR_STREAM("解析参数 " << key << " 时出错: " << e.what());
                }
            }
        }
      }

    void printParameters() {
        
        printUTF8("============================配置参数相关信息============================");
        printUTF8("最大线速度: " + to_string(max_linear_speed_) + " m/s");
        printUTF8("最大角速度: " + to_string(max_angular_speed_) + " rad/s");
        printUTF8("Kp: " + to_string(Kp_));
        printUTF8("Ki: " + to_string(Ki_));
        printUTF8("Kd: " + to_string(Kd_));
        printUTF8("死区: " + to_string(deadzone_) + " 像素");
        printUTF8("积分限幅: " + to_string(integral_limit_));
        printUTF8("积分分离阈值: " + to_string(error_threshold_));
        printUTF8("软限幅下限: " + to_string(soft_limit_lower_));
        printUTF8("软限幅上限: " + to_string(soft_limit_upper_));
        printUTF8("输入滤波系数: " + to_string(filter_coefficient_));
        printUTF8("PID调试输出: " + string(pid_debug_output_ ? "开启" : "关闭"));
        printUTF8("环岛转向偏移: " + to_string(roundabout_turn_bias_));
        printUTF8("直行阶段持续帧数: " + to_string(straight_phase_duration_));
        printUTF8("渐进转向系数: " + to_string(gradual_turn_factor_));
        printUTF8("环岛进入阈值: 点数>" + to_string(roundabout_entry_threshold_) + ", 宽度>" + to_string(roundabout_min_width_));
        printUTF8("环岛退出阈值: 点数<" + to_string(roundabout_exit_threshold_) + ", 宽度<" + to_string(roundabout_exit_min_width_));
        printUTF8("============================图像配置参数相关信息============================");
        printUTF8("HSV配置: (" + to_string(LowH_) + ", " + to_string(LowS_) + ", " + to_string(LowV_) + "), (" + to_string(HighH_) + ", " + to_string(HighS_) + ", " + to_string(HighV_) + ")");
        printUTF8("图像垂直分割比例: 1/" + to_string(range_y_));
        printUTF8("中心线扩展参数: " + to_string(center_p_));
        printUTF8("边界检测宽度: " + to_string(boundary_check_width_));
        printUTF8("图像调试输出: " + string(img_debug_output_ ? "开启" : "关闭"));
        printUTF8("订阅的图像话题名称: " + image_topic_);
        printUTF8("终点判停阈值: " + to_string(end_dist_));
        printUTF8("==========================================================================");
    }

   void followLine(const Mat& processedImage) {
    int height = processedImage.rows;
    int width = processedImage.cols;
    int startRow = 2 * height / 3;
    vector<Point> yellowPoints;
    int minX = width, maxX = 0;
    // 收集底部区域的黄色点
    for (int y = startRow; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3b pixel = processedImage.at<Vec3b>(y, x);
            if (pixel[0] == 0 && pixel[1] == 255 && pixel[2] == 255) {
                yellowPoints.push_back(Point(x, y));
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
            }
        }
    }
    int yellowCount = yellowPoints.size();
    int yellowWidth = (yellowCount > 0) ? (maxX - minX) : 0;
    
    // 计算黄色点的平均X坐标（用于位置跟踪）
    int sumX = 0;
    for (const auto& point : yellowPoints) {
        sumX += point.x;
    }
    int avgX = yellowPoints.empty() ? width / 2 : sumX / yellowPoints.size();

    std::ostringstream oss;
    oss << "yellowCount:" << yellowCount << ", yellowWidth:" << yellowWidth;
    printUTF8(oss.str());

    // 环岛检测与处理（简化版，仅支持直接通过）
    if (roundabout_state_ == NOT_IN_ROUNDABOUT) {
        // 检测是否进入环岛
        if (yellowCount > roundabout_entry_threshold_ && yellowWidth > roundabout_min_width_) {
            roundabout_counter_++;
            if (roundabout_counter_ > 3) {
                roundabout_state_ = ENTERING;
                
                if (pid_debug_output_) {
                    printUTF8("检测到进入环岛，采用直接通过模式");
                }
            }
        } else {
            roundabout_counter_ = 0;
        }
    }
    else {
        // 在环岛内的状态处理
        switch(roundabout_state_) {
            case ENTERING:
                // 进入环岛后，先进入直行阶段
                roundabout_state_ = STRAIGHT_PHASE;
                straight_phase_counter_ = 0;
                if (pid_debug_output_) {
                    printUTF8("进入环岛，开始直行阶段");
                }
                break;
                
            case STRAIGHT_PHASE:
                // 直行阶段，让车辆先往前走一段距离
                straight_phase_counter_++;
                if (straight_phase_counter_ >= straight_phase_duration_) {
                    roundabout_state_ = TURNING_PHASE;
                    if (pid_debug_output_) {
                        printUTF8("直行阶段完成，开始转向阶段");
                    }
                }
                break;
                
            case TURNING_PHASE:
                // 转向阶段，检测退出条件
                if (yellowCount < roundabout_exit_threshold_ || yellowWidth < roundabout_exit_min_width_) {
                    roundabout_counter_++;
                    if (roundabout_counter_ > 3) {
                        roundabout_state_ = EXITING;
                        if (pid_debug_output_) {
                            printUTF8("转向阶段完成，准备退出环岛");
                        }
                    }
                } else {
                    roundabout_counter_ = 0;
                }
                break;
                
            case EXITING:
                // 退出环岛
                roundabout_state_ = NOT_IN_ROUNDABOUT;
                roundabout_counter_ = 0;
                straight_phase_counter_ = 0;
                if (pid_debug_output_) {
                    printUTF8("成功退出环岛，恢复正常巡线");
                }
                break;
        }
    }

    if (is_stop) {
        stop();
        return;
    }

    geometry_msgs::Twist twist;

    if (roundabout_state_ != NOT_IN_ROUNDABOUT) {
        // 在环岛内，采用渐进式转向策略
        geometry_msgs::Twist twist;
        
        if (roundabout_state_ == STRAIGHT_PHASE) {
            // 直行阶段：主要保持直行，微调避免偏离
            int centerX = width / 2;
            int error = avgX - centerX;
            
            // 使用较小的控制增益，主要保持直行
            double straight_Kp = Kp_ * 0.3;
            double steeringAngle = straight_Kp * error;
            
            // 限制转向角度，确保主要是直行
            steeringAngle = max(-max_angular_speed_ * 0.3, min(max_angular_speed_ * 0.3, steeringAngle));
            
            twist.linear.x = max_linear_speed_ * 0.8;  // 保持较好的前进速度
            twist.angular.z = -steeringAngle;
            
            if (pid_debug_output_) {
                printUTF8("环岛直行阶段: 计数器=" + to_string(straight_phase_counter_) + 
                         "/" + to_string(straight_phase_duration_) + ", 微调角度=" + to_string(steeringAngle));
            }
        }
        else if (roundabout_state_ == TURNING_PHASE || roundabout_state_ == EXITING) {
            // 转向阶段：渐进式转向
            double turn_bias = roundabout_turn_bias_;
            double speed_factor = 0.7;
            
            // 根据阶段调整转向强度
            double turn_intensity = (roundabout_state_ == TURNING_PHASE) ? gradual_turn_factor_ : gradual_turn_factor_ * 0.7;
            
            int desired_middle = width / 2 - turn_bias * turn_intensity;
            int error = avgX - desired_middle;

            // PID控制参数
            double roundabout_Kp = Kp_ * 0.8 * turn_intensity;
            double steeringAngle = roundabout_Kp * error;

            // 限制角速度
            double max_angular = max_angular_speed_ * (0.6 + 0.4 * turn_intensity);
            steeringAngle = max(-max_angular, min(max_angular, steeringAngle));

            twist.linear.x = max_linear_speed_ * speed_factor;
            twist.angular.z = -steeringAngle;
            
            if (pid_debug_output_) {
                string direction = (abs(steeringAngle) < 0.01) ? "直行" : (steeringAngle > 0 ? "左转" : "右转");
                printUTF8("环岛转向 [" + getRoundaboutStateName(roundabout_state_) + 
                         "]: " + direction + ", 转向强度=" + to_string(turn_intensity) + 
                         ", 误差=" + to_string(error));
            }
        }
        else {
            // ENTERING 状态，保持当前巡线控制
            int centerX = width / 2;
            int error = avgX - centerX;
            double steeringAngle = Kp_ * 0.5 * error;
            steeringAngle = max(-max_angular_speed_ * 0.5, min(max_angular_speed_ * 0.5, steeringAngle));
            
            twist.linear.x = max_linear_speed_ * 0.6;
            twist.angular.z = -steeringAngle;
            
            if (pid_debug_output_) {
                printUTF8("环岛进入阶段: 准备进入环岛");
            }
        }

        cmd_vel_pub_.publish(twist);
        return;
    }

    // 非环岛时正常巡线控制
    int centerX = width / 2;
    int error = avgX - centerX;

    static int lastError = 0;
    static double integral = 0;
    static double lastFilteredError = 0;

    double filteredError = filter_coefficient_ * error + (1 - filter_coefficient_) * lastFilteredError;
    lastFilteredError = filteredError;

    if (abs(filteredError) < deadzone_) filteredError = 0;
    else filteredError = (filteredError > 0) ? filteredError - deadzone_ : filteredError + deadzone_;

    adaptPIDParameters(filteredError);

    if (abs(filteredError) < error_threshold_) integral += filteredError;
    else integral = 0;

    integral = max(-integral_limit_, min(integral_limit_, integral));
    int derivative = filteredError - lastError;

    double steeringAngle = adaptive_Kp_ * filteredError + adaptive_Ki_ * integral + adaptive_Kd_ * derivative;
    steeringAngle = max(soft_limit_lower_, min(soft_limit_upper_, steeringAngle));

    twist.linear.x = max_linear_speed_;
    twist.angular.z = -steeringAngle;
    twist.angular.z = max(-max_angular_speed_, min(max_angular_speed_, twist.angular.z));

    cmd_vel_pub_.publish(twist);

    if (pid_debug_output_) {
        printSpeedInfo(twist.linear.x, twist.angular.z);
    }

    lastError = filteredError;
}


    void adaptPIDParameters(double error) {
        // 简单的自适应策略示例
        if (abs(error) > error_threshold_ * 2) {
            adaptive_Kp_ = Kp_ * 1.5;
            adaptive_Ki_ = Ki_ * 0.5;
            adaptive_Kd_ = Kd_ * 2;
        } else {
            adaptive_Kp_ = Kp_;
            adaptive_Ki_ = Ki_;
            adaptive_Kd_ = Kd_;
        }
    }

    // 获取环岛状态名称
    string getRoundaboutStateName(RoundaboutState state) {
        switch(state) {
            case NOT_IN_ROUNDABOUT: return "未在环岛";
            case ENTERING: return "进入环岛";
            case STRAIGHT_PHASE: return "直行阶段";
            case TURNING_PHASE: return "转向阶段";
            case EXITING: return "退出环岛";
            default: return "未知状态";
        }
    }

    void stop() {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmd_vel_pub_.publish(twist);
        if (pid_debug_output_) printUTF8("停止");
        if (system(("aplay " + end_audio_msg_).c_str()) == -1) {
        ROS_WARN("Failed to execute audio command");
        }
        exit(0);
    }

    void printSpeedInfo(double linear_speed, double angular_speed) {
        string direction = (abs(angular_speed) < 0.01) ? "直行" : (angular_speed > 0 ? "左转" : "右转");
        ostringstream oss;
        oss << "线速度: " << linear_speed << " m/s, 角速度: " << angular_speed 
            << " rad/s, 检测到: " << direction;
        printUTF8(oss.str());
    }
};

int main(int argc, char** argv) {
    // 设置环境变量以支持UTF-8
    setenv("LC_ALL", "C.UTF-8", 1);
    setenv("LANG", "C.UTF-8", 1);
    
    ros::init(argc, argv, "line_follower");
    LineFollower lf;
    ros::spin();
    return 0;
}