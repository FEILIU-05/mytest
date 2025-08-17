#include <ros/ros.h>                      // 引入ROS的头文件
#include <image_transport/image_transport.h> // 引入图像传输模块的头文件
#include <cv_bridge/cv_bridge.h>           // 引入CV桥接模块的头文件
#include <sensor_msgs/image_encodings.h>   // 引入传感器消息的图像编码头文件
#include <sensor_msgs/CameraInfo.h>        // 引入相机信息消息头文件
#include <opencv2/imgproc/imgproc.hpp>     // 引入OpenCV图像处理模块的头文件
#include <opencv2/highgui/highgui.hpp>     // 引入OpenCV图形用户界面模块的头文件
#include <opencv2/calib3d/calib3d.hpp>     // 引入OpenCV相机标定模块的头文件
#include <dynamic_reconfigure/server.h>    // 引入动态重配置服务器的头文件
#include <line_follower2/HSVThresholdConfig.h> // 引入自定义的HSV阈值配置文件
#include <std_srvs/Empty.h>                // 引入空服务消息
#include <fstream>                         // 文件读取
#include <sstream>                         // 字符串流
#include <locale>                          // UTF-8 locale支持
#include <cstdlib>                         // setenv函数
#include <unordered_map>                   // 哈希映射容器

using namespace cv;                        // 使用OpenCV命名空间
using namespace std;                       // 使用标准命名空间

// 用于打印UTF-8中文的函数
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

// HSV色彩空间中的低高阈值 - 与line_follower2_node同步
static int iLowH = 0;
static int iHighH = 150;
static int iLowS = 0; 
static int iHighS = 100;
static int iLowV = 30;  // 修正为与配置文件一致
static int iHighV = 255;

// 配置参数 - 与line_follower2_node保持一致
static string image_topic = "/ucar_camera/image_raw";
static string camera_info_topic = "/ucar_camera/camera_info";
static bool img_debug_output = true;

// 路径计算参数
static int range_y = 3;                    // 图像垂直分割比例
static int center_p = 2;                   // 中心线扩展参数

// 相机标定参数
static bool camera_info_received = false;
static Mat camera_matrix = Mat::eye(3, 3, CV_64F);
static Mat dist_coeffs = Mat::zeros(1, 5, CV_64F);
static Mat map1, map2;  // 畸变校正映射表

// 读取配置文件
void readImageParameters() {
    ifstream file("/home/ucar/Desktop/ucar/src/line_follower2/config/image.cfg");
    if (!file.is_open()) {
        ROS_WARN("无法打开图像处理配置文件，使用默认参数");
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
                if (key == "LowH") iLowH = stoi(value);
                else if (key == "HighH") iHighH = stoi(value);
                else if (key == "LowS") iLowS = stoi(value);
                else if (key == "HighS") iHighS = stoi(value);
                else if (key == "LowV") iLowV = stoi(value);
                else if (key == "HighV") iHighV = stoi(value);
                else if (key == "image_topic") image_topic = value;
                else if (key == "camera_info_topic") camera_info_topic = value;
                else if (key == "img_debug_output") img_debug_output = (value == "true" || value == "1");
                else if (key == "range_y") range_y = stoi(value);
                else if (key == "center_p") center_p = stoi(value);
            }
            catch (const exception& e) {
                ROS_ERROR_STREAM("解析参数 " << key << " 时出错: " << e.what());
            }
        }
    }
    
    if (img_debug_output) {
        printUTF8("HSV参数加载完成: H(" + to_string(iLowH) + "," + to_string(iHighH) + 
                 ") S(" + to_string(iLowS) + "," + to_string(iHighS) + 
                 ") V(" + to_string(iLowV) + "," + to_string(iHighV) + ")");
        printUTF8("图像话题: " + image_topic);
        printUTF8("相机信息话题: " + camera_info_topic);
        printUTF8("从配置文件读取的参数已保存到静态变量");
    }
}

// 图像发布器
image_transport::Publisher rgb_pub;
image_transport::Publisher hsv_pub;
image_transport::Publisher result_pub;
image_transport::Publisher binary_pub;  // 新增二值化图像发布器

// 动态重配置服务器指针
dynamic_reconfigure::Server<line_follower2::HSVThresholdConfig>* config_server = nullptr;

// 重新加载配置文件的服务回调函数
bool reloadConfigCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    printUTF8("正在重新加载配置文件...");
    readImageParameters();
    
    if (config_server) {
        // 更新动态重配置服务器的参数
        line_follower2::HSVThresholdConfig config;
        config.low_H = iLowH;
        config.high_H = iHighH;
        config.low_S = iLowS;
        config.high_S = iHighS;
        config.low_V = iLowV;
        config.high_V = iHighV;
        config_server->updateConfig(config);
        printUTF8("配置文件重新加载完成！");
    }
    return true;
}

// 动态重配置回调函数
void dynamicReconfigureCallback(line_follower2::HSVThresholdConfig &config, uint32_t level) {
    // 添加日志显示动态重配置的值
    if (img_debug_output) {
        printUTF8("动态重配置回调被调用，传入参数: H(" + to_string(config.low_H) + "," + to_string(config.high_H) + 
                 ") S(" + to_string(config.low_S) + "," + to_string(config.high_S) + 
                 ") V(" + to_string(config.low_V) + "," + to_string(config.high_V) + ")");
    }
    
    iLowH = config.low_H;
    iHighH = config.high_H;
    iLowS = config.low_S;
    iHighS = config.high_S;
    iLowV = config.low_V;
    iHighV = config.high_V;
    
    if (img_debug_output) {
        printUTF8("动态重配置更新HSV参数: H(" + to_string(iLowH) + "," + to_string(iHighH) + 
                 ") S(" + to_string(iLowS) + "," + to_string(iHighS) + 
                 ") V(" + to_string(iLowV) + "," + to_string(iHighV) + ")");
    }
}

// 相机信息回调函数
void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (!camera_info_received) {
        // 提取相机内参矩阵
        camera_matrix.at<double>(0,0) = msg->K[0]; // fx
        camera_matrix.at<double>(0,1) = msg->K[1]; // skew
        camera_matrix.at<double>(0,2) = msg->K[2]; // cx
        camera_matrix.at<double>(1,0) = msg->K[3]; // 0
        camera_matrix.at<double>(1,1) = msg->K[4]; // fy
        camera_matrix.at<double>(1,2) = msg->K[5]; // cy
        camera_matrix.at<double>(2,0) = msg->K[6]; // 0
        camera_matrix.at<double>(2,1) = msg->K[7]; // 0
        camera_matrix.at<double>(2,2) = msg->K[8]; // 1

        // 提取畸变系数
        if (msg->D.size() >= 5) {
            for (int i = 0; i < 5; i++) {
                dist_coeffs.at<double>(0, i) = msg->D[i];
            }
        }

        // 计算畸变校正映射表
        Size imageSize(msg->width, msg->height);
        initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(), camera_matrix, 
                               imageSize, CV_16SC2, map1, map2);

        camera_info_received = true;
        
        if (img_debug_output) {
            printUTF8("相机标定信息已接收:");
            printUTF8("  图像尺寸: " + to_string(msg->width) + "x" + to_string(msg->height));
            printUTF8("  内参 fx: " + to_string(msg->K[0]) + ", fy: " + to_string(msg->K[4]));
            printUTF8("  中心点 cx: " + to_string(msg->K[2]) + ", cy: " + to_string(msg->K[5]));
            printUTF8("  畸变系数: [" + to_string(msg->D[0]) + ", " + to_string(msg->D[1]) + ", " + 
                     to_string(msg->D[2]) + ", " + to_string(msg->D[3]) + ", " + to_string(msg->D[4]) + "]");
            printUTF8("畸变校正映射表已生成，开始使用相机标定优化图像");
        }
    }
}

// 图像回调函数 - 与line_follower2_node同步处理逻辑
void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    // 每次处理图像时强制使用配置文件的参数（临时解决动态重配置覆盖问题）
    static bool first_call = true;
    static int frame_count = 0;
    frame_count++;
    
    // 每100帧重新读取一次配置文件参数，确保使用正确的值
    if (first_call || frame_count % 100 == 0) {
        ifstream file("/home/ucar/Desktop/ucar/src/line_follower2/config/image.cfg");
        if (file.is_open()) {
            string line;
            while (getline(file, line)) {
                if (line.empty() || line[0] == '#') continue;
                
                istringstream iss(line);
                string key, value;
                if (getline(iss, key, '=') && getline(iss, value)) {
                    size_t commentPos = value.find('#');
                    if (commentPos != string::npos) value = value.substr(0, commentPos);
                    
                    key.erase(0, key.find_first_not_of(" \t"));
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    
                    try {
                        if (key == "LowH") iLowH = stoi(value);
                        else if (key == "HighH") iHighH = stoi(value);
                        else if (key == "LowS") iLowS = stoi(value);
                        else if (key == "HighS") iHighS = stoi(value);
                        else if (key == "LowV") iLowV = stoi(value);
                        else if (key == "HighV") iHighV = stoi(value);
                    } catch (const exception& e) {}
                }
            }
            first_call = false;
        }
    }
    
    // 创建一个cv_bridge的CvImage指针
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // 将ROS的图像消息转换为OpenCV的图像格式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        // 捕获并打印异常
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 获取原始图像
    Mat imgOriginal = cv_ptr->image;
    
    // ============ 相机标定和畸变校正 ============
    Mat processedImage;
    if (camera_info_received && !map1.empty() && !map2.empty()) {
        // 使用相机标定信息进行畸变校正
        remap(imgOriginal, processedImage, map1, map2, INTER_LINEAR);
        
    } else {
        // 没有相机标定信息，直接使用原始图像
        processedImage = imgOriginal.clone();
        
        if (img_debug_output && frame_count % 200 == 0) {
            printUTF8("等待相机标定信息，使用原始图像进行处理");
        }
    }
    
    // 注意：这里简化处理，如果需要畸变校正和透视变换，
    // 可以根据line_follower2_node的设置进行相应添加
    // 目前保持简单处理，但添加了注释说明
    
    // 将RGB图像转换到HSV色彩空间
    Mat imgHSV;
    cvtColor(processedImage, imgHSV, COLOR_BGR2HSV);

    // 在HSV色彩空间中进行直方图均衡化 - 与line_follower2_node一致
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);

    // 对图像进行阈值处理，仅保留HSV值在指定范围内的像素
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

    // 应用额外的图像增强以保留远处的线条信息
    // 1. 对二值化图像进行膨胀操作，连接断开的远处线条
    Mat dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    dilate(imgThresholded, imgThresholded, dilate_element, Point(-1, -1), 1); 

    // 去掉图像的上四分之一，允许更远的视野进行线条检测
    int cropStartY = imgThresholded.rows / 4;  // 改为从四分之一处开始，保留更多远景
    Rect cropRegion(0, cropStartY, imgThresholded.cols, imgThresholded.rows - cropStartY);
    Mat imgCropped = imgThresholded(cropRegion).clone();
    
    // 创建一个与原图像大小相同的图像，上四分之一为黑色
    Mat imgFinal = Mat::zeros(imgThresholded.size(), imgThresholded.type());
    imgCropped.copyTo(imgFinal(cropRegion));
    
    // 使用裁剪后的图像继续处理
    imgThresholded = imgFinal; 

    // 进行开运算，去除小的噪点 - 使用较小的核保留远处细线
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));  // 减小核大小保留细节
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    // 进行闭运算，连接相邻的区域 - 使用稍大的核连接断开的线段
    Mat close_element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, close_element);

    // 创建彩色输出图像用于显示处理结果
    Mat colorResult;
    cvtColor(imgThresholded, colorResult, COLOR_GRAY2BGR);

    // 在图像上画一条红线标记裁剪分界线
    int cropLine = imgThresholded.rows / 4;  // 更新为四分之一
    line(colorResult, Point(0, cropLine), Point(imgThresholded.cols, cropLine), Scalar(0, 0, 255), 2);

    // ============ 路径计算和轨迹绘制 (与line_follower2_node完全同步) ============
    // 一开始初始化像素矩阵，进行边界255处理（与line_follower2_node一致）
    for (int y = 0; y < imgThresholded.rows; y++) {
        // 底部1处理
        if (y == imgThresholded.rows - 1) {
            for (int x = 2; x < imgThresholded.cols - 2; x++) {
                imgThresholded.at<uchar>(y, x) = 0;
            }
        }
    }

    // 记录中心点位置的x
    unordered_map<int, int> left_points;
    unordered_map<int, int> right_points;
    // 记录中心点位置的x，一直更新保持连贯性质
    int center_x = imgThresholded.cols / 2;

    // 考虑范围 - 大幅扩大处理区域，让黄线能画得更远
    int img_range = imgThresholded.rows / 6;  // 从图像上六分之一开始处理，覆盖更大范围

    // 后面方便计算center_x的中值
    int total_center_x = 0;
    // 补线长度
    int bx_size = 0;
    
    // 判断当前帧是否检测到路径端点
    bool is_end = false;

    for (int y = imgThresholded.rows - 1; y >= img_range; y--) 
    {
        int mid = center_x;
        int leftBoundary = -1;
        int rightBoundary = -1;

        // 使用局部变量保存 center_x
        int local_center_x = center_x;

        // 开始统计
        total_center_x += center_x;
        
        if (imgThresholded.at<uchar>(y, local_center_x) == 255) {
            if (img_debug_output) printUTF8("检测到路径端点");
            is_end = true;
            break;  // 简化处理，直接跳出循环
        }

        // 从中心向左遍历找到左边界点 - 优化远处线条检测
        for (int x = mid; x >= 0; x--) {
            if (x + 1 < imgThresholded.cols && x - 1 >= 0) {
                // 标准边界检测：白->白->黑
                if (imgThresholded.at<uchar>(y, x) == 255 && imgThresholded.at<uchar>(y, x - 1) == 255 && imgThresholded.at<uchar>(y, x + 1) == 0) {
                    leftBoundary = x;
                    left_points[y] = x;
                    break;
                }
                // 远处细线检测：白->黑 (更宽松的条件)
                else if (y < imgThresholded.rows / 2 && 
                         imgThresholded.at<uchar>(y, x) == 255 && 
                         imgThresholded.at<uchar>(y, x + 1) == 0) {
                    leftBoundary = x;
                    left_points[y] = x;
                    break;
                }
            }
        }

        // 从中心向右遍历找到右边界点 - 优化远处线条检测
        for (int x = mid; x < imgThresholded.cols; x++) {
            if (x + 1 < imgThresholded.cols && x - 1 >= 0) {
                // 标准边界检测：黑->白->白
                if (imgThresholded.at<uchar>(y, x) == 255 && imgThresholded.at<uchar>(y, x + 1) == 255 && imgThresholded.at<uchar>(y, x - 1) == 0) {
                    rightBoundary = x;
                    right_points[y] = x;
                    break;
                }
                // 远处细线检测：黑->白 (更宽松的条件)
                else if (y < imgThresholded.rows / 2 && 
                         imgThresholded.at<uchar>(y, x) == 255 && 
                         imgThresholded.at<uchar>(y, x - 1) == 0) {
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
            colorResult.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
        }
        else if (!hasLeft && hasRight) {
            center_x = (rightBoundary + 1) / 2;
            // 将中线点画出来
            colorResult.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
        }
        else if (hasLeft && !hasRight) {
            center_x = (leftBoundary + imgThresholded.cols - 2) / 2;
            // 将中线点画出来
            colorResult.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
        }
        else {
            // 两边都不存在，补线
            center_x = (imgThresholded.cols - 1) / 2;
            colorResult.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            bx_size ++;
        }

        // 扩展中线 - 与line_follower2_node完全一致
        for (int x = center_x, i = x - 1, j = x + 1; i >= center_x - center_p && j <= center_x + center_p; i--, j++) {
            if (i >= 0 && i < colorResult.cols) {
                colorResult.at<Vec3b>(y, i) = Vec3b(0, 255, 255); // 使用黄色
            }
            if (j >= 0 && j < colorResult.cols) {
                colorResult.at<Vec3b>(y, j) = Vec3b(0, 255, 255); // 使用黄色
            }
        }
    }

    // 在图像上添加HSV参数信息
    if (img_debug_output) {
        string hsvInfo = "HSV: H(" + to_string(iLowH) + "," + to_string(iHighH) + 
                        ") S(" + to_string(iLowS) + "," + to_string(iHighS) + 
                        ") V(" + to_string(iLowV) + "," + to_string(iHighV) + ")";
        putText(colorResult, hsvInfo, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        
        // 计算检测到的像素数量
        int pixelCount = countNonZero(imgThresholded);
        string pixelInfo = "Pixels: " + to_string(pixelCount);
        putText(colorResult, pixelInfo, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        // 添加裁剪信息
        string cropInfo = "Cropped: Top 1/4 removed";
        putText(colorResult, cropInfo, Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        
    // 添加路径信息 - 与line_follower2_node格式保持一致
    if (img_debug_output) {
        printUTF8("left_size：" + to_string(left_points.size()) + " right_size：" + to_string(right_points.size()) + " bx_size：" + to_string(bx_size));
        string pathInfo = "Path: L=" + to_string(left_points.size()) + " R=" + to_string(right_points.size()) + " BX=" + to_string(bx_size);
        string rangeInfo = " Range: " + to_string(img_range) + "->" + to_string(imgThresholded.rows-1);
        putText(colorResult, pathInfo + rangeInfo, Point(10, 210), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);
    }
        
        // 添加相机校正状态信息
        string calibInfo = camera_info_received ? "Camera: CALIBRATED" : "Camera: WAITING";
        Scalar calibColor = camera_info_received ? Scalar(0, 255, 0) : Scalar(0, 165, 255);
        putText(colorResult, calibInfo, Point(10, 180), FONT_HERSHEY_SIMPLEX, 0.7, calibColor, 2);
        
        // 添加效果评估信息
        string qualityInfo;
        if (pixelCount < 100) {
            qualityInfo = "Quality: TOO FEW pixels";
            putText(colorResult, qualityInfo, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        } else if (pixelCount > 3000) {
            qualityInfo = "Quality: TOO MANY pixels";
            putText(colorResult, qualityInfo, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        } else {
            qualityInfo = "Quality: GOOD";
            putText(colorResult, qualityInfo, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        }
        
        // 计算线条的连续性（简单评估）- 在下四分之三区域计算
        int centerX = imgThresholded.cols / 2;
        int continuousLines = 0;
        int startY = imgThresholded.rows / 4;  // 从四分之一处开始（对应裁剪后的有效区域）
        int totalRows = imgThresholded.rows - startY;
        
        for (int y = imgThresholded.rows - 1; y >= startY; y--) {
            bool foundLine = false;
            for (int x = centerX - 50; x <= centerX + 50; x++) {
                if (x >= 0 && x < imgThresholded.cols && imgThresholded.at<uchar>(y, x) == 255) {
                    foundLine = true;
                    break;
                }
            }
            if (foundLine) continuousLines++;
        }
        
        float continuity = (float)continuousLines / totalRows * 100;
        string continuityInfo = "Continuity: " + to_string((int)continuity) + "%";
        putText(colorResult, continuityInfo, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, 
                continuity > 60 ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
    }

    // 发布原始RGB图像
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgOriginal).toImageMsg();
    rgb_pub.publish(rgb_msg);

    // 发布HSV图像
    sensor_msgs::ImagePtr hsv_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgHSV).toImageMsg();
    hsv_pub.publish(hsv_msg);

    // 发布处理后的结果图像（彩色版本，便于观察）
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorResult).toImageMsg();
    result_pub.publish(result_msg);

    // 发布二值化图像
    sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThresholded).toImageMsg();
    binary_pub.publish(binary_msg);
}

int main(int argc, char **argv)
{
    // 设置环境变量以支持UTF-8
    setenv("LC_ALL", "C.UTF-8", 1);
    setenv("LANG", "C.UTF-8", 1);
    
    // 初始化ROS节点
    ros::init(argc, argv, "image_hsv");

    // 创建节点句柄
    ros::NodeHandle nh;
    
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

    // 读取配置文件 - 与line_follower2_node同步
    readImageParameters();

    // 设置动态重配置服务器
    config_server = new dynamic_reconfigure::Server<line_follower2::HSVThresholdConfig>();
    
    // 先设置回调函数
    dynamic_reconfigure::Server<line_follower2::HSVThresholdConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    config_server->setCallback(f);
    
    // 稍等一下让动态重配置系统初始化
    ros::Duration(0.1).sleep();
    
    // 将从配置文件读取的参数设置到动态重配置服务器
    line_follower2::HSVThresholdConfig config;
    config.low_H = iLowH;
    config.high_H = iHighH;
    config.low_S = iLowS;
    config.high_S = iHighS;
    config.low_V = iLowV;
    config.high_V = iHighV;
    
    if (img_debug_output) {
        printUTF8("准备更新动态重配置服务器参数:");
        printUTF8("  设置 H(" + to_string(config.low_H) + "," + to_string(config.high_H) + 
                 ") S(" + to_string(config.low_S) + "," + to_string(config.high_S) + 
                 ") V(" + to_string(config.low_V) + "," + to_string(config.high_V) + ")");
    }
    
    config_server->updateConfig(config);
    
    // 再次稍等让更新生效
    ros::Duration(0.1).sleep();
    
    // 创建重新加载配置文件的服务
    ros::ServiceServer reload_service = nh.advertiseService("/image_hsv/reload_config", reloadConfigCallback);

    // 订阅配置的图像话题
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe(image_topic, 1, Cam_RGB_Callback);

    // 订阅相机信息话题
    ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, CameraInfoCallback);

    // 创建图像发布器 - 增加更多话题用于调试
    rgb_pub = it.advertise("/image_hsv/rgb", 1);
    hsv_pub = it.advertise("/image_hsv/hsv", 1);
    result_pub = it.advertise("/image_hsv/result", 1);
    binary_pub = it.advertise("/image_hsv/binary", 1);

    // 打印启动信息
    if (img_debug_output) {
        printUTF8("============================HSV图像处理节点============================");
        printUTF8("订阅话题:");
        printUTF8("  - 图像话题: " + image_topic);
        printUTF8("  - 相机信息: " + camera_info_topic);
        printUTF8("发布话题:");
        printUTF8("  - 原始图像: /image_hsv/rgb");
        printUTF8("  - HSV图像: /image_hsv/hsv");
        printUTF8("  - 处理结果: /image_hsv/result");
        printUTF8("  - 二值化图像: /image_hsv/binary");
        printUTF8("服务:");
        printUTF8("  - 重新加载配置: /image_hsv/reload_config");
        printUTF8("调试模式: 开启");
        printUTF8("功能特性:");
        printUTF8("  - 实时HSV参数调优");
        printUTF8("  - 相机畸变校正");
        printUTF8("  - 图像质量评估");
        printUTF8("  - 空间裁剪 (忽略上1/4)");
        printUTF8("  - 远距离线条检测与轨迹预测");
        printUTF8("  - 与line_follower2节点画线算法同步");
        printUTF8("================================================================");
    }

    // 设置循环频率
    ros::Rate loop_rate(30);

    // 主循环
    while(ros::ok())
    {
        // 处理ROS的事件
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 清理动态分配的内存
    if (config_server) {
        delete config_server;
        config_server = nullptr;
    }

    return 0;
}