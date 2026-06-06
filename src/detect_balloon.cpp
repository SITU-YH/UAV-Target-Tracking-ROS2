#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV Headers
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PX4 ROS2 Message Headers
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

#define PI 3.14159265358979323846

struct Detect_Error {
    double ex = 0;
    double ey = 0;
    int w = 0;
    int flag = 0;
};

class DetectBalloonNode : public rclcpp::Node {
public:
    DetectBalloonNode() : Node("detect_balloon") {
        // 声明并获取参数
        this->declare_parameter<bool>("simulation", false);
        this->declare_parameter<bool>("save_video", true);
        this->declare_parameter<string>("camera_topic", "/iris/usb_cam/image_raw");

        this->get_parameter("simulation", simulation_);
        this->get_parameter("save_video", save_video_);
        this->get_parameter("camera_topic", camera_topic_);

        RCLCPP_INFO(this->get_logger(), "Input Camera Topic: %s", camera_topic_.c_str());

        // 为了防止图像回调与控制循环相互阻塞，创建两个不同的并发回调组
        callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_sub_;

        // 订阅图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, std::bind(&DetectBalloonNode::image_callback, this, std::placeholders::_1), sub_options);

        // 原生 PX4 话题发布器 (Micro XRCE-DDS 命名空间通常为 /fmu/in/...)
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // 启动 10Hz 定时器，等价于旧代码中的控制发布主循环
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&DetectBalloonNode::send_ext_cmd_loop, this), callback_group_timer_);

        // 视频保存初始化路径
        char s[30];
        time_t now = time(NULL);
        std::strftime(s, 30, "Fly_%Y_%b_%d_%H_%M_%S.avi", localtime(&now));
        char *cwd = getenv("HOME");
        save_path_ = string(cwd) + string("/") + string(s);
    }

    ~DetectBalloonNode() {
        if (save_video_ && !video_write_first_) {
            video_w_.release();
        }
        cv::destroyAllWindows();
    }

private:
    // 图像回调及气球检测
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            cv::cvtColor(cv_ptr->image, image_from_topic_, COLOR_RGB2BGR);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat imgOriginal = image_from_topic_.clone();
        if (imgOriginal.empty()) return;

        if (save_video_ && video_write_first_) {
            video_w_.open(save_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, 
                          cv::Size(imgOriginal.cols, imgOriginal.rows), true);
            video_write_first_ = false;
        }

        cv::Mat imgBalance = WhiteBalance_PRA(imgOriginal);
        if (imgBalance.rows <= 1) {
            imgBalance = WhiteBalance_Gray(imgOriginal);
        }

        cv::Mat imgRGB = imgBalance.clone();
        cv::Mat imgHSV, imgGray, output, binary;
        cvtColor(imgRGB, imgHSV, COLOR_BGR2HSV);

        for (int i = 0; i < imgHSV.rows; i++) {
            for (int j = 0; j < imgHSV.cols; j++) {
                int H = imgHSV.at<Vec3b>(i, j)[0];
                int S = imgHSV.at<Vec3b>(i, j)[1];
                int V = imgHSV.at<Vec3b>(i, j)[2];
                if ((((H >= 0) && (H <= 20) || (H >= 150) && (H <= 180)) && (S >= 43 && S <= 256) && (V > 120 && V < 256))) {
                    imgRGB.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
                } else {
                    imgRGB.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
                }
            }
        }

        cvtColor(imgRGB, imgGray, COLOR_BGR2GRAY);
        cv::Mat element1 = getStructuringElement(MORPH_RECT, Size(10, 10));
        cv::Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
        dilate(imgGray, output, element1);
        erode(output, output, element2);
        
        cv::imshow("rgb", imgRGB);
        cv::imshow("gray", output);
        threshold(output, binary, 120, 255, THRESH_BINARY_INV);
        cv::imshow("binary", binary);

        vector<vector<Point>> contours;
        vector<Vec4i> hierachy;
        findContours(binary, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        float areas[100] = {0}, _max = 0;
        int _maxid = 0;

        std::lock_guard<std::mutex> lock(data_mutex_); // 锁保护共享数据
        if (!contours.empty()) {
            for (size_t k = 0; k < contours.size() && k < 100; k++) {
                vector<Point> hull;
                convexHull(contours[k], hull);
                areas[k] = contourArea(hull);
                if (areas[k] > _max && areas[k] > 30 && areas[k] < 60000) {
                    _max = areas[k];
                    _maxid = k;
                }
            }

            Rect rect = boundingRect(contours[_maxid]);
            float var = circle_detect(contours[_maxid], rect.x + rect.width / 2, rect.y + rect.height / 2);

            if (_max > 30 && _max < 60000 && var < 100) {
                rectangle(imgOriginal, rect, Scalar(255, 0, 0));
                RCLCPP_INFO(this->get_logger(), "Detect Result: [%d, %d, %d, %d]", rect.x, rect.y, rect.width, rect.height);
                detect_error_.ex = imgOriginal.cols / 2 - (rect.x + rect.width / 2);
                detect_error_.ey = imgOriginal.rows / 2 - (rect.y + rect.height / 2);
                detect_error_.w = rect.width;
                detect_error_.flag = 1;
            } else {
                set_no_detection();
            }
        } else {
            set_no_detection();
        }

        cv::imshow("raw_img", imgOriginal);
        if (save_video_ && !video_write_first_) {
            video_w_.write(imgOriginal);
        }
        cv::waitKey(1);
    }

    void set_no_detection() {
        detect_error_.ex = 0;
        detect_error_.ey = 0;
        detect_error_.w = 250;
        detect_error_.flag = 0;
    }

    // 控制环路定时器（平替原 send_ext_cmd 线程）
    void send_ext_cmd_loop() {
        double desire_cmd_value[3] = {0.0, 0.0, 0.0};
        
        std::unique_lock<std::mutex> lock(data_mutex_);
        Detect_Error err = detect_error_;
        lock.unlock();

        // 模拟原本等待特定自定义模式的逻辑，ROS2中通常靠外部切入Offboard激活
        if (err.flag == 1 && abs(err.ex) < 120) {
            if (sqrt(err.ex * err.ex + err.ey * err.ey) < 10) {
                desire_cmd_value[0] = 0;
            } else {
                desire_cmd_value[0] = 0.003 * (250 - err.w);
            }
            desire_cmd_value[1] = 0.003 * err.ex; 
            desire_cmd_value[2] = 0.003 * err.ey;
            
            last_pos_ = err.ex / (abs(err.ex) + 1e-5);
            fail_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Find Balloon!");
        } else {
            if (err.flag == 0) {
                fail_count_++;
                if (fail_count_ > 5) {
                    desire_cmd_value[0] = 0;
                    desire_cmd_value[1] = 0;
                    desire_cmd_value[2] = 0.05; // 找不到时向上微爬

                    RCLCPP_WARN(this->get_logger(), "Not Detect Balloon! Searching...");
                    if (last_pos_ > 0) yaw_value_ += PI / 60;
                    else if (last_pos_ < 0) yaw_value_ += PI / 60;
                    else yaw_value_ += PI / 60;
                }
            } else if (err.flag == 1 && abs(err.ex) > 120) {
                desire_cmd_value[0] = 0; desire_cmd_value[1] = 0; desire_cmd_value[2] = 0;
                RCLCPP_WARN(this->get_logger(), "Balloon Lost!");
                if (err.ex > 0) yaw_value_ += PI / 60;
                else yaw_value_ -= PI / 60;
            }
        }

        // 限幅限速
        for (int i = 0; i < 3; i++) {
            if (desire_cmd_value[i] > 0.3) desire_cmd_value[i] = 0.3;
            if (desire_cmd_value[i] < -0.3) desire_cmd_value[i] = -0.3;
        }

        // 构造发送给 PX4 的 ROS2 航点控制命令（BODY 机体系速度控制）
        px4_msgs::msg::TrajectorySetpoint sp_msg;
        sp_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        // 注意：原代码控制属于BODY坐标系控制，PX4原生仅接收NED惯性系或航向相关的速度。
        // 此处为了平替原有BODY坐标系指令，直接填充为速度分量，并将位置设为NaN标识其不生效
        sp_msg.position = {static_cast<float>(NAN), static_cast<float>(NAN), static_cast<float>(NAN)};
        
        // 在PX4中机体系转换对应关系：原机体系向前=sp_msg.velocity[0]，向左= -sp_msg.velocity[1], 向上= -sp_msg.velocity[2]
        sp_msg.velocity = {static_cast<float>(desire_cmd_value[0]), 
                           static_cast<float>(-desire_cmd_value[1]), 
                           static_cast<float>(-desire_cmd_value[2])};
        sp_msg.yaw = yaw_value_;

        setpoint_pub_->publish(sp_msg);
    }

    // 图像处理辅助数学函数保持不变
    float circle_detect(vector<Point> contour, float cx, float cy) {
        int num = 0; float sum = 0, tmp = 0;
        for (size_t i = 0; i < contour.size(); i++) {
            sum += sqrt((cx - contour[i].x) * (cx - contour[i].x) + (cy - contour[i].y) * (cy - contour[i].y));
            num++;
        }
        float mean = sum / num; sum = 0.0f;
        for (size_t i = 0; i < contour.size(); i++) {
            tmp = (sqrt((cx - contour[i].x) * (cx - contour[i].x) + (cy - contour[i].y) * (cy - contour[i].y)) - mean);
            sum += tmp * tmp;
        }
        return sum / num;
    }

    cv::Mat WhiteBalance_PRA(cv::Mat src) {
        cv::Mat result = src.clone();
        if (src.channels() != 3) return result;
        vector<cv::Mat> Channel; cv::split(src, Channel);
        int row = src.rows; int col = src.cols;
        int RGBSum[765] = {0}; uchar maxR = 0, maxG = 0, maxB = 0;
        for (int i = 0; i < row; ++i) {
            uchar *b = Channel[0].ptr<uchar>(i); uchar *g = Channel[1].ptr<uchar>(i); uchar *r = Channel[2].ptr<uchar>(i);
            for (int j = 0; j < col; ++j) {
                RGBSum[b[j] + g[j] + r[j]]++;
                maxB = max(maxB, b[j]); maxG = max(maxG, g[j]); maxR = max(maxR, r[j]);
            }
        }
        int T = 0; int num = 0; int K = static_cast<int>(row * col * 0.5);
        for (int i = 765; i >= 0; --i) {
            num += RGBSum[i]; if (num > K) { T = i; break; }
        }
        double Bm = 0.0, Gm = 0.0, Rm = 0.0; int count = 0;
        for (int i = 0; i < row; ++i) {
            uchar *b = Channel[0].ptr<uchar>(i); uchar *g = Channel[1].ptr<uchar>(i); uchar *r = Channel[2].ptr<uchar>(i);
            for (int j = 0; j < col; ++j) {
                if ((b[j] + g[j] + r[j]) > T) { Bm += b[j]; Gm += g[j]; Rm += r[j]; count++; }
            }
        }
        if (count == 0) return cv::Mat();
        Bm /= count; Gm /= count; Rm /= count;
        Channel[0] *= maxB / Bm; Channel[1] *= maxG / Gm; Channel[2] *= maxR / Rm;
        cv::merge(Channel, result);
        return result;
    }

    cv::Mat WhiteBalance_Gray(cv::Mat src) {
        cv::Mat result = src.clone();
        if (src.channels() != 3) return result;
        std::vector<cv::Mat> Channel; cv::split(src, Channel);
        double Bm = cv::mean(Channel[0])[0]; double Gm = cv::mean(Channel[1])[0]; double Rm = cv::mean(Channel[2])[0];
        double Km = (Bm + Gm + Rm) / 3;
        Channel[0] *= Km / Bm; Channel[1] *= Km / Gm; Channel[2] *= Km / Rm;
        cv::merge(Channel, result);
        return result;
    }

    // 内部私有变量
    bool simulation_;
    bool save_video_;
    string camera_topic_;
    cv::Mat image_from_topic_;
    Detect_Error detect_error_;
    std::mutex data_mutex_;

    int fail_count_ = 0;
    double last_pos_ = 0;
    double yaw_value_ = 0;

    cv::VideoWriter video_w_;
    bool video_write_first_ = true;
    string save_path_;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectBalloonNode>();
    
    // 使用多线程执行器来保证并发回调顺畅触发
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}