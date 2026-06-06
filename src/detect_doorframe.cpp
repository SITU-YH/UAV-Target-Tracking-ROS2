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

struct Detect_Info {
    double ex = 0;
    double ey = 0;
    int w = 0;
    int flag = 0;
};

class DetectDoorFrameNode : public rclcpp::Node {
public:
    DetectDoorFrameNode() : Node("detect_doorframe") {
        this->declare_parameter<bool>("simulation", false);
        this->declare_parameter<bool>("save_video", false);
        this->declare_parameter<string>("camera_topic", "/iris/usb_cam/image_raw");

        this->get_parameter("simulation", simulation_);
        this->get_parameter("save_video", save_video_);
        this->get_parameter("camera_topic", camera_topic_);

        callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_sub_;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, std::bind(&DetectDoorFrameNode::image_callback, this, std::placeholders::_1), sub_options);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&DetectDoorFrameNode::send_ext_cmd_loop, this), callback_group_timer_);

        char s[30]; time_t now = time(NULL);
        std::strftime(s, 30, "Fly_%Y_%b_%d_%H_%M_%S.avi", localtime(&now));
        char *cwd = getenv("HOME");
        save_path_ = string(cwd) + string("/") + string(s);
        
        yaw_value_ = 0 - PI / 2; // 初始化位置姿态
    }

    ~DetectDoorFrameNode() {
        if (save_video_ && !video_write_first_) { video_w_.release(); }
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            cv::cvtColor(cv_ptr->image, image_from_topic_, COLOR_RGB2BGR);
        } catch (cv_bridge::Exception& e) {
            return;
        }

        cv::Mat imgOriginal = image_from_topic_.clone();
        if (imgOriginal.empty()) return;

        if (save_video_ && video_write_first_) {
            video_w_.open(save_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(imgOriginal.cols, imgOriginal.rows), true);
            video_write_first = false;
        }

        cv::Mat imgBalance = WhiteBalance_PRA(imgOriginal);
        if (imgBalance.rows <= 1) { imgBalance = WhiteBalance_Gray(imgOriginal); }

        cv::Mat imgRGB = imgBalance.clone();
        cv::Mat imgHSV, imgGray, output, HSV_mask, connect_lables, connect_stats, connect_centroids;
        cvtColor(imgRGB, imgHSV, COLOR_BGR2HSV);
        
        cv::Scalar HSV_lower(66, 91, 51);
        cv::Scalar HSV_upper(255, 255, 255);
        inRange(imgHSV, HSV_lower, HSV_upper, HSV_mask);

        int n = connectedComponentsWithStats(HSV_mask, connect_lables, connect_stats, connect_centroids);
        int max_label_id = 0, max_area = 0;
        imgGray = cv::Mat::zeros(imgOriginal.size(), CV_8UC1);

        if (n != 1) {
            for (int i = 1; i < n; i++) {
                if (connect_stats.at<int>(i, 4) > max_area) {
                    max_area = connect_stats.at<int>(i, 4); max_label_id = i;
                }
            }
            for (int y = 0; y < imgGray.rows; y++) {
                for (int x = 0; x < imgGray.cols; x++) {
                    if (connect_lables.at<int>(y, x) == max_label_id) {
                        imgGray.at<int8_t>(y, x) = 255;
                    }
                }
            }
        }

        vector<vector<Point>> contours;
        findContours(imgGray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cv::imshow("gray", imgGray);
        threshold(imgGray, output, 120, 255, THRESH_BINARY_INV);

        std::vector<cv::Vec3f> circles;
        double minRadius = 40; double maxRadius = 120;
        float area = 0, arc_length = 0, radius = 0;

        for (size_t i = 0; i < contours.size(); i++) {
            if (contours[i].size() < 5) continue;
            area = contourArea(contours[i]);
            if (area < minRadius * minRadius * PI || area > maxRadius * maxRadius * PI) continue;
            arc_length = arcLength(contours[i], true);
            radius = arc_length / PI / 2;
            if (radius < minRadius || radius > maxRadius) continue;

            cv::RotatedRect circle_ellipse = fitEllipse(contours[i]);
            circles.push_back(Vec3f(circle_ellipse.center.x, circle_ellipse.center.y, radius));
        }

        float _max = 0; int _maxid = 0;
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!circles.empty()) {
            for (size_t i = 0; i < circles.size(); i++) {
                cv::Vec3f cc = circles[i];
                cv::circle(imgOriginal, cv::Point(cc[0], cc[1]), cc[2], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                if (cc[2] > _max) { _max = cc[2]; _maxid = i; }
            }

            if (_max > 30) {
                RCLCPP_INFO(this->get_logger(), "Detect Result: [%.2f, %.2f, %.2f]", circles[_maxid][0], circles[_maxid][1], circles[_maxid][2]);
                detect_info_.ex = imgOriginal.cols / 2 - circles[_maxid][0];
                detect_info_.ey = imgOriginal.rows / 2 - circles[_maxid][1];
                detect_info_.w = circles[_maxid][2];
                detect_info_.flag = 1;

                if (circles[_maxid][2] > min(imgGray.rows, imgGray.cols) / 3 && (sqrt(detect_info_.ex * detect_info_.ex + detect_info_.ey * detect_info_.ey) < 10)) {
                    detect_info_.flag = 2; // 进入穿越状态
                }
            } else { set_no_detection(); }
        } else { set_no_detection(); }

        cv::imshow("raw_img", imgOriginal);
        if (save_video_ && !video_write_first_) { video_w_.write(imgOriginal); }
        cv::waitKey(1);
    }

    void set_no_detection() {
        detect_info_.ex = 0; detect_info_.ey = 0; detect_info_.w = 250; detect_info_.flag = 0;
    }

    // 控制环路
    void send_ext_cmd_loop() {
        double desire_cmd_value[3] = {0.0, 0.0, 0.0};

        std::unique_lock<std::mutex> lock(data_mutex_);
        Detect_Info info = detect_info_;
        lock.unlock();

        if (info.flag == 2 || fly_across_circle_flag_) {
            if (fly_time_ > fly_init_time_ / 2) {
                desire_cmd_value[0] = 0;
                desire_cmd_value[1] = 0.005 * info.ex;
                desire_cmd_value[2] = 0.01 * info.ey;
            } else {
                desire_cmd_value[0] = 0.5; // 向前冲过门框
                desire_cmd_value[1] = 0; desire_cmd_value[2] = 0;
            }
            fly_across_circle_flag_ = true;
            fly_time_ -= 1.0;
            if (fly_time_ < 0) {
                RCLCPP_INFO(this->get_logger(), "Flying Accross DoorFrame Done.");
                desire_cmd_value[0] = 0; desire_cmd_value[1] = 0; desire_cmd_value[2] = 0;
            }
        } else if (info.flag == 1 && abs(info.ex) < 120) {
            if (sqrt(info.ex * info.ex + info.ey * info.ey) < 10) {
                desire_cmd_value[0] = 0.005 * (250 - info.w);
            } else {
                desire_cmd_value[0] = 0;
            }
            desire_cmd_value[1] = 0.005 * info.ex;
            desire_cmd_value[2] = 0.01 * info.ey;
            fail_count_ = 0;
            last_pos_ = info.ex / (abs(info.ex) + 1e-5);
            RCLCPP_INFO(this->get_logger(), "Detect DoorFrame!");
        } else {
            if (info.flag == 0) {
                fail_count_++;
                if (fail_count_ > 5) {
                    desire_cmd_value[0] = 0; desire_cmd_value[1] = 0; desire_cmd_value[2] = 0.01;
                    if (last_pos_ > 0) yaw_value_ -= PI / 60;
                    else if (last_pos_ < 0) yaw_value_ += PI / 60;
                    else yaw_value_ += PI / 60;
                }
            }
        }

        if (abs(yaw_value_) > 2 * PI) {
            yaw_value_ = yaw_value_ - floor(yaw_value_ / 2 / PI) * 2 * PI;
        }

        // 限幅限速
        for (int i = 0; i < 3; i++) {
            if (desire_cmd_value[i] > 0.3 && !fly_across_circle_flag_) desire_cmd_value[i] = 0.3;
            if (desire_cmd_value[i] < -0.3 && !fly_across_circle_flag_) desire_cmd_value[i] = -0.3;
        }

        // 发布逻辑
        px4_msgs::msg::TrajectorySetpoint sp_msg;
        sp_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        sp_msg.position = {static_cast<float>(NAN), static_cast<float>(NAN), static_cast<float>(NAN)};
        sp_msg.velocity = {static_cast<float>(desire_cmd_value[0]), 
                           static_cast<float>(-desire_cmd_value[1]), 
                           static_cast<float>(-desire_cmd_value[2])};
        sp_msg.yaw = yaw_value_;
        setpoint_pub_->publish(sp_msg);
    }

    // 完美反射及灰度白平衡处理（同上，略去冗余说明）
    cv::Mat WhiteBalance_PRA(cv::Mat src) {
        cv::Mat result = src.clone(); if (src.channels() != 3) return result;
        vector<cv::Mat> Channel; cv::split(src, Channel);
        int row = src.rows; int col = src.cols; int RGBSum[765] = {0}; uchar maxB = 0, maxG = 0, maxR = 0;
        for (int i = 0; i < row; ++i) {
            uchar *b = Channel[0].ptr<uchar>(i); uchar *g = Channel[1].ptr<uchar>(i); uchar *r = Channel[2].ptr<uchar>(i);
            for (int j = 0; j < col; ++j) {
                RGBSum[b[j] + g[j] + r[j]]++; maxB = max(maxB, b[j]); maxG = max(maxG, g[j]); maxR = max(maxR, r[j]);
            }
        }
        int T = 0; int num = 0; int K = static_cast<int>(row * col * 0.5);
        for (int i = 765; i >= 0; --i) { num += RGBSum[i]; if (num > K) { T = i; break; } }
        double Bm = 0.0, Gm = 0.0, Rm = 0.0; int count = 0;
        for (int i = 0; i < row; ++i) {
            uchar *b = Channel[0].ptr<uchar>(i); uchar *g = Channel[1].ptr<uchar>(i); uchar *r = Channel[2].ptr<uchar>(i);
            for (int j = 0; j < col; ++j) {
                if ((b[j] + g[j] + r[j]) > T) { Bm += b[j]; Gm += g[j]; Rm += r[j]; count++; }
            }
        }
        if (count == 0) return cv::Mat();
        Bm /= count; Gm /= count; Rm /= count; Channel[0] *= maxB / Bm; Channel[1] *= maxG / Gm; Channel[2] *= maxR / Rm;
        cv::merge(Channel, result); return result;
    }

    cv::Mat WhiteBalance_Gray(cv::Mat src) {
        cv::Mat result = src.clone(); if (src.channels() != 3) return result;
        std::vector<cv::Mat> Channel; cv::split(src, Channel);
        double Bm = cv::mean(Channel[0])[0]; double Gm = cv::mean(Channel[1])[0]; double Rm = cv::mean(Channel[2])[0];
        double Km = (Bm + Gm + Rm) / 4; Channel[0] *= Km / Bm; Channel[1] *= Km / Gm; Channel[2] *= Km / Rm;
        cv::merge(Channel, result); return result;
    }

    bool simulation_; bool save_video_; string camera_topic_;
    cv::Mat image_from_topic_; Detect_Info detect_info_; std::mutex data_mutex_;

    int fail_count_ = 0; double last_pos_ = 0; double yaw_value_ = 0;
    float fly_init_time_ = 20.0; float fly_time_ = 20.0; bool fly_across_circle_flag_ = false;

    cv::VideoWriter video_w_; bool video_write_first_ = true; string save_path_;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectDoorFrameNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}