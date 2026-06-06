#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

#define PI 3.14159265358979323846

struct Target_Info {
    double ex = 0; double ey = 0; int w = 0; int flag = 0;
};

class DetectMissionNode : public rclcpp::Node {
public:
    DetectMissionNode() : Node("detect_mission") {
        this->declare_parameter<string>("camera_topic", "/iris/usb_cam/image_raw");
        this->get_parameter("camera_topic", camera_topic_);

        // 状态机初始化：阶段1 代表寻找门框
        mission_stage_ = 1; 

        callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_sub_;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, std::bind(&DetectMissionNode::image_callback, this, std::placeholders::_1), sub_options);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&DetectMissionNode::control_loop, this), callback_group_timer_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            cv::cvtColor(cv_ptr->image, image_from_topic_, COLOR_RGB2BGR);
        } catch (cv_bridge::Exception& e) { return; }

        cv::Mat imgOriginal = image_from_topic_.clone();
        if (imgOriginal.empty()) return;

        cv::Mat imgHSV, mask;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        std::lock_guard<std::mutex> lock(data_mutex_);

        // ==================== 阶段 1：视觉寻找圆形门框 ====================
        if (mission_stage_ == 1) {
            // 使用原本识别圆环的绿色/特定颜色HSV区间
            inRange(imgHSV, Scalar(66, 91, 51), Scalar(255, 255, 255), mask);
            
            vector<vector<Point>> contours;
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            float max_radius = 0; int max_id = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                if (contours[i].size() < 5) continue;
                float area = contourArea(contours[i]);
                float arc_len = arcLength(contours[i], true);
                float r = arc_len / PI / 2;
                if (r > 30 && r > max_radius) { max_radius = r; max_id = i; }
            }

            if (max_id != -1) {
                cv::RotatedRect ellipse_rect = fitEllipse(contours[max_id]);
                cv::circle(imgOriginal, ellipse_rect.center, max_radius, Scalar(0, 255, 0), 2);
                
                target_.ex = imgOriginal.cols / 2 - ellipse_rect.center.x;
                target_.ey = imgOriginal.rows / 2 - ellipse_rect.center.y;
                target_.w = max_radius;
                target_.flag = 1;

                // 原本的穿越条件：如果圆环半径大于屏幕1/3且对准了，切入阶段2（盲冲穿越）
                if (max_radius > min(imgOriginal.rows, imgOriginal.cols) / 3 && (sqrt(target_.ex*target_.ex + target_.ey*target_.ey) < 15)) {
                    mission_stage_ = 2;
                    fly_time_ = 25.0; // 设定穿越盲冲计数（2.5秒）
                    RCLCPP_WARN(this->get_logger(), ">>> 极其接近门框！状态机切入阶段2：盲冲穿越！");
                }
            } else { set_no_target(); }
        }
        // ==================== 阶段 3：视觉寻找红球（穿越后触发） ====================
        else if (mission_stage_ == 3) {
            // 使用原本识别气球的红色HSV双区间抠图逻辑
            cv::Mat mask1, mask2;
            inRange(imgHSV, Scalar(0, 43, 120), Scalar(20, 256, 256), mask1);
            inRange(imgHSV, Scalar(150, 43, 120), Scalar(180, 256, 256), mask2);
            mask = mask1 | mask2;

            vector<vector<Point>> contours;
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            float max_area = 0; int max_id = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                float a = contourArea(contours[i]);
                if (a > max_area && a > 30) { max_area = a; max_id = i; }
            }

            if (max_id != -1) {
                Rect rect = boundingRect(contours[max_id]);
                cv::rectangle(imgOriginal, rect, Scalar(0, 0, 255), 2);
                
                target_.ex = imgOriginal.cols / 2 - (rect.x + rect.width / 2);
                target_.ey = imgOriginal.rows / 2 - (rect.y + rect.height / 2);
                target_.w = rect.width;
                target_.flag = 1;
            } else { set_no_target(); }
        }

        cv::imshow("Mission Monitor", imgOriginal);
        cv::waitKey(1);
    }

    void set_no_target() { target_.ex = 0; target_.ey = 0; target_.w = 250; target_.flag = 0; }

    // ==================== 状态机控制输出主循环 ====================
    void control_loop() {
        double desire_vx = 0.0, desire_vy = 0.0, desire_vz = 0.0;

        std::unique_lock<std::mutex> lock(data_mutex_);
        int stage = mission_stage_;
        Target_Info tgt = target_;
        lock.unlock();

        if (stage == 1) { // 追踪圆环
            if (tgt.flag == 1) {
                desire_vx = 0.005 * (250 - tgt.w); // 逼近
                desire_vy = 0.005 * tgt.ex; desire_vz = 0.01 * tgt.ey;
            } else { desire_vz = 0.02; } // 没看到就微爬升寻找
        } 
        else if (stage == 2) { // 盲冲穿越门框
            desire_vx = 0.6; desire_vy = 0.0; desire_vz = 0.0; // 笔直向前冲
            
            std::lock_guard<std::mutex> lock_stage(data_mutex_);
            fly_time_ -= 1.0;
            if (fly_time_ <= 0) {
                mission_stage_ = 3; // 盲冲时间到，圆环已在身后的草地上，切换到阶段3寻找红球
                RCLCPP_WARN(this->get_logger(), ">>> 穿越成功！状态机切入阶段3：开始追击红球！");
            }
        } 
        else if (stage == 3) { // 撞击红球
            if (tgt.flag == 1) {
                desire_vx = 0.25; // 持续向前
                desire_vy = 0.004 * tgt.ex; desire_vz = 0.004 * tgt.ey;
                if (tgt.w > 200) { RCLCPP_INFO(this->get_logger(), "Successfully Hit the RedBall!!!"); }
            } else { desire_vx = 0.0; desire_vy = 0.0; desire_vz = 0.0; } // 丢失则悬停
        }

        // 限制速度幅值
        if (stage != 2) {
            desire_vx = max(-0.3, min(0.3, desire_vx));
            desire_vy = max(-0.3, min(0.3, desire_vy));
            desire_vz = max(-0.3, min(0.3, desire_vz));
        }

        // 发送给 PX4 飞控
        px4_msgs::msg::TrajectorySetpoint sp;
        sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        sp.position = {static_cast<float>(NAN), static_cast<float>(NAN), static_cast<float>(NAN)};
        sp.velocity = {static_cast<float>(desire_vx), static_cast<float>(-desire_vy), static_cast<float>(-desire_vz)};
        sp.yaw = 0.0; // 保持轴向锁死
        setpoint_pub_->publish(sp);
    }

    string camera_topic_; cv::Mat image_from_topic_;
    Target_Info target_; std::mutex data_mutex_;
    int mission_stage_; float fly_time_ = 0.0;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectMissionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}