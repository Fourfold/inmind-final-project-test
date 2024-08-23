#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "detection_interfaces/action/find_yellow.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using FindYellow = detection_interfaces::action::FindYellow;
using GoalHandleFindYellow = rclcpp_action::ServerGoalHandle<FindYellow>;

class YellowBallActionServer : public rclcpp::Node {
public:
    YellowBallActionServer();
private:
    rclcpp_action::Server<FindYellow>::SharedPtr action_server;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FindYellow::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFindYellow> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFindYellow> goal_handle);
    void execute(const std::shared_ptr<GoalHandleFindYellow> goal_handle);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv_bridge::CvImagePtr cv_ptr;
};


YellowBallActionServer::YellowBallActionServer() : Node("yellow_ball_action_server") {
    using namespace std::placeholders;
    // Subscriber to the camera's image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&YellowBallActionServer::image_callback, this, std::placeholders::_1));
    this->action_server = rclcpp_action::create_server<FindYellow>(
        this,
        "yellow_ball",
        std::bind(&YellowBallActionServer::handle_goal, this, _1, _2),
        std::bind(&YellowBallActionServer::handle_cancel, this, _1),
        std::bind(&YellowBallActionServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FindYellow server started. Ready for requests.");
}

rclcpp_action::GoalResponse YellowBallActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FindYellow::Goal> goal) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal request.");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YellowBallActionServer::handle_cancel(const std::shared_ptr<GoalHandleFindYellow> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void YellowBallActionServer::handle_accepted(const std::shared_ptr<GoalHandleFindYellow> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&YellowBallActionServer::execute, this, _1), goal_handle}.detach();
}

void YellowBallActionServer::execute(const std::shared_ptr<GoalHandleFindYellow> goal_handle) {
    int cx, cy;
    bool sphere_detected = false;
    int frame_width;

    // Get frame width
    frame_width = cv_ptr->image.cols;

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the range for yellow color in HSV
    cv::Scalar lower_yellow(10, 70, 70);
    cv::Scalar upper_yellow(30, 255, 255);

    // Threshold the HSV image to get only yellow colors
    cv::Mat mask;
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
    {
        // Find the largest contour
        double max_area = 0;
        int largest_contour_index = -1;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = cv::contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                largest_contour_index = i;
            }
        }

        // If a valid contour is found
        if (largest_contour_index >= 0 && max_area > 20)
        {
            // Calculate the centroid of the largest contour
            cv::Moments M = cv::moments(contours[largest_contour_index]);
            if (M.m00 > 0)
            {
                cx = static_cast<int>(M.m10 / M.m00);
                cy = static_cast<int>(M.m01 / M.m00);
                sphere_detected = true;
            }
        }
    }

    auto result = std::make_shared<FindYellow::Result>();
    if (sphere_detected) {
        result->found = true;
        result->frame_width = frame_width;
        result->cx = cx;
        result->cy = cy;
    } else {
        result->found = false;
        result->frame_width = frame_width;
        result->cx = -1;
        result->cy = -1;
    }
    goal_handle->succeed(result);
}

void YellowBallActionServer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS Image message to OpenCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}
