#ifndef YELLOW_BALL_FINDER_H_
#define YELLOW_BALL_FINDER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class YellowSphereFollower : public rclcpp::Node
{
public:
    YellowSphereFollower() : Node("yellow_sphere_follower"), sphere_detected_(false)
    {
        // Subscriber to the camera's image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&YellowSphereFollower::image_callback, this, std::placeholders::_1));

        // Publisher to the TurtleBot3's velocity command topic
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer to control the robot's movements
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&YellowSphereFollower::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Yellow Sphere Follower Node Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get frame width
        frame_width_ = cv_ptr->image.cols;

        // Convert the image to HSV color space
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Define the range for yellow color in HSV
        cv::Scalar lower_yellow(20, 100, 100);
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
            if (largest_contour_index >= 0 && max_area > 500)
            {
                // Calculate the centroid of the largest contour
                cv::Moments M = cv::moments(contours[largest_contour_index]);
                if (M.m00 > 0)
                {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);
                    sphere_center_ = cv::Point(cx, cy);
                    sphere_detected_ = true;
                    return;
                }
            }
        }

        // If no valid sphere is detected, set sphere_detected_ to false
        sphere_detected_ = false;
    }

    void control_loop()
    {
        auto twist_msg = geometry_msgs::msg::Twist();

        if (sphere_detected_)
        {
            int error_x = sphere_center_.x - frame_width_ / 2;

            // Rotation direction based on error
            if (std::abs(error_x) > 20)
            {
                twist_msg.angular.z = -0.002 * error_x; // Rotate proportionally to the error
            }
            else
            {
                twist_msg.linear.x = 0.1; // Move forward slowly
            }
        }
        else
        {
            // No sphere detected, rotate right
            twist_msg.angular.z = -0.3;
        }

        // Publish the velocity command
        cmd_vel_pub_->publish(twist_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Point sphere_center_;
    bool sphere_detected_;
    int frame_width_;
};




#endif