#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// Action Node to move forward
class ImageCollector : public rclcpp::Node {
public:
    ImageCollector() : rclcpp::Node("image_collector") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&ImageCollector::image_callback, this, std::placeholders::_1)
        );
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV image
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if (ImageCollector::getImagePtr() == nullptr) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Null image pointer.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set image pointer.");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    static cv_bridge::CvImagePtr getImagePtr() {
        return cv_ptr;
    }

    static cv_bridge::CvImagePtr cv_ptr;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

cv_bridge::CvImagePtr ImageCollector::cv_ptr = nullptr;