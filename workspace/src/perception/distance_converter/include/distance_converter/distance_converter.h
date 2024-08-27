#ifndef DISTANCE_CONVERTER_H_
#define DISTANCE_CONVERTER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class DistanceConverter : public rclcpp::Node {
public:
    DistanceConverter(): Node("distance_sensor_server") {
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&DistanceConverter::scan_callback, this, std::placeholders::_1));
        distance_publisher = this->create_publisher<std_msgs::msg::Float32>("/distanceToObstacle", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & data) {
        float forwardSum = data.ranges.at(360 - 1) + data.ranges.at(360 - 2) + data.ranges.at(360 - 3) + data.ranges.at(360 - 4) + data.ranges.at(360 - 5);
        std_msgs::msg::Float32 msg;
        forwardSum = 15 * forwardSum / 5;
        if (forwardSum > 300.0) {
            msg.data = 0.0;
        } else {
            msg.data = forwardSum;
        }
        distance_publisher->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher;
};

#endif