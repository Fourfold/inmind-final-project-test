#ifndef GRIPPER_OBJECT_AT_CORRECT_DISTANCE_H_
#define GRIPPER_OBJECT_AT_CORRECT_DISTANCE_H_

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <distance_interfaces/srv/get_front_distance.hpp>

#include <iostream>

class DistanceSensorServer : public rclcpp::Node {
public:
    DistanceSensorServer(): Node("distance_sensor_server") {
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&DistanceSensorServer::scan_callback, this, std::placeholders::_1));
        service = this->create_service<distance_interfaces::srv::GetFrontDistance>
            ("get_front_distance", std::bind(&DistanceSensorServer::get_front_distance_callback, this, std::placeholders::_1, std::placeholders::_2));
        forwardSum = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetFrontDistance server started. Ready for requests.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & data) {
        forwardSum = data.ranges.at(360 - 1) + data.ranges.at(360 - 2) + data.ranges.at(360 - 3) + data.ranges.at(360 - 4) + data.ranges.at(360 - 5);
    }

    void get_front_distance_callback(const std::shared_ptr<distance_interfaces::srv::GetFrontDistance::Request> request,
          std::shared_ptr<distance_interfaces::srv::GetFrontDistance::Response> response) {
        response->front_distance = forwardSum / 5;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Service<distance_interfaces::srv::GetFrontDistance>::SharedPtr service;
    float forwardSum;
};

#endif
