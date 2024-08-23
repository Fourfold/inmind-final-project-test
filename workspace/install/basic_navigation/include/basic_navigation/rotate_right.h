#ifndef ROTATE_RIGHT_H_
#define ROTATE_RIGHT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Action Node to move forward
class RotateRight : public BT::SyncActionNode, public rclcpp::Node {
public:
    RotateRight(const std::string& name) : BT::SyncActionNode(name, {}), rclcpp::Node("rotate_right") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    BT::NodeStatus tick() override {
        geometry_msgs::msg::Twist msg;
        msg.angular.z = -0.25;
        cmd_vel_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};


#endif