#ifndef MOVE_FORWARD_RIGHT_H_
#define MOVE_FORWARD_RIGHT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Action Node to move forward
class MoveForwardRight : public BT::SyncActionNode, public rclcpp::Node {
public:
    MoveForwardRight(const std::string& name) : BT::SyncActionNode(name, {}), rclcpp::Node("move_forward_right") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    BT::NodeStatus tick() override {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.2;
        msg.angular.z = -0.2;
        cmd_vel_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};


#endif