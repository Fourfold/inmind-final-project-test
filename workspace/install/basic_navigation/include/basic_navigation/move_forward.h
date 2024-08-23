#ifndef MOVE_FORWARD_H_
#define MOVE_FORWARD_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Action Node to move forward
class MoveForward : public BT::SyncActionNode, public rclcpp::Node {
public:
    MoveForward(const std::string& name) : BT::SyncActionNode(name, {}), rclcpp::Node("move_forward") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    BT::NodeStatus tick() override {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.3;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        cmd_vel_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};


#endif