#ifndef WAIT_H_
#define WAIT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>


// Action Node to move forward
class Wait : public BT::SyncActionNode, public rclcpp::Node {
public:
    Wait(const std::string& name) : BT::SyncActionNode(name, {}), rclcpp::Node("wait") {}
    BT::NodeStatus tick() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return BT::NodeStatus::SUCCESS;
    }
};


#endif