#ifndef YELLOW_BALL_AT_CENTER_H_
#define YELLOW_BALL_AT_CENTER_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_yellow.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Condition Node to check if the user wants coffee
class YellowBallAtCenter : public BT::ConditionNode {
public:
    YellowBallAtCenter(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("cx")};
    }

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "At center called.");

        BT::Optional<int> cxOptional = getInput<int>("cx");

        if (!cxOptional) {
            throw BT::RuntimeError("Invalid required input message: ", cxOptional.error());
        }

        int cx = cxOptional.value();

        int frame_center = 1000 / 2; // TODO: change
        int error_x = cx - frame_center;

        // Rotation direction based on error
        if (std::abs(error_x) > 100)
        {
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
};

#endif