#ifndef OBJECT_AT_CENTER_H_
#define OBJECT_AT_CENTER_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_yellow.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Condition Node to check if the user wants coffee
class ObjectAtCenter : public BT::ConditionNode {
public:
    ObjectAtCenter(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("frame_width"), BT::InputPort<int>("cx")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<int> cxOptional = getInput<int>("cx");

        if (!cxOptional) {
            throw BT::RuntimeError("Invalid required input message: ", cxOptional.error());
        }

        BT::Optional<int> frameWidthOptional = getInput<int>("frame_width");

        if (!frameWidthOptional) {
            throw BT::RuntimeError("Invalid required input message: ", frameWidthOptional.error());
        }

        int cx = cxOptional.value();
        int frame_width = frameWidthOptional.value();

        int frame_center = frame_width / 2;
        int error_x = cx - frame_center;

        // Rotation direction based on error
        if (std::abs(error_x) > 20)
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