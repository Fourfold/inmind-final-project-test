#ifndef OBJECT_ON_RIGHT_H_
#define OBJECT_ON_RIGHT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_yellow.hpp>

// Condition Node to check if the user wants coffee
class ObjectOnRight : public BT::ConditionNode {
public:
    ObjectOnRight(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("frame_width"), BT::InputPort<int>("cx"), BT::OutputPort<int>("error_x")};
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
        if (error_x >= 20)
        {
            setOutput("error_x", std::abs(error_x));
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
};

#endif