#ifndef GRIPPER_OBJECT_AT_CORRECT_DISTANCE_H_
#define GRIPPER_OBJECT_AT_CORRECT_DISTANCE_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <distance_interfaces/srv/get_front_distance.hpp>

// Condition Node to check if the object to be gripped is at the correct distance
class GripperObjectAtCorrectDistance : public BT::ConditionNode, public rclcpp::Node {
public:
    GripperObjectAtCorrectDistance(const std::string& name) : BT::ConditionNode(name, {}), rclcpp::Node("gripper_object_at_correct_distance") {
        client = this->create_client<distance_interfaces::srv::GetFrontDistance>("get_front_distance");
        request = std::make_shared<distance_interfaces::srv::GetFrontDistance::Request>();

        using namespace std::chrono_literals;
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }
    }

    BT::NodeStatus tick() override {
        if (getDistance() > 0.4) {
            return BT::NodeStatus::FAILURE;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Close distance.");
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    float getDistance() {
        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return result.get()->front_distance;
        } else {
            return 0.0;
        }
    }

    rclcpp::Client<distance_interfaces::srv::GetFrontDistance>::SharedPtr client;
    std::shared_ptr<distance_interfaces::srv::GetFrontDistance::Request> request;
};

#endif
