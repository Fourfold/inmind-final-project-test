#ifndef ENSURE_DETECTION_READY_H_
#define ENSURE_DETECTION_READY_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_object.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using FindObjectAction = detection_interfaces::action::FindObject;
using GoalHandleFindObject = rclcpp_action::ClientGoalHandle<FindObjectAction>;

// Action Node to ensure detection model is ready
class EnsureDetectionReady : public BT::SyncActionNode, public rclcpp::Node {
public:
    EnsureDetectionReady(const std::string& name) : BT::SyncActionNode(name, {}), rclcpp::Node("ensure_detection_ready") {

        ready = false;

        this->action_client = rclcpp_action::create_client<FindObjectAction>(
            this,
            "find_object_server"
        );
        using namespace std::placeholders;
        send_goal_options = rclcpp_action::Client<FindObjectAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(
            &EnsureDetectionReady::goal_response_callback,
            this,
            _1
        );
        send_goal_options.feedback_callback = std::bind(
            &EnsureDetectionReady::feedback_callback,
            this,
            _1,
            _2
        );
        send_goal_options.result_callback = std::bind(
            &EnsureDetectionReady::result_callback,
            this,
            _1
        );
    }

    BT::NodeStatus tick() override {
        auto goal_msg = FindObjectAction::Goal();
        goal_msg.object_type = "test";

        while (!action_client->action_server_is_ready()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        auto result = action_client->async_send_goal(goal_msg, send_goal_options);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

        return ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(const GoalHandleFindObject::SharedPtr & goal_handle) { }

    void feedback_callback(
        GoalHandleFindObject::SharedPtr,
        const std::shared_ptr<const FindObjectAction::Feedback> feedback
    ) { }

    void result_callback(const GoalHandleFindObject::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal canceled.");
                return;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unkown result.");
                return;
        }
        ready = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object detection server ready.");
    }

    rclcpp_action::Client<FindObjectAction>::SendGoalOptions send_goal_options;
    rclcpp_action::Client<FindObjectAction>::SharedPtr action_client;
    bool ready;
};


#endif