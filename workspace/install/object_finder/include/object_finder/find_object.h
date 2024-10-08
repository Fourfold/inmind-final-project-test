#ifndef FIND_OBJECT_H_
#define FIND_OBJECT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_object.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using FindObjectAction = detection_interfaces::action::FindObject;
using GoalHandleFindObject = rclcpp_action::ClientGoalHandle<FindObjectAction>;

// Action Node to move forward
class FindObject : public BT::SyncActionNode, public rclcpp::Node {
public:
    FindObject(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config), rclcpp::Node("find_object") {
        this->action_client = rclcpp_action::create_client<FindObjectAction>(
            this,
            "find_object_server"
        );
        using namespace std::placeholders;
        send_goal_options = rclcpp_action::Client<FindObjectAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(
            &FindObject::goal_response_callback,
            this,
            _1
        );
        send_goal_options.feedback_callback = std::bind(
            &FindObject::feedback_callback,
            this,
            _1,
            _2
        );
        send_goal_options.result_callback = std::bind(
            &FindObject::result_callback,
            this,
            _1
        );
        receivedResult = false;
        frame_width = 0;
        cx = 0;
        cy = 0;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("object_type"), BT::OutputPort<int>("frame_width"), BT::OutputPort<int>("cx"), BT::OutputPort<int>("cy")};
    }

    BT::NodeStatus tick() override {
        BT::Optional<std::string> typeOptional = getInput<std::string>("object_type");
        if (!typeOptional) {
            throw BT::RuntimeError("Invalid required input message: ", typeOptional.error());
        }
        
        auto goal_msg = FindObjectAction::Goal();
        goal_msg.object_type = typeOptional.value();

        auto result = action_client->async_send_goal(goal_msg, send_goal_options);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

        if (receivedResult) {
            setOutput<int>("frame_width", frame_width);
            setOutput<int>("cx", cx);
            setOutput<int>("cy", cy);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    void goal_response_callback(const GoalHandleFindObject::SharedPtr & goal_handle) {
        // if (!goal_handle) {
        //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected.");
        // } else {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted.");
        // }
    }

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
        receivedResult = result.result->found;
        frame_width = result.result->frame_width;
        cx = result.result->cx;
        cy = result.result->cy;
    }

    rclcpp_action::Client<FindObjectAction>::SendGoalOptions send_goal_options;
    rclcpp_action::Client<FindObjectAction>::SharedPtr action_client;
    bool receivedResult;
    int frame_width;
    int cx;
    int cy;
};


#endif
