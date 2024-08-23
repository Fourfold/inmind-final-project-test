#ifndef FIND_YELLOW_BALL_H_
#define FIND_YELLOW_BALL_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <detection_interfaces/action/find_yellow.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using FindYellow = detection_interfaces::action::FindYellow;
using GoalHandleFindYellow = rclcpp_action::ClientGoalHandle<FindYellow>;

// Action Node to move forward
class FindYellowBall : public BT::SyncActionNode, public rclcpp::Node {
public:
    FindYellowBall(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config), rclcpp::Node("find_yellow_ball") {
        this->action_client = rclcpp_action::create_client<FindYellow>(
            this,
            "yellow_ball"
        );
        using namespace std::placeholders;
        send_goal_options = rclcpp_action::Client<FindYellow>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(
            &FindYellowBall::goal_response_callback,
            this,
            _1
        );
        send_goal_options.feedback_callback = std::bind(
            &FindYellowBall::feedback_callback,
            this,
            _1,
            _2
        );
        send_goal_options.result_callback = std::bind(
            &FindYellowBall::result_callback,
            this,
            _1
        );
        receivedResult = false;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<int>("cx"), BT::OutputPort<int>("cy")};
    }

    BT::NodeStatus tick() override {
        auto goal_msg = FindYellow::Goal();
        auto result = action_client->async_send_goal(goal_msg, send_goal_options);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

        if (receivedResult) {
            setOutput("cx", cx);
            setOutput("cy", cy);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    void goal_response_callback(const GoalHandleFindYellow::SharedPtr & goal_handle) {
        // if (!goal_handle) {
        //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected.");
        // } else {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted.");
        // }
    }

    void feedback_callback(
        GoalHandleFindYellow::SharedPtr,
        const std::shared_ptr<const FindYellow::Feedback> feedback
    ) { }

    void result_callback(const GoalHandleFindYellow::WrappedResult & result) {
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
        cx = result.result->cx;
        cy = result.result->cy;
        if (receivedResult) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received result: FOUND");
        }
    }

    rclcpp_action::Client<FindYellow>::SendGoalOptions send_goal_options;
    rclcpp_action::Client<FindYellow>::SharedPtr action_client;
    bool receivedResult;
    int cx;
    int cy;
};


#endif