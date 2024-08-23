#include <basic_navigation/move_forward.h>
#include <basic_navigation/rotate_right.h>
#include <basic_navigation/move_forward_right.h>
#include <basic_navigation/halt.h>
#include <yellow_ball_finder/find_yellow_ball.h>
#include <yellow_ball_finder/yellow_ball_at_center.h>
#include <main/wait.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

class MainNode : public rclcpp::Node {
public:
    MainNode() : rclcpp::Node("main_node") {
        // Register nodes
        factory_.registerNodeType<FindYellowBall>("FindYellowBall");
        factory_.registerNodeType<RotateRight>("RotateRight");
        factory_.registerNodeType<MoveForward>("MoveForward");
        factory_.registerNodeType<MoveForwardRight>("MoveForwardRight");
        factory_.registerNodeType<YellowBallAtCenter>("YellowBallAtCenter");
        factory_.registerNodeType<Halt>("Halt");
        factory_.registerNodeType<Wait>("Wait");
        auto blackboard = BT::Blackboard::create({});
        blackboard->set("found", false);
        blackboard->set("cx", 0);
        blackboard->set("cy", 0);
        auto tree = factory_.createTreeFromFile("./src/main/main/include/main/main_behavior_tree.xml", blackboard);
        BT::NodeStatus status = tree.tickRoot();
    }
private:
    BT::BehaviorTreeFactory factory_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());

    return 0;
}