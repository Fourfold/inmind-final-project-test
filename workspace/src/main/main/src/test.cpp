#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TestNode : public rclcpp::Node {
public:
    TestNode() : rclcpp::Node("test_node") {
        test_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>
            ("/cmd_vel", 10, std::bind(&TestNode::brew_coffee, this, std::placeholders::_1));
    }
private:
    void brew_coffee(const geometry_msgs::msg::Twist & msg) { }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr test_subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();

    return 0;
}