#include <yellow_ball_finder/yellow_ball_action_server.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YellowBallActionServer>());
    rclcpp::shutdown();
    return 0;
}