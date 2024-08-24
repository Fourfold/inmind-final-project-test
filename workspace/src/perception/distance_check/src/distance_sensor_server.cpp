#include <distance_check/distance_sensor_server.h>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceSensorServer>());
    rclcpp::shutdown();

    return 0;
}