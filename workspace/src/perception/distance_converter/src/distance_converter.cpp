#include <distance_converter/distance_converter.h>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceConverter>());
    rclcpp::shutdown();

    return 0;
}