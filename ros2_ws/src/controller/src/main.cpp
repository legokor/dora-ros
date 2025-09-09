#include "controller/controller_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // we only create publishers, subscribers and timers during initialization
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<dora::ControllerNode>());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
