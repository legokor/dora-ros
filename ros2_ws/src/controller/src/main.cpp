#include "controller/controller_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // we only create publishers, subscribers and timers during initialization
    rclcpp::executors::SingleThreadedExecutor executor;

    // must be declared as a variable to not be instantly dropped...
    auto controller_node = std::make_shared<dora::ControllerNode>();
    executor.add_node(controller_node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
