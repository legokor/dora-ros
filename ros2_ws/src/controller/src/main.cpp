#include "controller/controller_node.hpp"
#include "controller/odometry_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // We only create publishers, subscribers and timers during initialization
    rclcpp::executors::SingleThreadedExecutor executor;

    // Creating nodes and adding them to the executor. Nodes must be decleared.
    auto controller_node = std::make_shared<dora::ControllerNode>();
    executor.add_node(controller_node);
    auto odom_node = std::make_shared<dora::OdometryNode>();
    executor.add_node(odom_node);

	// Starting node execution
    executor.spin();

	// Shutting down.
    rclcpp::shutdown();

    return 0;
}
