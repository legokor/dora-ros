#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "controller/uart_handler.hpp"

#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include <thread>
#include <atomic>

namespace dora {

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode();
    ~ControllerNode();

private:
    UARTHandler uart;

    std::atomic<bool> io_thread_running;
    std::thread io_thread;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr imu_publisher;

    void sendTwist(const geometry_msgs::msg::Twist::SharedPtr& msg);
    void handleReceivedMessage(const dora::ReceivedMessage& msg);
};

} // namespace dora

#endif
