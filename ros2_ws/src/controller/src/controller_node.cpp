#include "controller/controller_node.hpp"

#include <termios.h>
#include <expected>
#include <string>
#include <variant>

using Twist = geometry_msgs::msg::Twist;

using namespace dora;

ControllerNode::ControllerNode() : Node("uart_handler_node"), uart("/dev/ttyUSB1", B115200) {
    velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, [this](Twist::SharedPtr t) { sendTwist(t); });
    imu_publisher = create_publisher<Twist>("odom", 10);

    io_thread = std::thread([this]() {
        while (io_thread_running) {
            std::expected<ReceivedMessage, std::string> msg = uart.receiveMessage();

            if (msg)
                handleReceivedMessage(*msg);
            else
                RCLCPP_ERROR(get_logger(), "UART read failed: %s", msg.error().c_str());
        }
    });
}

void ControllerNode::sendTwist(const Twist::SharedPtr& msg) {
    uart.sendSpeedCommand(msg->linear.x, msg->linear.y, msg->angular.z);
}

void ControllerNode::handleReceivedMessage(const ReceivedMessage& msg) {
    if (std::holds_alternative<SpeedData>(msg)) {
        auto [x, y, w] = std::get<SpeedData>(msg);

        Twist twist;
        twist.linear.x = x;
        twist.linear.y = y;
        twist.angular.z = w;
        imu_publisher->publish(twist);

        RCLCPP_INFO(get_logger(), "got speed data: x:%f y:%f w:%f", x, y, w);
    } else if (std::holds_alternative<RobotStatus>(msg)) {
        auto [voltage] = std::get<RobotStatus>(msg);
        RCLCPP_INFO(get_logger(), "got robot status: v:%f", voltage);
    }
}

ControllerNode::~ControllerNode() {
    io_thread_running = false;
    if (io_thread.joinable())
        io_thread.join();
}
