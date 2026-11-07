#include "controller/controller_node.hpp"

#include <asm-generic/errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <expected>
#include <string>
#include <variant>

using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using namespace dora;

ControllerNode::ControllerNode() : Node("uart_handler_node"), uart("/dev/ttyUSB1", B115200) {

    // Initialising publishers and subscribers
    velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, [this](Twist::SharedPtr t) { sendTwist(t); });
    twist_publisher = create_publisher<TwistStamped>("encoder_speed", 10);
    
    // Starting IO thread
    io_thread = std::thread([this]() {
        io_thread_running = true;
        while (io_thread_running) {
            std::expected<ReceivedMessage, std::string> msg = uart.receiveMessage();
            if (msg) {
                handleReceivedMessage(*msg);
            } else {
                if (strcmp(msg.error().c_str(), "No new messages on UART") == 0) {
                    RCLCPP_ERROR(get_logger(), "Stopped receiving messages from UART.");
                    break;
                } else {
                    RCLCPP_ERROR(get_logger(), "UART read failed: %s", msg.error().c_str());
                }
            }
        }
    });

    RCLCPP_INFO(get_logger(), "Controller node initialized succesfully");
}

void ControllerNode::sendTwist(const Twist::SharedPtr& msg) {
    uart.sendSpeedCommand(msg->linear.x, msg->linear.y, msg->angular.z);
}

void ControllerNode::publishMeasure(const SpeedData& msg) {
    auto [x, y, th] = msg;

    auto newmsg = TwistStamped();
    newmsg.twist.linear.x = x;
    newmsg.twist.linear.y = y;
    newmsg.twist.angular.z = th;
    
    newmsg.header.stamp = this->get_clock()->now();

    twist_publisher->publish(newmsg);
}

// Currently only looks for SpeedData and RobotStatus messages
void ControllerNode::handleReceivedMessage(const ReceivedMessage& msg) {
    if (std::holds_alternative<SpeedData>(msg)) {
        publishMeasure(std::get<SpeedData>(msg));
    } else if (std::holds_alternative<RobotStatus>(msg)) {
        auto [voltage] = std::get<RobotStatus>(msg);
    }
}

ControllerNode::~ControllerNode() {
    io_thread_running = false;
    if (io_thread.joinable())
        io_thread.join();
}
