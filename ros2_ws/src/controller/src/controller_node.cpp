#include "controller/controller_node.hpp"

#include <termios.h>
#include <expected>
#include <string>
#include <variant>

using Twist = geometry_msgs::msg::Twist;

using namespace dora;

ControllerNode::ControllerNode() : Node("uart_handler_node"), uart("/dev/ttyUSB1", B115200) {

    // Initialising publishers and subscribers
    velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, [this](Twist::SharedPtr t) { sendTwist(t); });
    twist_publisher = create_publisher<Twist>("encoder_speed", 10);
    
    // Starting IO thread
    io_thread = std::thread([this]() {
        io_thread_running = true;
        while (io_thread_running) {
            std::expected<ReceivedMessage, std::string> msg = uart.receiveMessage();
            if (msg)
                handleReceivedMessage(*msg);
            else
                RCLCPP_ERROR(get_logger(), "UART read failed: %s", msg.error().c_str());
        }
    });

    RCLCPP_INFO(get_logger(), "controller node initialized succesfully");
}

void ControllerNode::sendTwist(const Twist::SharedPtr& msg) {
    uart.sendSpeedCommand(msg->linear.x, msg->linear.y, msg->angular.z);
}

void ControllerNode::publishMeasure(const SpeedData& msg) {
    auto [x, y, th] = msg;

    auto newmsg = Twist();
    newmsg.linear.x = x;
    newmsg.linear.y = y;
    newmsg.angular.z = th;

    twist_publisher->publish(newmsg);
}

void ControllerNode::handleReceivedMessage(const ReceivedMessage& msg) {
    if (std::holds_alternative<SpeedData>(msg)) {
        publishMeasure(std::get<SpeedData>(msg));
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
