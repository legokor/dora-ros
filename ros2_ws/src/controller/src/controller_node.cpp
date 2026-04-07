#include "controller/controller_node.hpp"

#include <asm-generic/errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <expected>
#include <string>
#include <variant>

// #define DEBUG_WITHOUT_UART

using Twist = geometry_msgs::msg::Twist;

using namespace dora;

ControllerNode::ControllerNode() : Node("uart_handler_node"), uart("/dev/ttyUSB1", B115200) {
    velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, [this](Twist::SharedPtr t) { sendTwist(t); });
    imu_publisher = create_publisher<Twist>("odom", 10);

    std::cerr<<io_thread_running<<std::endl;

    io_thread_running = true;
    io_thread = std::thread([this]() {
        while (io_thread_running) {
            std::expected<ReceivedMessage, std::string> msg = uart.receiveMessage();

            if (msg){
                handleReceivedMessage(*msg);
            }
            else{
            	// if error message is "no new message from uart", don't stop WHEN DEBUGGING
            	if(strcmp(msg.error().c_str(),"No new messages on UART") == 0){
             		RCLCPP_ERROR(get_logger(), "Stopped receiving messages from UART.");

               		// Ha nem debug, ennél ugyan úgy megáll
               		#ifdef DEBUG_WITHOUT_UART
                    break;
                 	#endif
             	}else{
	                RCLCPP_ERROR(get_logger(), "UART read failed: %s", msg.error().c_str());
					// When debugging, dont stop on error
					#ifndef DEBUG_WITHOUT_UART
	                	break;
					#endif
              	}
            }
        }
    });
    // io_thread.join();
    RCLCPP_INFO(get_logger(), "controller node initialized succesfully");
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
