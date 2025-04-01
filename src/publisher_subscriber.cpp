#include <mutex>
#include "controller/uart_handler.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <vector>
#include <chrono>
#include <memory>

class UARTHandlerNode : public rclcpp::Node {
public:
    // TODO: usb port
    UARTHandlerNode() : Node("uart_handler_node"), uart("/dev/ttyTHS1", B115200) {
        // UART port and baudrate TODO: check!
        // Publisher for received speed data
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("current_velocity", 10);

        // Subscriber for sending speed commands
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "target_velocity", 10, std::bind(&UARTHandlerNode::sendTwist, this, std::placeholders::_1));

        // Timer to read UART data every 50ms
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), // TODO is 50ms OK?
                                         std::bind(&UARTHandlerNode::readUART, this));
    }

private:
    UARTHandler uart;
    std::mutex uart_mutex; // Prevents concurrent access

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    void sendTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(uart_mutex);
        uart.sendSpeedCommand(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void readUART() {
        auto start = std::chrono::steady_clock::now(); // check if 50ms is OK

        std::lock_guard<std::mutex> lock(uart_mutex);

        float x, y, w;
        if (uart.receiveSpeedData(x, y, w)) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = x;
            twist_msg.linear.y = y;
            twist_msg.angular.z = w;
            twist_publisher_->publish(twist_msg);
        }

        auto end = std::chrono::steady_clock::now(); // End timing
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        RCLCPP_INFO(this->get_logger(), "UART read took %ld ms", duration.count());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTHandlerNode>());
    rclcpp::shutdown();

    return 0;
}
