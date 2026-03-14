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
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Starting IO thread
    io_thread = std::thread([this]() {
        while (io_thread_running) {
            std::expected<ReceivedMessage, std::string> msg = uart.receiveMessage();

            if (msg)
                handleReceivedMessage(*msg);
            else
                RCLCPP_ERROR(get_logger(), "UART read failed: %s", msg.error().c_str());
        }
    });

    // Starting measurement time
    last_time = this->get_clock()->now();

    RCLCPP_INFO(get_logger(), "controller node initialized succesfully");
}

void ControllerNode::sendTwist(const Twist::SharedPtr& msg) {
    uart.sendSpeedCommand(msg->linear.x, msg->linear.y, msg->angular.z);
}

void ControllerNode::handleReceivedMessage(const ReceivedMessage& msg) {
    if (std::holds_alternative<SpeedData>(msg)) {
        odomUpdate(std::get<SpeedData>(msg));
    } else if (std::holds_alternative<RobotStatus>(msg)) {
        auto [voltage] = std::get<RobotStatus>(msg);
        RCLCPP_INFO(get_logger(), "got robot status: v:%f", voltage);
    }
}

void ControllerNode::odomUpdate(const SpeedData& speedData) {

    // Getting the elapsed time from the last measurement
    rclcpp::Time current_time = this->get_clock()->now();
    double delta_time = (current_time - last_time).seconds();

    // Extracting speed data
    auto [vx, vy, vth] = speedData;

    // Coordinate transform
    double delta_th = vth * delta_time;
    double mid_th = th_total + delta_th / 2.0; // The average of the time interval
    double delta_x = (vx * cos(mid_th) - vy * sin(mid_th)) * delta_time;
    double delta_y = (vx * sin(mid_th) + vy * cos(mid_th)) * delta_time;

    x_total += delta_x;
    y_total += delta_y;
    th_total += delta_th;

    // Sending data:
    sendTransform(current_time);
    sendOdometry(current_time, speedData);
}

void ControllerNode::sendTransform(const rclcpp::Time& current_time) {
        // Initializing message
        geometry_msgs::msg::TransformStamped msg;

        // Setting header (Time is also important)
        msg.header.stamp = current_time;
        msg.header.frame_id = "world";
        msg.child_frame_id = "base_footprint";

        // Setting positional data
        msg.transform.translation.x = x_total;
        msg.transform.translation.y = y_total;
        msg.transform.translation.z = 0.0;

        // Rotational data
        tf2::Quaternion q;
        q.setRPY(0, 0, th_total);
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();
        
        // Publishing message
        tf_broadcaster->sendTransform(msg);

        // Logging
        std::stringstream ss;
        ss << "Transformation:\n" <<
            "- Pose: " << x_total << " ; " << y_total << " ; " << th_total << "\n";
        RCLCPP_INFO(get_logger(), ss.str().c_str());
}

void ControllerNode::sendOdometry(const rclcpp::Time& current_time, const SpeedData& speedData) {
        // Initializing message
        nav_msgs::msg::Odometry msg;

        // Setting header (Time is also important)
        msg.header.stamp = current_time;
        msg.header.frame_id = "world";
        msg.child_frame_id = "base_footprint";

        // Setting positional data
        msg.pose.pose.position.x = x_total;
        msg.pose.pose.position.y = y_total;
        msg.pose.pose.position.z = 0;

        // Rotational data
        tf2::Quaternion q;
        q.setRPY(0, 0, th_total);
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Velocity data
        auto [vx, vy, vth] = speedData;
        msg.twist.twist.linear.x = vx;
        msg.twist.twist.linear.y = vy;
        msg.twist.twist.angular.z = vth;

        // TODO: Covarience matrix if the data is noisy and Dora doesn't move smoothly
        
        // Publishing message
        odom_publisher->publish(msg);

        // Logging
        std::stringstream ss;
        ss << "Odometry:\n" <<
            "- Pose: " << x_total << " ; " << y_total << " ; " << th_total << "\n" <<
            "- Velocity: " << vx << " ; " << vy << " ; " << vth << "\n";
        RCLCPP_INFO(get_logger(), ss.str().c_str());
}

ControllerNode::~ControllerNode() {
    io_thread_running = false;
    if (io_thread.joinable())
        io_thread.join();
}
