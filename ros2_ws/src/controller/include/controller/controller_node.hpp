#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "controller/uart_handler.hpp"

#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <thread>
#include <atomic>

namespace dora {

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode();
    ~ControllerNode();

private:

    // Odometry data:
    float x_total = 0;
    float y_total = 0;
    float th_total = 0;
    rclcpp::Time last_time;

    UARTHandler uart;

    std::atomic<bool> io_thread_running;
    std::thread io_thread;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher; // Eredetileg imu_publisher, de nem enkóderből mértet küld vissza?
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    void sendTwist(const geometry_msgs::msg::Twist::SharedPtr& msg);
    void handleReceivedMessage(const dora::ReceivedMessage& msg);
    void odomUpdate(const dora::SpeedData& speedData);
    void sendTransform(const rclcpp::Time& current_time);
    void sendOdometry(const rclcpp::Time& current_time, const dora::SpeedData& speedData);
};

} // namespace dora

#endif
