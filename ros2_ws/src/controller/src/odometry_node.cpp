#include "controller/odometry_node.hpp"

#include <string>

using namespace dora;
using Twist = geometry_msgs::msg::Twist;

OdometryNode::OdometryNode() : Node("odometry_node") {
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    speed_subscriber = create_subscription<Twist>("encoder_speed", 10, [this](Twist::SharedPtr msg) {odomUpdate(msg);});

    // Starting time measurement
    last_time = this->get_clock()->now();
}

void OdometryNode::sendTransform(const rclcpp::Time& current_time) {
        // Initializing message
        geometry_msgs::msg::TransformStamped msg;

        // Setting header (Time is also important)
        msg.header.stamp = current_time;
        msg.header.frame_id = "odom";
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

void OdometryNode::sendOdometry(const rclcpp::Time& current_time, const Twist::SharedPtr& speedData) {
        // Initializing message
        nav_msgs::msg::Odometry msg;

        // Setting header (Time is also important)
        msg.header.stamp = current_time;
        msg.header.frame_id = "odom";
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
        msg.twist.twist.linear.x = speedData->linear.x;
        msg.twist.twist.linear.y = speedData->linear.y;
        msg.twist.twist.angular.z = speedData->angular.z;

        // TODO: Covarience matrix if the data is noisy and Dora doesn't move smoothly
        
        // Publishing message
        odom_publisher->publish(msg);

        // Logging
        std::stringstream ss;
        ss << "Odometry:\n" <<
            "- Pose: " << x_total << " ; " << y_total << " ; " << th_total << "\n" <<
            "- Velocity: " << speedData->linear.x << " ; " << speedData->linear.y << " ; " << speedData->angular.z << "\n";
        RCLCPP_INFO(get_logger(), ss.str().c_str());
}

void OdometryNode::odomUpdate(const Twist::SharedPtr& speedData) {

    // Getting the elapsed time from the last measurement
    rclcpp::Time current_time = this->get_clock()->now();
    double delta_time = (current_time - last_time).seconds();

    // Extracting speed data
    double vx = speedData->linear.x;
    double vy = speedData->linear.y;
    double vth = speedData->angular.z;

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

    // Setting time:
    last_time = current_time;
}

OdometryNode::~OdometryNode() {
    // Empty for now.
}
