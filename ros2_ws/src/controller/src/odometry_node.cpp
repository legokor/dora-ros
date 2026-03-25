#include "controller/odometry_node.hpp"

#include <string>

using namespace dora;
using TwistStamped = geometry_msgs::msg::TwistStamped;

OdometryNode::OdometryNode() : Node("odometry_node") {
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    speed_subscriber = create_subscription<TwistStamped>("encoder_speed", 10, [this](TwistStamped::SharedPtr msg) {odomUpdate(msg);});

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

void OdometryNode::sendOdometry(const TwistStamped::SharedPtr& speedData) {
        // Initializing message
        nav_msgs::msg::Odometry msg;

        // Setting header (Time is also important)
        msg.header.stamp = speedData->header.stamp;
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
        msg.twist.twist.linear.x = speedData->twist.linear.x;
        msg.twist.twist.linear.y = speedData->twist.linear.y;
        msg.twist.twist.angular.z = speedData->twist.angular.z;

        // TODO: Calculate proper covariance matrix
        msg.pose.covariance = {
            0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  // X 
            0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  // Y
            0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,  // Z
            0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,  // Roll
            0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,  // Pitch
            0.0,  0.0,  0.0,  0.0,  0.0,  0.03  // Yaw
        };

        msg.twist.covariance = {
            0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  // X 
            0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  // Y
            0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,  // Z
            0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,  // Roll
            0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,  // Pitch
            0.0,  0.0,  0.0,  0.0,  0.0,  0.03  // Yaw
        };
        
        // Publishing message
        odom_publisher->publish(msg);

        // Logging
        std::stringstream ss;
        ss << "Odometry:\n" <<
            "- Pose: " << x_total << " ; " << y_total << " ; " << th_total << "\n" <<
            "- Velocity: " << speedData->twist.linear.x << " ; " << speedData->twist.linear.y << " ; " << speedData->twist.angular.z << "\n";
        RCLCPP_INFO(get_logger(), ss.str().c_str());
}

void OdometryNode::odomUpdate(const TwistStamped::SharedPtr& speedData) {
    // Getting the elapsed time from the last measurement
    rclcpp::Time current_time = speedData->header.stamp;
    double delta_time = (current_time - last_time).seconds();

    // Extracting speed data
    double vx = speedData->twist.linear.x;
    double vy = speedData->twist.linear.y;
    double vth = speedData->twist.angular.z;

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
    sendOdometry(speedData);

    // Setting time:
    last_time = current_time;
}

OdometryNode::~OdometryNode() {
    // Empty for now.
}
