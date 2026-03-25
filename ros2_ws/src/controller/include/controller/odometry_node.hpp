#ifndef ODOMETRY_NODE
#define ODOMTERY_NODE

#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dora {

    class OdometryNode : public rclcpp::Node {

    public:

        OdometryNode();
        ~OdometryNode();

    private:

        // Odometry data:
        float x_total = 0;
        float y_total = 0;
        float th_total = 0;
        rclcpp::Time last_time;

        // Communication
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        void sendTransform(const rclcpp::Time& current_time);
        void sendOdometry(const geometry_msgs::msg::TwistStamped::SharedPtr& speedData);
        void odomUpdate(const geometry_msgs::msg::TwistStamped::SharedPtr& speedData);
};

}

#endif