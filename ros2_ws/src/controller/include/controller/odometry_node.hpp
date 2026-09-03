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

#include <string>

// Robot specific namespace
namespace dora {

	/*
	 * Calculates the robot's odometry and publishes it. 
	 * The odometry is a position estimated from IMU and wheel encoder data
	 * (Current the robot only calculates from encoder data)
	*/ 
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

        // Ros communication:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

		/* Broadcasts transform (positional and rotational) data with timestamp. 
		 * Also logs received messages.
		 * @param current_time: The moment sensor data was read. Currently this timepoint is tied
		 * 						to inside the ControllerNode's publishMeasure function.
		*/
        void sendTransform(const rclcpp::Time& current_time);
        
        /* Publishes odometry data. Also logs messages
         * @param speedData: A TwistStamped message to transform into an Odometery message
        */
        void sendOdometry(const geometry_msgs::msg::TwistStamped::SharedPtr& speedData);
        
        /* Updates the odometry based on received measurements.
         * @param speedData: A TwistStamped message containing the new measurements 
        */
        void odomUpdate(const geometry_msgs::msg::TwistStamped::SharedPtr& speedData);
};

}

#endif
