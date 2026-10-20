#ifndef DIS_CON
#define DIS_CON

#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

using TwistStamped = geometry_msgs::msg::TwistStamped;
using LaserScan = sensor_msgs::msg::LaserScan;

using namespace rclcpp;

class Distance_Control_Node : public Node {
    
    // Ros2 communication
    Subscription<LaserScan>::SharedPtr scan_subscriber;
    Publisher<TwistStamped>::SharedPtr twist_publisher;
    
    // Parameters.
    float minDistance = declare_parameter<float>("minimum_distance", 0.5);
    float maxDistance = declare_parameter<float>("maximum_distance", 1.0);
    
    public:
    
	Distance_Control_Node();
	~Distance_Control_Node();
    
	/* Also calls the other 2 functions during processing
	 * Then publishes the message. */
	void ProcessLaserScan(LaserScan::SharedPtr& msg);
	
	/* It is minimal when an obstacle is at maxDistance or above
	 * and maximal when it is at minDistance or below */
	float CalcSpeed(float distance);
	
	/* Doesn't publish the message */
	void SendTwist(float degree, float magnitude);
};

#endif
