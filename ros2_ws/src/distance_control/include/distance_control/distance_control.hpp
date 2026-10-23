#ifndef DIS_CON
#define DIS_CON

#include "rclcpp/node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <math.h>

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using SetBool = std_srvs::srv::SetBool;

using namespace rclcpp;

class Distance_Control_Node : public Node {
    
    // Ros2 communication
    Subscription<LaserScan>::SharedPtr scan_subscriber;
    Publisher<Twist>::SharedPtr twist_publisher;
    Service<SetBool>::SharedPtr toggle_service;
    
    /* Parameters.
    * For me it makes more sense the mininum distance is actually lower,
    * but most of you refer to minimal_distance as the furthest distance
    * before the robot moves */
    float minDistance = declare_parameter<float>("minimum_distance", 0.5f);
    float maxDistance = declare_parameter<float>("maximum_distance", 1.0f);
    float clearLimit = declare_parameter<float>("clear_Limit", 0.1f);
    
    // Values:
    bool isOn = true;
    
    public:
    
	Distance_Control_Node();
	~Distance_Control_Node();
    
	/* Also calls the other 2 functions during processing.
	 * Contains the degree and distance selection logic in 
	 * case the current behaviour is unsatisfactory. */
	void ProcessLaserScan(LaserScan::SharedPtr& msg);
	
	/* Extracting degree calculation behaviour */
	inline float TransformDegree(float degree);
	
	/* It is minimal when an obstacle is at maxDistance or above
	 * and maximal when it is at minDistance or below */
	inline float CalcSpeed(float distance);
	
	/* Doesn't publish the message */
	void SendTwist(float degree, float magnitude);
	
	/* Service callback */
	inline void SwitchToggle(bool state);
};

#endif
