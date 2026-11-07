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
    
    // Parameters
    float minDistance = declare_parameter<float>("minimum_distance", 0.5f);
    float maxDistance = declare_parameter<float>("maximum_distance", 1.0f);
    float clearLimit = declare_parameter<float>("clear_Limit", 0.1f);
    
    // Values:
    bool isOn = true;
    
    public:
    
	Distance_Control_Node();
	~Distance_Control_Node();
    
	// Also calls the other 2 functions during processing.
	void ProcessLaserScan(LaserScan::SharedPtr& msg);
	
	inline float CalcDegree(float degree);
	
	inline float CalcSpeed(float distance);
	
	void SendTwist(float degree, float magnitude);
	
	inline void SwitchToggle(bool state);
};

#endif
