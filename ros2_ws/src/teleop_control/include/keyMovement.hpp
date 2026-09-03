#ifndef KEYMOV
#define KEYMOV

#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "teleop_control/msg/key_input_msg.hpp"
#include <cmath>

using namespace rclcpp;
using KeyMsg = teleop_control::msg::KeyInputMsg;
using Twist = geometry_msgs::msg::Twist;

/* 
 * This node publishes Twist messages based on keyboard control messages
 * ROS Parameters:
 * @param absSpeedLimit: The absolute limit of linear and stafing speed
 * @param absAngLimit: The absolute limit of angular speed
 * @param linAccel: Increases absolute linear and strafing speed by when receiving a message by the amount
 * @param angAccel: Increases absolute angular speed by when receiving a message by the amount
 * @param deaccel: Decrease absolute speed (both linear and angular) by when publishing a new message
*/
class KeyMovementNode : public Node {
	// ROS tools
    Subscription<KeyMsg>::SharedPtr key_subscriber;
    Publisher<Twist>::SharedPtr movement_publisher;
    TimerBase::SharedPtr timer;
    
    // Parameters
    double absSpeedLimit = declare_parameter<double>("absSpeedLimit", 1.0);
    double absAngLimit = declare_parameter<double>("absAngLimit", 1.0);
    double linAccel = declare_parameter<double>("linAccel", 0.1);
    double angAccel = declare_parameter<double>("angAccel", 0.1);
    double deaccel = declare_parameter<double>("deaccel", 0.01);
    
    // Variables
    double linSpeed = 0.0;
    double strafeSpeed = 0.0;
    double angSpeed = 0.0;
    
    // Functions
    
    /* Changes the published speed data according to input
     * @param msg: The message containing the currently pressed key
    */
    void ChangeSpeed(const KeyMsg::SharedPtr& msg);
    
    /* Publishes speed data for the robot controller and deacceleretes the robot's speed
    */
    void PublishSpeed();
	
    public:
		
        KeyMovementNode();
        ~KeyMovementNode();
		
};

#define _USE_MATH_DEFINES
#endif
