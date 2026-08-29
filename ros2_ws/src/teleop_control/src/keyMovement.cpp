#include "keyMovement.hpp"

using Twist = geometry_msgs::msg::Twist;
using KeyMsg = keyMovement::msg::KeyInputMsg;
using namespace rclcpp;
using namespace std::chrono_literals;

// Constructor
KeyMovementNode::KeyMovementNode() : Node("key_movement_node") {
    key_subscriber = create_subscription<KeyMsg>("key_teleop_control", 10,
			[this](KeyMsg::SharedPtr msg) {Subscription_callback(msg);});
    movement_publisher = create_publisher<Twist>("cmd_vel", 10);
			timer = create_wall_timer(50ms, [this]() {Publisher_callback();})
}

// Reacts to key input and changes the robot's speed accordingly
void KeyMovementNode::Subscription_callback(const KeyMsg::SharedPtr msg) {
	// Reacting to key input
	switch(msg->key) {
		case 'w': linSpeed += linAccel; break;
		case 'a': strafeSpeed -= linAccel; break;
		case 's': linSpeed -= linAccel; break;
		case 'd': strafeSpeed += linAccel; break;
		case 'q': angSpeed -= angAccel; break;
		case 'e': angSpeed += angAccel; break;
	}
    
    // Clamping to limit
    linSpeed = std::clamp(linSpeed, -absSpeedLimit, absSpeedLimit);
	stafeSpeed = std::clamp(stafeSpeed, -absSpeedLimit, absSpeedLimit);
    angSpeed = std::clamp(angSpeed, -absAngLimit, absAngLimit);
}

// Publishes speed data for the robot controller and deacceleretes the robot's speed
void KeyMovementNode::Publisher_callback() {
	// Clearing data to stop the wheels
    if (abs(linSpeed) < 0.01) linSpeed = 0.0;
    if (abs(stafeSpeed) < 0.01) strafeSpeed = 0.0;
    if (abs(angSpeed) < 0.01) angSpeed = 0.0;
    
    // Deacceleration
    linSpeed -= deaccel;
    strafeSpeed -= deaccel;
    angSpeed -= deaccel;
    
    // Generating message
	auto new_msg = Twist();
    new_msg.linear.x = linSpeed;
    new_msg.linear.y = strafeSpeed;
    new_msg.angular.z = angSpeed;
    
    // Publishing
    movement_publisher->publish(new_msg);
}

// Initilise then spin
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

	// Initialising then spinning
    auto key_movement_node = std::make_shared<KeyMovementNode>();
    rclcpp::spin(drift_node);

    rclcpp::shutdown();
    return 0;
}
