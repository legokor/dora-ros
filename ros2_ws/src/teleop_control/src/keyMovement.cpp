#include "keyMovement.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>

using namespace rclcpp;
using namespace std::chrono_literals;

// Constructor
KeyMovementNode::KeyMovementNode() : Node("key_movement_node") {
    key_subscriber = create_subscription<!!!>("!!!", 10,
			[this](!!! msg) {ChangeSpeed(msg);});
    movement_publisher = create_publisher<>("!!!", 10);
	timer = create_wall_timer(50ms, [this](){!!!});
}

// Reacts to key input and changes the robot's speed accordingly
void KeyMovementNode::ChangeSpeed(!!! msg) {
	// Reacting to key input
	for (auto key_code : msg->keys) {
		switch(key_code) {
			case 'w': linSpeed += linAccel; break;
			!!!
		}
	}
	
    // Clamping to limit
    linSpeed = std::clamp(linSpeed, -absSpeedLimit, absSpeedLimit);
	strafeSpeed = std::clamp(strafeSpeed, -absSpeedLimit, absSpeedLimit);
    angSpeed = std::clamp(angSpeed, -absAngLimit, absAngLimit);
}

// Publishes speed data for the robot controller and deacceleretes the robot's speed
void KeyMovementNode::PublishSpeed() {
    // Deacceleration
    linSpeed /= deaccel;
    strafeSpeed /= deaccel;
    angSpeed /= deaccel;
    
    // Clearing data to stop the wheels
    if (abs(linSpeed) < 0.001) linSpeed = 0.0;
    if (abs(strafeSpeed) < 0.001) strafeSpeed = 0.0;
    if (abs(angSpeed) < 0.001) angSpeed = 0.0;
    
    // Clamping to prevent UART errors
    linSpeed = std::clamp(linSpeed, -absSpeedLimit, absSpeedLimit);
	strafeSpeed = std::clamp(strafeSpeed, -absSpeedLimit, absSpeedLimit);
    angSpeed = std::clamp(angSpeed, -absAngLimit, absAngLimit);
    
    // Generating message
	!!!
    
    // Publishing
    !!!
}

KeyMovementNode::~KeyMovementNode() {
	// Empty for now
}

// Initilise then spin
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

	// Initialising then spinning
    auto key_movement_node = std::make_shared<!!!>();
    rclcpp::spin(!!!);

    rclcpp::shutdown();
    return 0;
}
