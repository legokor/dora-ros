#include "distance_control.hpp"

Distance_Control_Node::Distance_Control_Node() : Node("distance_controller") {
    twist_publisher = create_publisher<Twist>("/cmd_vel", 10);
    scan_subscriber = create_subscription<LaserScan>("/scan", 10,
	[this](LaserScan::SharedPtr msg) {ProcessLaserScan(msg);});
};

void Distance_Control_Node::ProcessLaserScan(LaserScan::SharedPtr& msg) {
    
    // Extra for C++ fans: Do this with STL library functions ^w^
    // Selecting the value and index of the closes point
    std::vector<float> ranges = msg->ranges;
    size_t minIndex = 0; // Ignoring potential error at 0 index for now.
    for (size_t i = 1; i < ranges.size(); i++) {
		if (ranges[i] < ranges[minIndex] && ranges[i] < msg->range_max && ranges[i] > msg->range_min) {
			minIndex = i;
		}
    }
    
    float distance = ranges[minIndex];
    float baseDegree = msg->angle_min + minIndex*msg->angle_increment;
    float degree = TransformDegree(baseDegree); // Calculating degree from index
    
    // Calculating speed
    float speedMagnitude = CalcSpeed(distance);
    
    SendTwist(degree, speedMagnitude);
}

inline float Distance_Control_Node::TransformDegree(float degree) {
	return degree + M_PI/2.0f; // rotating 90 degrees, don't ask me why.
}

inline float Distance_Control_Node::CalcSpeed(float distance) {
	// if less than minDistance, 1.0 magnitude
    return 1.0f-std::clamp((distance-minDistance)/(maxDistance-minDistance), 0.0f, 1.0f);
}

void Distance_Control_Node::SendTwist(float degree, float magnitude) {
    float new_x = std::clamp(cosf(degree) * magnitude, -1.0f, 1.0f);
    float new_y = std::clamp(sinf(degree) * magnitude, -1.0f, 1.0f);
    
    // Clearing at low values:
    
    if (abs(new_x) < clearLimit) new_x = 0.0f;
    if (abs(new_y) < clearLimit) new_y = 0.0f;
    
    auto msg = Twist();
    
    msg.linear.x = new_x;
    msg.linear.y = new_y;
    
    twist_publisher->publish(msg);
}

Distance_Control_Node::~Distance_Control_Node() {
    // Empty
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // We only create publishers, subscribers and timers during initialization
    rclcpp::executors::SingleThreadedExecutor executor;
    
    auto distanceController = std::make_shared<Distance_Control_Node>();
    executor.add_node(distanceController);
    
    // Starting node execution
    executor.spin();

	// Shutting down.
    rclcpp::shutdown();

    return 0;
}
