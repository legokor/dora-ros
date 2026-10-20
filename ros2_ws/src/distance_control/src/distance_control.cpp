#include "distance_control.hpp"

Distance_Control_Node::Distance_Control_Node() : Node("distance_controller") {
    twist_publisher = create_publisher<TwistStamped>("/cmd_vel", 10);
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
    float degree = msg->angle_min + minIndex*msg->angle_increment + 3.14; // Calculating movement degree from index
    
    // Calculating speed
    float speedMagnitude = CalcSpeed(distance);
    
    SendTwist(degree, speedMagnitude);
}

float Distance_Control_Node::CalcSpeed(float distance) {
    return std::clamp(distance/(maxDistance-minDistance), 0.0f, 1.0f);
}

void Distance_Control_Node::SendTwist(float degree, float magnitude) {
    auto msg = TwistStamped();
    // Filling the header is optional for us.
    msg.twist.linear.x = cosf(degree) * magnitude;
    msg.twist.linear.y = sinf(degree) * magnitude;
    
    twist_publisher->publish(msg);
}

Distance_Control_Node::~Distance_Control_Node() {
    // Empty
}

int main() {
    
}
