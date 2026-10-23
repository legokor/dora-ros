#include "distance_control.hpp"

Distance_Control_Node::Distance_Control_Node() : Node("distance_controller") {
    twist_publisher = create_publisher<Twist>("cmd_vel", 10);
    scan_subscriber = create_subscription<LaserScan>("scan", 10,
        [this](LaserScan::SharedPtr msg) {ProcessLaserScan(msg);});
    toggle_service = create_service<SetBool>("set_state",
        [this](const std::shared_ptr<SetBool::Request> request,
            const std::shared_ptr<SetBool::Response> response) {
                (void) response;
                SwitchToggle(request->data);
            });
};

void Distance_Control_Node::ProcessLaserScan(LaserScan::SharedPtr& msg) {
    if (!isOn) return;
    
    // Making a weighted average, where the weights are the ranges
    // and the numbers are the indexes*angle_increment.
    // A single range is small. 2 will fit inside the cache easily,
    // meaning multiple iterations do not cost much performance.
    // (Assuming the jetson has 1 Mb cache)
    
    // Filtering and inverting ranges, then accumulating total
    // Inverting range so the closest point has the highest weight
    // using 0.0 to filter because mathematically it will drop false values out.
    std::vector<float>& ranges = msg->ranges;
    const float range_min = msg->range_min;
    const float range_max = msg->range_max;
    for (float& range : ranges) {
        if (std::isfinite(range) || range > range_max || range < range_min) range = range_max;
        range = range_max - range;
    }
    const float closest_total = std::accumulate(ranges.begin(), ranges.end(), 0.0);
    const float closest = *std::max_element(ranges.begin(), ranges.end());
    const float speedMagnitude = CalcSpeed(closest);
    
    // Constructing degree array using vectorisation, then calculating
    std::vector<float> degrees(ranges.size());
    std::iota(degrees.begin(), degrees.end(), 0.0);
    float weighted_degree = std::inner_product(
        degrees.begin(), degrees.end(), ranges.begin(), 0.0)
        * msg->angle_increment / closest_total; // angle_increment doesn't have to be inside the degree array
    float degree = msg->angle_min + TransformDegree(weighted_degree);
    
    SendTwist(degree, speedMagnitude);
}

inline float Distance_Control_Node::TransformDegree(float degree) {
	return degree + M_PI/2.0f; // rotating 90 degrees, don't ask me why.
}

inline float Distance_Control_Node::CalcSpeed(float distance) {
	// if less than minDistance, 1.0 magnitude
    return std::clamp((distance-minDistance)/(maxDistance-minDistance), 0.0f, 1.0f);
}

void Distance_Control_Node::SendTwist(const float degree, const float magnitude) {
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

inline void Distance_Control_Node::SwitchToggle(bool state) {
    isOn = state;
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
