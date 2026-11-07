#include "distance_control.hpp"

Distance_Control_Node::Distance_Control_Node() : Node("distance_controller") {
    twist_publisher = create_publisher<Twist>("cmd_vel", 10);
    scan_subscriber = create_subscription<LaserScan>("scan", 10,
        [this](LaserScan::SharedPtr msg) {ProcessLaserScan(msg);});
    toggle_service = create_service<SetBool>("set_state",
        [this](const std::shared_ptr<SetBool::Request> request,
        const std::shared_ptr<SetBool::Response> response) {
            (void) response; // No response for now.
            SwitchToggle(request->data);
        });
};

void Distance_Control_Node::ProcessLaserScan(LaserScan::SharedPtr& msg) {
    if (!isOn) return;
    
    // Filtering errors and reflecting ray lengths.
    std::vector<float>& ranges = msg->ranges;
    for (float& range : ranges) {
        if (std::isfinite(range) && range < msg->range_max && range > msg->range_min)
            range = std::clamp(msg->range_max - range, msg->range_min, msg->range_max);
        else 
            range = msg->range_min;
    }
    
    const float closest_total = std::accumulate(ranges.begin(), ranges.end(), 0.0);
    const float closest = *std::max_element(ranges.begin(), ranges.end());
    const float speedMagnitude = CalcSpeed(closest);
    
    // Constructing degree array using vectorisation, then calculating
    std::vector<float> degrees(ranges.size());
    std::iota(degrees.begin(), degrees.end(), 0.0);
    
    // Making a weighted average.
    float weighted_degree = std::inner_product(
        degrees.begin(), degrees.end(), ranges.begin(), 0.0)
        * msg->angle_increment / closest_total;
    float degree = msg->angle_min + CalcDegree(weighted_degree);
    
    SendTwist(degree, speedMagnitude);
}

inline float Distance_Control_Node::CalcDegree(float degree) {
	return degree + M_PI/2.0f; // rotating 90 degrees, don't ask me why.
}

inline float Distance_Control_Node::CalcSpeed(float distance) {
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

    rclcpp::executors::SingleThreadedExecutor executor;
    
    auto distanceController = std::make_shared<Distance_Control_Node>();
    executor.add_node(distanceController);
    
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
