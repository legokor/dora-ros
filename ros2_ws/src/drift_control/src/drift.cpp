#include "drift.hpp"

using Twist = geometry_msgs::msg::Twist;
using namespace rclcpp;

class DriftNode : public Node {
    public:
        DriftNode() : Node("drift_node") {
            drift_subscriber = create_subscription<Twist>("drift_cmd_vel", 10, std::bind(&Subscription_callback, this));
            drift_publisher = create_publisher<Twist>("cmd_vel", 10);
        }

        void Subscription_callback(Twist::SharedPtr& msg) {
            msg->linear.x;
        }

    private:
        Subscription<Twist>::SharedPtr drift_subscriber;
        Publisher<Twist>::SharedPtr drift_publisher;
}
