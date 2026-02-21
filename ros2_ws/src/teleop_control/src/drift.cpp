#include "drift.hpp"

using Twist = geometry_msgs::msg::Twist;
using namespace rclcpp;

class DriftNode : public Node {
    public:
        DriftNode() : Node("drift_node") {
            drift_subscriber = create_subscription<Twist>("drift_cmd_vel", 10,
                std::bind(&DriftNode::Subscription_callback, this, std::placeholders::_1));
            drift_publisher = create_publisher<Twist>("cmd_vel", 10);
            declare_parameter("damping", 100.0);
        }

        void Subscription_callback(const Twist::SharedPtr msg) {
            double dam = get_parameter("damping").as_double();
            
            double linDeaccel = linSpeed*0.1;
            linSpeed += (msg->linear.x-linDeaccel) / dam;
            linSpeed *= cos(angSpeed);

            double angDeaccel = angSpeed*0.1;

            angSpeed += (msg->angular.z-angDeaccel) / dam;

            linSpeed = std::clamp(linSpeed, linLimit[0], linLimit[1]);
            angSpeed = std::clamp(angSpeed, angLimit[0], angLimit[1]);

            // Clearing to stop the wheels
            if (abs(linSpeed) < 0.001) linSpeed = 0.0;
            if (abs(angSpeed) < 0.001) angSpeed = 0.0;

            auto new_msg = Twist();
            new_msg.linear.x = linSpeed;
            new_msg.angular.z = angSpeed;
            drift_publisher->publish(new_msg);
        }

    private:
        Subscription<Twist>::SharedPtr drift_subscriber;
        Publisher<Twist>::SharedPtr drift_publisher;
        double linSpeed = 0;
        double angSpeed = 0;
        double linLimit[2] = {-1.0, 1.0};
        double angLimit[2] = {-M_PI/3, M_PI/3};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto drift_node = std::make_shared<DriftNode>();
    rclcpp::spin(drift_node);

    rclcpp::shutdown();

    return 0;
}
