#include "rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace rclcpp;
using namespace std::literals::chrono_literals;

class Mock_lidar : public Node {
    public:
        Mock_lidar() : Node("Mock_lidar") {
            laserScanPub = create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
            timer = create_wall_timer(1ms);
        }
    
    private:
    Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScanPub;
    TimerBase::SharedPtr timer;

}