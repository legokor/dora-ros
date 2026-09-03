#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "controller/uart_handler.hpp"

#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <thread>
#include <atomic>

// Robot specific namespace
namespace dora {

/* Responsible for storing and publishing data received from and to the firmware.
 * Provides an interface for high-level control
*/
class ControllerNode : public rclcpp::Node {
	
public:

    ControllerNode();
    ~ControllerNode();

private:

	// Robot UART interface
    UARTHandler uart;

	// Thread
    std::atomic<bool> io_thread_running;
    std::thread io_thread;

	// Ros subscriptions and Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;

	/* Receives a Twist message, then sends velocity data to the firmware through uart
	 * @param msg: The pointer of the Twist message
	*/
    void sendTwist(const geometry_msgs::msg::Twist::SharedPtr& msg);
    
    /* Publishes a TwistStamped message from the SpeedData message received from the
     * handleReceivedMessage function.
     * @param msg: The firmware's message from UART
    */
    void publishMeasure(const SpeedData& msg);
    
    /* Transforms messages into structured datatypes from UARTHandler, then calls other
     * functions which use that datatype.
     * @param msg: The message received from UARTHandler
    */
    void handleReceivedMessage(const dora::ReceivedMessage& msg);
};

} // namespace dora

#endif
