#ifndef RS1_NAVIGATION__VELOCITY_SELECTOR_HPP_
#define RS1_NAVIGATION__VELOCITY_SELECTOR_HPP_


#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rs1_interfaces/msg/mission_status.hpp"

namespace rs1_navigation
{

class VelocitySelectorNode : public rclcpp::Node
{
public:
    VelocitySelectorNode();


private:

    // Callback and central control loop
    void receiveCommand(const geometry_msgs::msg::Twist &input, const std::string &source); // Get current robot command input
    void publishCommand();

    // Helpers
    static double age(std::chrono::steady_clock::time_point timestamp); // Returns current timestamp/age of message
    bool statusFresh() const; // Returns if current status is fresh or stale
    void clearCommand();
    void resetController();

    rclcpp::TimerBase::SharedPtr timer_;

    // Pubs/subs
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_; // Autonomous command sub
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_; // Manual command sub
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<rs1_interfaces::msg::MissionStatus>::SharedPtr status_sub_;

    // Cbs
    void autoCb(const geometry_msgs::msg::Twist &msg);
    void manualCb(const geometry_msgs::msg::Twist &msg);
    void controllerCb(const geometry_msgs::msg::Twist &msg);
    void joyCb(const sensor_msgs::msg::Joy &msg);
    void statusCb(const rs1_interfaces::msg::MissionStatus &msg);

    std::string robot_;
    std::string mode_{"auto"}; // Initialise to autonomous mode
    std::string instance_id_;
    std::string manual_input_{"keyboard"};
    uint64_t input_revision_{0};

    double command_timeout_; // Timeout threshold
    double status_timeout_;
    int controller_enable_button_;
    bool controller_released_{false};
    bool controller_enabled_{false};
    std::chrono::steady_clock::time_point joy_received_{};

    bool status_seen_{false}; // Initialise both of these to false
    bool command_received_{false};

    std::chrono::steady_clock::time_point status_received_{}; // When latest status msg is received
    std::chrono::steady_clock::time_point command_time_{};
    geometry_msgs::msg::Twist command_;

};

} // namespace rs1_navigation


#endif // RS1_NAVIGATION__VELOCITY_SELECTOR_HPP_
