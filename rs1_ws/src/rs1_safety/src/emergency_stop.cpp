#include "rs1_safety/emergency_stop.hpp"

#include <chrono>
#include <memory>
#include <string>

EmergencyStop::EmergencyStop()
: Node("emergency_stop"), husky_stopped_(false), parrot_stopped_(false)
{
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  state_pub_ = create_publisher<rs1_interfaces::msg::SafetyState>("/safety/state", state_qos);
  // This node is the only publisher on the *_safe topics the Gazebo bridges read.
  husky_pub_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel_safe", 10);
  parrot_pub_ = create_publisher<geometry_msgs::msg::Twist>("/parrot1/cmd_vel_safe", 10);

  // Forward controller commands only while the robot's stop is clear.
  husky_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/husky1/cmd_vel", 10, [this](const geometry_msgs::msg::Twist & msg) {
      if (!husky_stopped_) {
        husky_pub_->publish(msg);
      }
    });
  parrot_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/parrot1/cmd_vel_combined", 10, [this](const geometry_msgs::msg::Twist & msg) {
      if (!parrot_stopped_) {
        parrot_pub_->publish(msg);
      }
    });

  stop_service_ = create_service<rs1_interfaces::srv::SafetyStop>(
    "/safety/stop",
    [this](const std::shared_ptr<rs1_interfaces::srv::SafetyStop::Request> request,
    std::shared_ptr<rs1_interfaces::srv::SafetyStop::Response> response) {
      const bool husky = request->robot == "husky1" || request->robot == "all";
      const bool parrot = request->robot == "parrot1" || request->robot == "all";
      if (!husky && !parrot) {
        response->success = false;
        response->message = "robot must be husky1, parrot1 or all";
        return;
      }
      if (husky) {
        husky_stopped_ = request->stopped;
      }
      if (parrot) {
        parrot_stopped_ = request->stopped;
      }
      publish_state();
      if (request->stopped) {
        if (husky) {
          publish_zero(husky_pub_);
        }
        if (parrot) {
          publish_zero(parrot_pub_);
        }
      }
      response->success = true;
      response->message = request->stopped ? request->robot + " stop latched" :
        request->robot + " stop reset";
    });

  publish_state();
  // Keep braking while latched in case Gazebo holds the last forwarded command.
  zero_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
    if (husky_stopped_) {
      publish_zero(husky_pub_);
    }
    if (parrot_stopped_) {
      publish_zero(parrot_pub_);
    }
  });
  RCLCPP_INFO(get_logger(), "Emergency stop gate ready on /safety/stop and /safety/state");
}

void EmergencyStop::publish_state()
{
  rs1_interfaces::msg::SafetyState state;
  state.header.stamp = now();
  state.husky_stopped = husky_stopped_;
  state.parrot_stopped = parrot_stopped_;
  state_pub_->publish(state);
}

void EmergencyStop::publish_zero(
  const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &publisher)
{
  if (publisher) {
    publisher->publish(geometry_msgs::msg::Twist{});
  }
}