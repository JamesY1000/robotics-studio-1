#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs1_interfaces/msg/safety_state.hpp"
#include "rs1_interfaces/srv/safety_stop.hpp"

class EmergencyStop final : public rclcpp::Node
{
public:
  EmergencyStop();

private:
  void publish_state();
  static void publish_zero(
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &publisher);

  bool husky_stopped_;
  bool parrot_stopped_;
  rclcpp::Publisher<rs1_interfaces::msg::SafetyState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr husky_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr parrot_pub_;
  rclcpp::Service<rs1_interfaces::srv::SafetyStop>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr zero_timer_;
};