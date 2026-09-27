#include "rs1_navigation/terrain_following.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <algorithm>

#include "rclcpp/create_timer.hpp"

namespace rs1_navigation
{

TerrainFollowingNode::TerrainFollowingNode() : Node("terrain_following_node"),
  received_scan_{false},
  received_cmd_vel_{false},
  last_cmd_vel_time_{0, 0, RCL_ROS_TIME}
{
  // Grab params
  offset_height_ = declare_parameter<double>("target_clearance", 4.0);
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  scan_timeout_ = declare_parameter<double>("scan_timeout", 0.25);
  cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.25);
  kp_ = declare_parameter<double>("kp", 0.5);
  MAX_VERTICAL_SPEED_ = declare_parameter<double>("max_vertical_speed", 0.5);


  // Check valid offset height and control rate
  if (!std::isfinite(offset_height_) || offset_height_ <= 0.0) {
    throw std::invalid_argument("target_clearance must be finite and greater than zero");
  }
  if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
    throw std::invalid_argument("control_rate_hz must be finite and greater than zero");
  }

  // Initialise sub/pubs
  auto scan_qos = rclcpp::SensorDataQoS();
  auto cmd_vel_qos = rclcpp::QoS(10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "downwards_scan", // TODO (james): Store this in maybe rs1_interfaces (or the config file)? So it's not just hardcoded?
    scan_qos.keep_last(1),
    std::bind(&TerrainFollowingNode::scanCb, this, std::placeholders::_1)
  );

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", // TODO (james): Store in config file
    cmd_vel_qos,
    std::bind(&TerrainFollowingNode::cmdVelCb, this, std::placeholders::_1)
  );

  combined_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel_combined", // TODO (james): Store in config file
    cmd_vel_qos
  );

  // Create ros timer as main control loop
  auto timer_period = 1.0 / control_rate_hz_;
  control_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    rclcpp::Duration::from_seconds(timer_period),
    std::bind(&TerrainFollowingNode::controlLoop, this)
  );


  RCLCPP_INFO(get_logger(), "Terrain following node initialised (target %.2f m, loop %.1f Hz)", offset_height_, control_rate_hz_);
}

void TerrainFollowingNode::scanCb(const sensor_msgs::msg::LaserScan &msg)
{
  if (!received_scan_) received_scan_ = true;
  latest_scan_ = msg;
}
void TerrainFollowingNode::cmdVelCb(const geometry_msgs::msg::Twist &msg)
{
  if (!received_cmd_vel_) received_cmd_vel_ = true;
  last_cmd_vel_time_ = now();
  latest_cmd_vel_ = msg;
}

void TerrainFollowingNode::controlLoop()
{
  const auto current_time = now();

  // Create a twist msg for the combined velocity
  geometry_msgs::msg::Twist combined_vel{};

  // Copy latest nav2 cmd_vel into combined_vel
  if (received_cmd_vel_)
  {
    const double cmd_vel_age = (current_time - last_cmd_vel_time_).seconds();

    if (cmd_vel_age >= 0.0 && cmd_vel_age <= cmd_vel_timeout_)
    {
      combined_vel.linear.x = latest_cmd_vel_.linear.x;
      combined_vel.linear.y = latest_cmd_vel_.linear.y;
      combined_vel.angular.z = latest_cmd_vel_.angular.z;
    }
  }

  // If we have no new scan, just publish untouched cmd_vel
  if (!received_scan_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for first downward scan");
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  const double scan_age = (current_time - rclcpp::Time(latest_scan_.header.stamp)).seconds();

  // If scans become stale or invalid, just publish untouched cmd_vel
  if (scan_age < 0.0 || scan_age > scan_timeout_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Downward scan stale or invalid: age %.2f s", scan_age);
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  // Currently the lidar has only 3 lasers. If the lidar is invalid, just publish untouched cmd_vel
  if (latest_scan_.ranges.size() != 3)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Expected 3 downward lidar readings, received %zu", latest_scan_.ranges.size());
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  // Get current height - use the middle scan
  const double current_height = latest_scan_.ranges[1];

  // Check scans are within valid ranges, if not just publish untouched cmd_vel
  if (!std::isfinite(current_height) ||
      current_height < latest_scan_.range_min ||
      current_height > latest_scan_.range_max)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid downward lidar reading");
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  // Compute z velocity, combine it into the cmd_vel and publish
  combined_vel.linear.z = computeZVel(current_height);
  combined_vel_pub_->publish(combined_vel);
}

double TerrainFollowingNode::computeZVel(double current_height)
{
  // Compute z velocity - use P controller for now
  const double error = offset_height_ - current_height;
  auto z_vel = std::clamp(kp_ * error, -MAX_VERTICAL_SPEED_, MAX_VERTICAL_SPEED_);

  return z_vel;
}



}  // namespace rs1_navigation
