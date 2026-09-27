#ifndef RS1_NAVIGATION__TERRAIN_FOLLOWING_HPP_
#define RS1_NAVIGATION__TERRAIN_FOLLOWING_HPP_

#include "rclcpp/create_timer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace rs1_navigation
{

class TerrainFollowingNode : public rclcpp::Node
{
public:
  TerrainFollowingNode();

private:

  // Subscribe to laser scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Subscribe to cmd_vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Publishes combined cmd_vel to /parrot1/cmd_vel_combined, then make gz plugin subscribe to this topic? What else is needed here?
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr combined_vel_pub_;

  // Ros timer for control rate
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Callbacks
  void scanCb(const sensor_msgs::msg::LaserScan &msg);
  void cmdVelCb(const geometry_msgs::msg::Twist &msg);

  void controlLoop(); // Main control loop
  double computeZVel(double current_height);

  // We also want some filtering to (maybe delta gradient suddenly too high?) and maybe ignore those as they might be rocks/bushes.
  // This should be called by our combine_cmd_vel method

  double offset_height_; // TODO (james): How to make this cleaner - maybe subscribe from a topic? For now let's just initialise at 6.0m above ground
  double control_rate_hz_; // Same here, just initialise to 20 hz for now
  sensor_msgs::msg::LaserScan latest_scan_;
  geometry_msgs::msg::Twist latest_cmd_vel_;
  bool received_scan_;
  bool received_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  double scan_timeout_; // Scans older than this timeout should be considered stale
  double kp_;
  double MAX_VERTICAL_SPEED_;
  double cmd_vel_timeout_;    // seconds

};

}  // namespace rs1_navigation

#endif  // RS1_NAVIGATION__TERRAIN_FOLLOWING_HPP_
