#ifndef RS1_NAVIGATION__TERRAIN_FOLLOWING_HPP_
#define RS1_NAVIGATION__TERRAIN_FOLLOWING_HPP_

#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/create_timer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rs1_interfaces/srv/set_clearance.hpp"

namespace rs1_navigation
{

class TerrainFollowingNode : public rclcpp::Node
{
public:
  TerrainFollowingNode();

private:

  // Keep the node running in both modes. Hover sends zero velocity rather than
  // stopping publication, so Gazebo does not keep using the last climb command.
  enum class Mode
  {
    HOVER,
    TERRAIN_FOLLOWING,
    BACKUP_HOLD_HEIGHT
  };

  // Subscribe to laser scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Subscribe to cmd_vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Watch Nav2 itself, so mission, exploration and manually sent goals all work.
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr backup_status_sub_;

  // Publishes combined cmd_vel to /parrot1/cmd_vel_combined, then make gz plugin subscribe to this topic? What else is needed here?
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr combined_vel_pub_;

  // Lets the mission executor switch the held clearance (transit vs search).
  // The parrot executor drives its sweep through this service.
  rclcpp::Service<rs1_interfaces::srv::SetClearance>::SharedPtr set_clearance_service_;

  // Ros timer for control rate
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Nav2 retains finished goals in each action's status array. Keep navigation
  // and backup separate: exploration can back up after navigation has finished.
  struct ActionState
  {
    bool received_goal{false};
    action_msgs::msg::GoalInfo goal;
    bool goal_finished{false};
    bool executing{false};
    // Only needed for replacements accepted on the same simulation tick.
    std::vector<action_msgs::msg::GoalInfo> superseded_goals;
  };

  // Callbacks
  void scanCb(const sensor_msgs::msg::LaserScan &msg);
  void cmdVelCb(const geometry_msgs::msg::Twist &msg);
  void navigationStatusCb(const action_msgs::msg::GoalStatusArray &msg);
  void backupStatusCb(const action_msgs::msg::GoalStatusArray &msg);
  void setClearanceCb(const std::shared_ptr<rs1_interfaces::srv::SetClearance::Request> request,
                      std::shared_ptr<rs1_interfaces::srv::SetClearance::Response> response);

  void controlLoop(); // Main control loop
  double computeZVel(double current_height);
  void setMode(Mode mode);
  void updateMode();
  void updateActionState(const action_msgs::msg::GoalStatusArray &msg, ActionState &state);

  Mode mode_{Mode::HOVER};
  ActionState navigation_state_;
  ActionState backup_state_;

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
