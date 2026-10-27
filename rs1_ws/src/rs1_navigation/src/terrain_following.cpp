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

  // Nav2's status publisher retains its latest message. Match its QoS so this
  // node also learns about an existing goal if it starts after navigation does.
  navigation_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TerrainFollowingNode::navigationStatusCb, this, std::placeholders::_1)
  );

  // Exploration can request a backup without an active NavigateToPose goal.
  // Allow that retreat, but hold height instead of resuming the aborted climb.
  backup_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "backup/_action/status",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TerrainFollowingNode::backupStatusCb, this, std::placeholders::_1)
  );

  // Create ros timer as main control loop
  auto timer_period = 1.0 / control_rate_hz_;
  control_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    rclcpp::Duration::from_seconds(timer_period),
    std::bind(&TerrainFollowingNode::controlLoop, this)
  );


  RCLCPP_INFO(get_logger(), "Terrain following node initialised in hover (target %.2f m, loop %.1f Hz)", offset_height_, control_rate_hz_);
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

void TerrainFollowingNode::navigationStatusCb(const action_msgs::msg::GoalStatusArray &msg)
{
  updateActionState(msg, navigation_state_);
  updateMode();
}

void TerrainFollowingNode::backupStatusCb(const action_msgs::msg::GoalStatusArray &msg)
{
  updateActionState(msg, backup_state_);
  updateMode();
}

void TerrainFollowingNode::updateActionState(
  const action_msgs::msg::GoalStatusArray &msg, ActionState &state)
{
  // Status arrays are not necessarily ordered. Find the most recently accepted
  // goal, including finished goals, rather than looking for any executing goal.
  const action_msgs::msg::GoalStatus *latest_status = nullptr;
  for (const auto &status : msg.status_list)
  {
    // Timestamps cannot distinguish replacements accepted on the same tick.
    // Remember those previous IDs so their late statuses cannot resume them.
    bool superseded = false;
    for (const auto &goal : state.superseded_goals)
    {
      if (status.goal_info.goal_id == goal.goal_id) superseded = true;
    }
    if (superseded) continue;

    if (latest_status == nullptr ||
        rclcpp::Time(status.goal_info.stamp) > rclcpp::Time(latest_status->goal_info.stamp))
    {
      latest_status = &status;
    }
    else if (status.goal_info.stamp == latest_status->goal_info.stamp)
    {
      // Two goals can be accepted on the same simulation tick. Prefer the
      // executing goal over a finished one; otherwise keep our current goal.
      const bool status_executing =
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
      const bool latest_executing =
        latest_status->status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
      if ((status_executing && !latest_executing) ||
          (status_executing == latest_executing &&
           status.goal_info.goal_id == state.goal.goal_id))
      {
        latest_status = &status;
      }
    }
  }

  if (latest_status == nullptr)
  {
    // An empty array means no goals. An array containing only superseded goals
    // is an old snapshot, so it must not stop the newer goal we already know.
    if (msg.status_list.empty()) state.executing = false;
    return;
  }

  // Ignore an older status snapshot if a newer goal is already known.
  if (state.received_goal &&
      rclcpp::Time(latest_status->goal_info.stamp) < rclcpp::Time(state.goal.stamp))
  {
    return;
  }

  const bool new_goal = !state.received_goal ||
    latest_status->goal_info.goal_id != state.goal.goal_id;
  if (new_goal)
  {
    if (state.received_goal && latest_status->goal_info.stamp == state.goal.stamp)
    {
      state.superseded_goals.push_back(state.goal);
    }
    else
    {
      // Once the acceptance time advances, the timestamp check is sufficient.
      state.superseded_goals.clear();
    }
    state.goal = latest_status->goal_info;
    state.received_goal = true;
    state.goal_finished = false;

    // A replacement goal must receive its own fresh velocity command. Do not
    // carry the previous goal's motion into the new goal while it is planning.
    received_cmd_vel_ = false;
    combined_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  const auto status = latest_status->status;
  if (status == action_msgs::msg::GoalStatus::STATUS_CANCELING ||
      status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
      status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
  {
    state.goal_finished = true;
  }

  // Accepted goals may still be waiting to execute. Only follow terrain once
  // execution starts, and never resume this same goal after it has finished.
  state.executing = status == action_msgs::msg::GoalStatus::STATUS_EXECUTING &&
    !state.goal_finished;
}

void TerrainFollowingNode::updateMode()
{
  if (navigation_state_.executing)
  {
    setMode(Mode::TERRAIN_FOLLOWING);
  }
  else if (backup_state_.executing)
  {
    setMode(Mode::BACKUP_HOLD_HEIGHT);
  }
  else
  {
    setMode(Mode::HOVER);
  }
}

void TerrainFollowingNode::setMode(Mode mode)
{
  if (mode_ == mode) return;
  mode_ = mode;

  // Stop immediately at the transition as well as on each control-loop tick.
  // Clear any old command so starting navigation again cannot reuse it.
  received_cmd_vel_ = false;
  combined_vel_pub_->publish(geometry_msgs::msg::Twist{});
  if (mode_ == Mode::HOVER)
    RCLCPP_INFO(get_logger(), "Navigation stopped - hovering");
  else if (mode_ == Mode::TERRAIN_FOLLOWING)
    RCLCPP_INFO(get_logger(), "Navigation executing - following terrain");
  else
    RCLCPP_INFO(get_logger(), "Backup executing - holding height during retreat");
}

void TerrainFollowingNode::controlLoop()
{
  const auto current_time = now();

  // Create a twist msg for the combined velocity
  geometry_msgs::msg::Twist combined_vel{};

  // Status is retained, so also stop if the corresponding action server goes
  // away. A new status message is required before that action can resume.
  if (navigation_state_.executing && navigation_status_sub_->get_publisher_count() == 0)
    navigation_state_.executing = false;
  if (backup_state_.executing && backup_status_sub_->get_publisher_count() == 0)
    backup_state_.executing = false;
  updateMode();

  // Hover before the first goal and after arrival, abort or cancellation.
  // In particular, do not keep climbing towards the target ground clearance.
  if (mode_ == Mode::HOVER)
  {
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  // Nav2 status changes are event-driven, not a heartbeat. Use command freshness
  // to stop all motion if commands disappear, including the vertical correction.
  const double cmd_vel_age = (current_time - last_cmd_vel_time_).seconds();
  if (!received_cmd_vel_ || cmd_vel_age < 0.0 || cmd_vel_age > cmd_vel_timeout_)
  {
    combined_vel_pub_->publish(combined_vel);
    return;
  }

  // A fresh zero command can be a temporary pause during an active goal.
  // Keep height correction available during that pause; it is not arrival.
  combined_vel.linear.x = latest_cmd_vel_.linear.x;
  combined_vel.linear.y = latest_cmd_vel_.linear.y;
  combined_vel.angular.z = latest_cmd_vel_.angular.z;

  // A standalone recovery can move away from foliage without climbing into it.
  // Normal Nav2 recovery inside an executing navigation goal keeps following terrain.
  if (mode_ == Mode::BACKUP_HOLD_HEIGHT)
  {
    combined_vel_pub_->publish(combined_vel);
    return;
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
