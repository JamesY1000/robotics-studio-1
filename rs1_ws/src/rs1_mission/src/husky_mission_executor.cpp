// This guy should handle the execution of mission commands for just the husky alone:
// For now it's just:
// Receive TraverseToROI goal and wait for generated ground path from parrot
// Receive and store parrot's ground path
// Navigate short chunks of the supplied route using nav2

// CURRENTLY NOT IN SCOPE:
// 2.
// Receives tree map
// Selects tree sampling strategy
// Drives to each tree and take dbh measurements
// Drive from ROI back to base

// Note** - information generated between robots is shared directly

#include "husky_mission_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

namespace rs1_mission
{

HuskyMissionExecutorNode::HuskyMissionExecutorNode() : Node("husky_mission_executor_node"),
    husky_state_{HuskyState::IDLE}
{

    // Allow the ground path to finish near the requested ROI
    ground_path_goal_tolerance_ = this->declare_parameter<double>("ground_path_goal_tolerance", 1.0);
    if (!std::isfinite(ground_path_goal_tolerance_) || ground_path_goal_tolerance_ < 0.0)
    {
        throw std::invalid_argument("ground_path_goal_tolerance must be finite and non-negative");
    }

    waypoint_spacing_ = this->declare_parameter<double>("waypoint_spacing", 5.0);
    max_chunk_waypoints_ = this->declare_parameter<int>("max_chunk_waypoints", 3);
    waypoint_wait_timeout_ = this->declare_parameter<double>("waypoint_wait_timeout", 10.0);
    max_chunk_retries_ = this->declare_parameter<int>("max_chunk_retries", 3);
    roi_tolerance_ = this->declare_parameter<double>("roi_tolerance", 0.5);
    if (!std::isfinite(waypoint_spacing_) || waypoint_spacing_ <= 0.0 ||
        !std::isfinite(waypoint_wait_timeout_) || waypoint_wait_timeout_ <= 0.0 ||
        !std::isfinite(roi_tolerance_) || roi_tolerance_ <= 0.0 ||
        max_chunk_waypoints_ < 1 || max_chunk_retries_ < 0)
    {
        throw std::invalid_argument("Invalid waypoint spacing, chunk size, timeout or retry limit");
    }

    navigate_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this, "/husky1/navigate_through_poses");
    global_costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/husky1/global_costmap/costmap", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&HuskyMissionExecutorNode::globalCostmapCb, this, std::placeholders::_1));
    navigation_timer_ = this->create_wall_timer(200ms,
        std::bind(&HuskyMissionExecutorNode::navigateNextChunk, this));
    navigation_timer_->cancel();

    // Create tf buffers and listeners
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // Create GetCurrentPose and SetGroundPath service servers
    get_current_pose_service_ = this->create_service<rs1_interfaces::srv::GetCurrentPose>(
        "/husky1/get_current_pose",
        std::bind(
            &HuskyMissionExecutorNode::getCurrentPoseRequestCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    set_ground_path_service_ = this->create_service<rs1_interfaces::srv::SetGroundPath>(
        "/husky1/set_ground_path",
        std::bind(
            &HuskyMissionExecutorNode::setGroundPathRequestCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // TraverseToROI server
    traverse_to_roi_server_ = rclcpp_action::create_server<rs1_interfaces::action::TraverseToROI>(
        this,
        "/husky1/traverse_to_roi",
        std::bind(&HuskyMissionExecutorNode::traverseToROIGoalRequestCb, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HuskyMissionExecutorNode::traverseToROICancelRequestCb, this, std::placeholders::_1),
        std::bind(&HuskyMissionExecutorNode::traverseToROIGoalAcceptedCb, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Husky mission executor initialised to IDLE");
}

rclcpp_action::GoalResponse HuskyMissionExecutorNode::traverseToROIGoalRequestCb(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Goal> goal)
{
    // Reject goal if husky is busy or the ROI is invalid
    if (husky_state_ != HuskyState::IDLE)
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Husky is currently busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validTargetPose(goal->target_roi) ||
        (goal->target_roi.header.frame_id != "parrot1_map" && goal->target_roi.header.frame_id != "husky1_map"))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Target ROI must be valid and in parrot1_map or husky1_map");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Wait for parrot to generate and send the ground path
    husky_state_ = HuskyState::WAITING_FOR_GROUND_PATH;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HuskyMissionExecutorNode::traverseToROICancelRequestCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle)
{
    // Check that this is the current mission goal
    if (!goal_handle || goal_handle != traverse_to_roi_goal_handle_ || !goal_handle->is_active())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // ROS marks the goal CANCELING after this callback returns
    cancel_timer_ = this->create_wall_timer(50ms,
        std::bind(&HuskyMissionExecutorNode::cancelTraversalToROI, this));
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HuskyMissionExecutorNode::traverseToROIGoalAcceptedCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle)
{
    traverse_to_roi_goal_handle_ = goal_handle;

    // Wait for parrot to send the ground path
    traverseToROI();
}

void HuskyMissionExecutorNode::getCurrentPoseRequestCb(
    const std::shared_ptr<rs1_interfaces::srv::GetCurrentPose::Request> request,
    std::shared_ptr<rs1_interfaces::srv::GetCurrentPose::Response> response)
{
    // Ensure we supply a frame id
    response->success = false;

    if (request->frame_id.empty())
    {
        response->message = "Requested pose frame is empty";
        return;
    }

    try
    {
        response->pose = getCurrentPose(request->frame_id);
        response->success = true;
        response->message = "Current Husky pose returned";
    }
    catch (const std::exception &e)
    {
        response->message = std::string("Cannot transform Husky pose: ") + e.what();
    }
}

void HuskyMissionExecutorNode::setGroundPathRequestCb(
    const std::shared_ptr<rs1_interfaces::srv::SetGroundPath::Request> request,
    std::shared_ptr<rs1_interfaces::srv::SetGroundPath::Response> response)
{
    response->accepted = false;

    // Only accept a path while waiting for one for the current mission
    if (husky_state_ != HuskyState::WAITING_FOR_GROUND_PATH || !traverse_to_roi_goal_handle_ ||
        !traverse_to_roi_goal_handle_->is_active() || traverse_to_roi_goal_handle_->is_canceling())
    {
        response->message = "Husky is not waiting for a ground path";
        return;
    }

    // Validate received path
    if (!validGroundPath(request->path))
    {
        response->message = "Ground path is empty or contains invalid poses/frames";
        return;
    }

    try
    {
        // Check that the path leads to the ROI requested by this mission
        if (!pathTargetsROI(request->path))
        {
            response->message = "Ground path does not end near the requested ROI";
            return;
        }

        // Transform the received ground path into husky frame
        ground_path_ = transformGroundPath(request->path);

        target_roi_ = traverse_to_roi_goal_handle_->get_goal()->target_roi;
        if (target_roi_.header.frame_id != "husky1_map")
        {
            const auto transform = tf_buffer_->lookupTransform("husky1_map", target_roi_.header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed_roi;
            tf2::doTransform(target_roi_, transformed_roi, transform);
            target_roi_ = transformed_roi;
        }
        target_roi_.pose.position.z = 0.0; // We don't really care about z for husky as it's on the ground

        // Take 'guidance' poses from the supplied path. This way, we have custom access to Husky's local planning
        // through nav2's NavigateThroughPoses action
        createGuidancePoses();
        husky_state_ = HuskyState::GROUND_PATH_RECEIVED;
        waypoint_wait_started_ = std::chrono::steady_clock::now();
        navigation_timer_->reset();
        publishTraversalFeedback("Ground path received; preparing navigation chunk");
        RCLCPP_INFO(this->get_logger(), "Received ground path with %zu poses; sampled %zu guidance poses",
            ground_path_.poses.size(), guidance_poses_.size());
    }
    catch (const std::exception &e)
    {
        response->message = std::string("Could not receive ground path: ") + e.what();
        abortTraversalToROI(response->message);
        return;
    }

    // Parrot ground path handoff -> husky received and stored the path
    response->accepted = true;
    response->message = "Ground path accepted for chunk navigation";
}

void HuskyMissionExecutorNode::traverseToROI()
{
    // Husky waits here until parrot sends the generated ground path
    publishTraversalFeedback("Waiting for ground path");
    RCLCPP_INFO(this->get_logger(), "Waiting for Parrot's ground path to the requested ROI");
}

geometry_msgs::msg::PoseStamped HuskyMissionExecutorNode::getCurrentPose(const std::string &frame_id, double max_age)
{
    const auto transform = tf_buffer_->lookupTransform(frame_id, "husky1_base_link", tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp, this->get_clock()->get_clock_type());
    const double age = (this->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0.0 || age > max_age)
    {
        throw std::runtime_error("Husky pose transform is uninitialised, stale or future-dated");
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return pose;
}

void HuskyMissionExecutorNode::createGuidancePoses()
{
    guidance_poses_.clear();
    double travelled = 0.0;
    double next_sample = waypoint_spacing_;

    // Sample by distance so changing the planner's pose density doesn't change our spacing
    auto route = ground_path_.poses;
    route.push_back(target_roi_);
    for (std::size_t i = 1; i < route.size(); ++i)
    {
        const auto &start = route[i - 1].pose.position;
        const auto &end = route[i].pose.position;
        const double length = std::hypot(end.x - start.x, end.y - start.y);
        if (length < 1e-6) continue;
        while (next_sample <= travelled + length)
        {
            const double fraction = (next_sample - travelled) / length;
            auto pose = route[i];
            pose.pose.position.x = start.x + fraction * (end.x - start.x);
            pose.pose.position.y = start.y + fraction * (end.y - start.y);
            pose.pose.position.z = 0.0;
            const double yaw = std::atan2(end.y - start.y, end.x - start.x);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            guidance_poses_.push_back(pose);
            next_sample += waypoint_spacing_;
        }
        travelled += length;
    }

    // The mission ROI is mandatory, even if parrot's path ends slightly short
    if (!guidance_poses_.empty() &&
        std::hypot(guidance_poses_.back().pose.position.x - target_roi_.pose.position.x,
                   guidance_poses_.back().pose.position.y - target_roi_.pose.position.y) <= roi_tolerance_)
    {
        guidance_poses_.pop_back();
    }
    guidance_poses_.push_back(target_roi_);
    discarded_guidance_.assign(guidance_poses_.size(), false);
}

void HuskyMissionExecutorNode::globalCostmapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
    const auto &info = map->info;
    geometry_msgs::msg::PoseStamped origin;
    origin.header = map->header;
    origin.pose = info.origin;

    // Validate the costmap and store locally
    if (map->header.frame_id != "husky1_map" || !validTargetPose(origin) ||
        !std::isfinite(info.resolution) || info.resolution <= 0.0 || info.width == 0 || info.height == 0 ||
        map->data.size() != static_cast<std::size_t>(info.width) * info.height)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Ignoring invalid Husky global costmap");
        return;
    }
    global_costmap_ = map;
}

int HuskyMissionExecutorNode::costmapValue(const geometry_msgs::msg::PoseStamped &pose) const
{
    const auto &info = global_costmap_->info;
    const auto &q = info.origin.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double dx = pose.pose.position.x - info.origin.position.x;
    const double dy = pose.pose.position.y - info.origin.position.y;
    const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / info.resolution;
    const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / info.resolution;
    if (x < 0.0 || y < 0.0 || x >= info.width || y >= info.height) return -2;
    return global_costmap_->data[static_cast<std::size_t>(y) * info.width + static_cast<std::size_t>(x)];
}

void HuskyMissionExecutorNode::waitForWaypoint(const std::string &reason)
{
    publishTraversalFeedback(reason);
    const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - waypoint_wait_started_).count();
    if (waited >= waypoint_wait_timeout_)
    {
        abortTraversalToROI(reason + "; waypoint selection timed out");
    }
}

void HuskyMissionExecutorNode::navigateNextChunk()
{
    const auto mission_goal = traverse_to_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling() ||
        navigation_goal_pending_ || navigation_goal_handle_) return;

    geometry_msgs::msg::PoseStamped robot_pose;
    try
    {
        // Nav2 needs a fresher transform than the general pose query service.
        // Wait here instead of sending a goal that its controller cannot use.
        robot_pose = getCurrentPose("husky1_map", navigation_pose_max_age_);
    }
    catch (const std::exception &e)
    {
        waitForWaypoint(std::string("Waiting for fresh localisation transform: ") + e.what());
        return;
    }

    // Handle end of path or reaching of ROI
    if (next_guidance_index_ + 1 >= guidance_poses_.size() &&
        std::hypot(robot_pose.pose.position.x - target_roi_.pose.position.x,
                   robot_pose.pose.position.y - target_roi_.pose.position.y) <= roi_tolerance_)
    {
        auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
        result->message = "Husky reached the ROI";
        mission_goal->succeed(result);
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
        returnToIdle();
        return;
    }
    if (!global_costmap_)
    {
        waitForWaypoint("Waiting for Husky's global costmap");
        return;
    }
    const rclcpp::Time stamp(global_costmap_->header.stamp, this->get_clock()->get_clock_type());
    const double age = (this->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0.0 || age > 2.0)
    {
        waitForWaypoint("Waiting for a fresh Husky global costmap");
        return;
    }
    if (!navigate_through_poses_client_->action_server_is_ready())
    {
        waitForWaypoint("Waiting for Nav2 NavigateThroughPoses");
        return;
    }

    nav2_msgs::action::NavigateThroughPoses::Goal goal;
    chunk_indices_.clear();
    std::string unavailable = "No usable guidance poses in this part of the route";
    const auto chunk_size = static_cast<std::size_t>(std::max(1, max_chunk_waypoints_ - chunk_failures_));
    for (std::size_t i = next_guidance_index_;
         i < guidance_poses_.size() && goal.poses.size() < chunk_size; ++i)
    {
        if (discarded_guidance_[i]) continue;
        const int cost = costmapValue(guidance_poses_[i]);
        if (cost == -2)
        {
            unavailable = "Next guidance pose is outside Husky's global costmap";
            break;
        }
        if (cost >= 99)
        {
            if (i + 1 == guidance_poses_.size())
            {
                unavailable = "ROI is blocked in Husky's global costmap";
                break;
            }
            discarded_guidance_[i] = true;
            RCLCPP_DEBUG(this->get_logger(), "Discarding blocked guidance pose %zu", i + 1);
            continue;
        }
        auto pose = guidance_poses_[i];
        pose.header.stamp = this->now();
        goal.poses.push_back(pose);
        chunk_indices_.push_back(i);
    }
    if (goal.poses.empty())
    {
        waitForWaypoint(unavailable);
        return;
    }

    // Send 'chunk' of goals through nav2's NavigateThroughPoses action server
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
        &HuskyMissionExecutorNode::navigateThroughPosesResponseCb, this, mission_goal, std::placeholders::_1);
    options.feedback_callback = std::bind(
        &HuskyMissionExecutorNode::navigateThroughPosesFeedbackCb, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(
        &HuskyMissionExecutorNode::navigateThroughPosesResultCb, this, mission_goal, std::placeholders::_1);
    try
    {
        navigation_goal_pending_ = true;
        husky_state_ = HuskyState::TRAVERSING_TO_ROI;
        navigate_through_poses_client_->async_send_goal(goal, options);
        publishTraversalFeedback("Navigating through " + std::to_string(goal.poses.size()) + " guidance poses");
        RCLCPP_DEBUG(this->get_logger(), "Requested navigation chunk ending at guidance pose %zu/%zu (%zu poses)",
            chunk_indices_.back() + 1, guidance_poses_.size(), chunk_indices_.size());
    }
    catch (const std::exception &e)
    {
        navigation_goal_pending_ = false;
        retryChunk(std::string("Could not request navigation: ") + e.what());
    }
}

void HuskyMissionExecutorNode::navigateThroughPosesResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle)
{
    // Check valid mission goal
    if (mission_goal != traverse_to_roi_goal_handle_) return;
    navigation_goal_pending_ = false;
    navigation_goal_handle_ = goal_handle;
    if (mission_goal->is_canceling())
    {
        cancelTraversalToROI();
    }
    else if (!goal_handle)
    {
        retryChunk("Nav2 rejected the navigation chunk");
    }
}

void HuskyMissionExecutorNode::advanceRouteProgress(std::size_t next_index)
{
    if (next_index > next_guidance_index_)
    {
        next_guidance_index_ = next_index;
        chunk_failures_ = 0;
    }
}

void HuskyMissionExecutorNode::navigateThroughPosesFeedbackCb(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
    if (!navigation_goal_handle_ || goal_handle->get_goal_id() != navigation_goal_handle_->get_goal_id() ||
        !traverse_to_roi_goal_handle_ || traverse_to_roi_goal_handle_->is_canceling()) return;

    // Nav2 removes passed intermediate poses; keep that progress if this chunk later fails
    const auto remaining = feedback->number_of_poses_remaining;
    if (remaining > 0 && static_cast<std::size_t>(remaining) < chunk_indices_.size())
    {
        advanceRouteProgress(chunk_indices_[chunk_indices_.size() - remaining - 1] + 1);
    }
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Husky traversing to ROI: %.2f m to chunk end, %d Nav2 recoveries",
        feedback->distance_remaining, feedback->number_of_recoveries);
}

void HuskyMissionExecutorNode::navigateThroughPosesResultCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
{
    if (mission_goal != traverse_to_roi_goal_handle_ || !navigation_goal_handle_ ||
        result.goal_id != navigation_goal_handle_->get_goal_id()) return;
    navigation_goal_handle_.reset();
    navigation_cancel_requested_ = false;
    if (mission_goal->is_canceling())
    {
        cancelTraversalToROI();
        return;
    }
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        const std::size_t endpoint = chunk_indices_.back();
        try
        {
            const auto pose = getCurrentPose("husky1_map");
            const auto &target = guidance_poses_[endpoint].pose.position;
            // Intermediate poses allow Nav2's combined planning and goal tolerances
            const double tolerance = endpoint + 1 == guidance_poses_.size() ? roi_tolerance_ : 1.0;
            if (std::hypot(pose.pose.position.x - target.x, pose.pose.position.y - target.y) > tolerance)
            {
                retryChunk("Nav2 finished outside the guidance pose arrival tolerance");
                return;
            }
        }
        catch (const std::exception &e)
        {
            retryChunk(std::string("Could not verify guidance pose arrival: ") + e.what());
            return;
        }
        advanceRouteProgress(endpoint + 1);
        waypoint_wait_started_ = std::chrono::steady_clock::now();
    }
    else
    {
        retryChunk("Navigation chunk failed; checking the remaining guidance poses");
    }
}

void HuskyMissionExecutorNode::retryChunk(const std::string &reason)
{
    // A transform interruption is a readiness problem, not a failed route.
    // Let the navigation timer wait for fresh data, with the usual bounded
    // waypoint timeout, without spending one of the navigation retries.
    try
    {
        getCurrentPose("husky1_map", navigation_pose_max_age_);
    }
    catch (const std::exception &e)
    {
        waypoint_wait_started_ = std::chrono::steady_clock::now();
        waitForWaypoint(std::string("Waiting for fresh localisation transform: ") + e.what());
        return;
    }

    if (++chunk_failures_ > max_chunk_retries_)
    {
        abortTraversalToROI(reason + "; no further route progress after chunk retries");
        return;
    }
    waypoint_wait_started_ = std::chrono::steady_clock::now();
    publishTraversalFeedback(reason);
    RCLCPP_WARN(this->get_logger(), "%s (retry %d/%d)", reason.c_str(), chunk_failures_, max_chunk_retries_);
}

bool HuskyMissionExecutorNode::validTargetPose(const geometry_msgs::msg::PoseStamped &pose) const
{
    // Check frame id, position and orientation
    const auto &position = pose.pose.position;
    const auto &orientation = pose.pose.orientation;
    const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                                orientation.z * orientation.z + orientation.w * orientation.w;

    return !pose.header.frame_id.empty() &&
        std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
        std::isfinite(norm_squared) && std::abs(norm_squared - 1.0) <= 1e-3;
}

bool HuskyMissionExecutorNode::validGroundPath(const nav_msgs::msg::Path &path) const
{
    // Check supplied path is valid
    if (path.header.frame_id.empty() || path.poses.empty())
    {
        return false;
    }

    // Validate each pose in the path - frame id, position and orientation
    for (const auto &pose : path.poses)
    {
        if (pose.header.frame_id != path.header.frame_id || !validTargetPose(pose))
        {
            return false;
        }
    }

    return true;
}

bool HuskyMissionExecutorNode::pathTargetsROI(const nav_msgs::msg::Path &path)
{
    // Compare the path endpoint and mission ROI in the same frame
    auto target_roi = traverse_to_roi_goal_handle_->get_goal()->target_roi;
    if (target_roi.header.frame_id != path.header.frame_id)
    {
        const auto transform = tf_buffer_->lookupTransform(path.header.frame_id, target_roi.header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped transformed_roi;
        tf2::doTransform(target_roi, transformed_roi, transform);
        target_roi = transformed_roi;
    }

    // Only compare xy since husky is following a ground path
    const auto &endpoint = path.poses.back().pose.position;
    const auto &target = target_roi.pose.position;
    return std::hypot(endpoint.x - target.x, endpoint.y - target.y) <= ground_path_goal_tolerance_;
}

nav_msgs::msg::Path HuskyMissionExecutorNode::transformGroundPath(
    const nav_msgs::msg::Path &path)
{
    // Transform the path from parrot -> husky frame to be usable for husky navigation
    nav_msgs::msg::Path transformed_path = path;

    // First let's check that it's not already transformed
    if (path.header.frame_id != "husky1_map")
    {
        // Lookup transform from the path's frame id to husky frame
        const auto transform = tf_buffer_->lookupTransform("husky1_map", path.header.frame_id, tf2::TimePointZero);

        for (std::size_t i = 0; i < path.poses.size(); ++i)
        {
            // Apply transformation from whatever frame (probably parrot frame) -> husky frame for each pose within the path
            tf2::doTransform(path.poses[i], transformed_path.poses[i], transform);
        }
    }

    // Update the header for the new transformed path, and update each pose inside the path too
    transformed_path.header.frame_id = "husky1_map";
    transformed_path.header.stamp = this->now();
    for (auto &pose : transformed_path.poses)
    {
        pose.header = transformed_path.header;
    }

    return transformed_path;
}

void HuskyMissionExecutorNode::publishTraversalFeedback(const std::string &stage)
{
    // Check that the mission is still running
    if (!traverse_to_roi_goal_handle_ || !traverse_to_roi_goal_handle_->is_active() ||
        traverse_to_roi_goal_handle_->is_canceling()) return;

    if (stage == last_stage_) return;
    last_stage_ = stage;
    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = stage;
    traverse_to_roi_goal_handle_->publish_feedback(feedback);
}

void HuskyMissionExecutorNode::abortTraversalToROI(const std::string &message)
{
    // Finish the mission once there is no navigation action running
    if (!traverse_to_roi_goal_handle_) return;
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        cancelTraversalToROI();
        return;
    }

    auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
    result->message = message;
    traverse_to_roi_goal_handle_->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    returnToIdle();
}

void HuskyMissionExecutorNode::cancelTraversalToROI()
{
    // Wait until ROS has marked the mission goal CANCELING
    if (!traverse_to_roi_goal_handle_ || !traverse_to_roi_goal_handle_->is_canceling()) return;
    if (navigation_goal_pending_) return;
    if (navigation_goal_handle_)
    {
        if (!navigation_cancel_requested_)
        {
            try
            {
                navigate_through_poses_client_->async_cancel_goal(navigation_goal_handle_);
                navigation_cancel_requested_ = true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "Could not cancel Husky navigation: %s", e.what());
            }
        }
        return;
    }

    if (cancel_timer_) cancel_timer_->cancel();
    auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
    result->message = "Husky traversal to ROI was canceled";
    traverse_to_roi_goal_handle_->canceled(result);
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    returnToIdle();
}

void HuskyMissionExecutorNode::returnToIdle()
{
    // Reset husky state to IDLE, reset the goal handles and stored ground_path_
    if (cancel_timer_) cancel_timer_->cancel();
    cancel_timer_.reset();
    navigation_timer_->cancel();
    navigation_goal_handle_.reset();
    navigation_goal_pending_ = false;
    navigation_cancel_requested_ = false;
    guidance_poses_.clear();
    discarded_guidance_.clear();
    chunk_indices_.clear();
    next_guidance_index_ = 0;
    chunk_failures_ = 0;
    last_stage_.clear();
    traverse_to_roi_goal_handle_.reset();
    ground_path_ = nav_msgs::msg::Path{};
    husky_state_ = HuskyState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Husky mission executor returned to IDLE");
}

}  // namespace rs1_mission
