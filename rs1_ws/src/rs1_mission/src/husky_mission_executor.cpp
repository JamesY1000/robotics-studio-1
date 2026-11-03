// Husky follows the ground path, measures the mapped trees, then returns to its start

#include "husky_mission_executor.hpp"
#include "tree_survey.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <string>
#include <type_traits>
#include <tf2/utils.h>

using namespace std::chrono_literals;

namespace rs1_mission
{
namespace
{
std::string metres(double distance)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << distance << " m";
    return out.str();
}

double neighbourBound(const rs1_interfaces::msg::TreeMap &map, std::size_t index,
                      const geometry_msgs::msg::Point &centre, double limit)
{
    // Keep the search on this tree's side of the gap to its mapped neighbours
    for (std::size_t i = 0; i < map.trees.size(); ++i)
    {
        if (i == index) continue;
        const auto &other = map.trees[i].location;
        limit = std::min(limit, std::hypot(other.x - centre.x, other.y - centre.y) / 2);
    }
    return limit;
}

bool timedOut(std::chrono::steady_clock::time_point start, double seconds)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= seconds;
}
}

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

    // Tree database built from the coordinator's forwarded tree map (the
    // SampleROI action result the parrot returned).
    tree_database_file_ = this->declare_parameter<std::string>(
        "tree_database_file", "/tmp/rs1_tree_database.csv");
    tree_map_subscription_ = this->create_subscription<rs1_interfaces::msg::TreeMap>(
        "/mission/tree_map", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&HuskyMissionExecutorNode::treeMapCb, this, std::placeholders::_1));
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

    setupTreeSurvey();
    home_execution_ = std::make_unique<ReturnHomeExecution>(*this, tf_buffer_, "husky1",
        [this]() { return husky_state_ == HuskyState::IDLE; },
        [this](bool busy) { husky_state_ = busy ? HuskyState::RETURNING_HOME : HuskyState::IDLE; });

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

void HuskyMissionExecutorNode::treeMapCb(const rs1_interfaces::msg::TreeMap::SharedPtr msg)
{
    if (!msg) return;

    std::ofstream file(tree_database_file_, std::ios::trunc);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open tree database file: %s", tree_database_file_.c_str());
        return;
    }

    file << "tree_type,x,y,z,observations\n";
    for (const auto &tree : msg->trees)
    {
        file << tree.tree_type << ','
             << tree.location.x << ','
             << tree.location.y << ','
             << tree.location.z << ','
             << tree.observations << '\n';
    }
    file.close();

    RCLCPP_INFO(this->get_logger(), "Stored %zu tree(s) in %s",
        msg->trees.size(), tree_database_file_.c_str());
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

    if (goal->resume_route && (goal->route_id.empty() || saved_route_id_ != goal->route_id ||
        saved_route_roi_.header.frame_id != goal->target_roi.header.frame_id ||
        saved_route_roi_.pose != goal->target_roi.pose || guidance_poses_.empty()))
        return rclcpp_action::GoalResponse::REJECT;

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

    if (goal_handle->get_goal()->resume_route)
    {
        husky_state_ = HuskyState::GROUND_PATH_RECEIVED;
        waypoint_wait_started_ = std::chrono::steady_clock::now();
        navigation_timer_->reset();
        publishTraversalFeedback("Resuming the accepted ground path");
        return;
    }
    saved_route_id_.clear();
    guidance_poses_.clear();
    discarded_guidance_.clear();
    next_guidance_index_ = 0;
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
        saved_route_id_ = traverse_to_roi_goal_handle_->get_goal()->route_id;
        saved_route_roi_ = traverse_to_roi_goal_handle_->get_goal()->target_roi;
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
                const auto goal = navigation_goal_handle_;
                navigate_through_poses_client_->async_cancel_goal(goal, [this, goal](auto reply) {
                    if (goal == navigation_goal_handle_ && (!reply || reply->goals_canceling.empty()))
                        navigation_cancel_requested_ = false;
                });
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
    // Keep the remaining guidance poses for a resume of this route
    chunk_indices_.clear();
    chunk_failures_ = 0;
    last_stage_.clear();
    traverse_to_roi_goal_handle_.reset();
    ground_path_ = nav_msgs::msg::Path{};
    husky_state_ = HuskyState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Husky mission executor returned to IDLE");
}

// Check Husky's footprint and the route before committing to a tree approach

void HuskyMissionExecutorNode::footprintCb(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
    if (msg->polygon.points.size() < 3) return;
    try
    {
        // Use Nav2's actual padded footprint so the survey doesn't invent a different robot size
        const auto transform = tf_buffer_->lookupTransform(
            "husky1_base_link", msg->header.frame_id,
            tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds())));
        std::vector<geometry_msgs::msg::Point> footprint;
        for (const auto &vertex : msg->polygon.points)
        {
            geometry_msgs::msg::Point input, output;
            input.x = vertex.x;
            input.y = vertex.y;
            input.z = vertex.z;
            tf2::doTransform(input, output, transform);
            if (!std::isfinite(output.x) || !std::isfinite(output.y)) return;
            footprint.push_back(output);
        }
        survey_footprint_ = std::move(footprint);
        footprint_stamp_ = msg->header.stamp;
    }
    catch (const tf2::TransformException &)
    {
        // A later footprint will arrive once its transform is available
    }
}

bool HuskyMissionExecutorNode::surveyFootprintClear(const geometry_msgs::msg::PoseStamped &pose, bool allow_unknown) const
{
    return global_costmap_ && footprintClearOnMap(pose, *global_costmap_, allow_unknown);
}

bool HuskyMissionExecutorNode::surveyMapsFresh() const
{
    if (!global_costmap_ || !survey_local_map_ || survey_footprint_.empty()) return false;
    for (const auto &stamp : {rclcpp::Time(global_costmap_->header.stamp),
                             rclcpp::Time(survey_local_map_->header.stamp), footprint_stamp_})
    {
        const double age = (now()-stamp).seconds();
        if (age < 0 || age > 2.0) return false;
    }
    return true;
}

bool HuskyMissionExecutorNode::surveyRobotClear(const geometry_msgs::msg::PoseStamped &pose) const
{
    if (!surveyMapsFresh() || !surveyFootprintClear(pose)) return false;
    auto local_pose = pose;
    if (pose.header.frame_id != survey_local_map_->header.frame_id)
    {
        const auto transform = tf_buffer_->lookupTransform(
            survey_local_map_->header.frame_id, pose.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(pose, local_pose, transform);
    }
    return footprintClearOnMap(local_pose, *survey_local_map_, false);
}

bool HuskyMissionExecutorNode::footprintClearOnMap(const geometry_msgs::msg::PoseStamped &pose,
    const nav_msgs::msg::OccupancyGrid &map, bool allow_unknown) const
{
    if (survey_footprint_.empty() || pose.header.frame_id != map.header.frame_id) return false;
    const auto &info = map.info;
    const double origin_yaw = tf2::getYaw(info.origin.orientation);
    const double dx = pose.pose.position.x-info.origin.position.x;
    const double dy = pose.pose.position.y-info.origin.position.y;
    const double cx = (std::cos(origin_yaw)*dx + std::sin(origin_yaw)*dy)/info.resolution;
    const double cy = (-std::sin(origin_yaw)*dx + std::cos(origin_yaw)*dy)/info.resolution;
    if (cx < 0 || cy < 0 || cx >= info.width || cy >= info.height) return false;
    const int centre_cost = map.data[static_cast<std::size_t>(cy)*info.width + static_cast<std::size_t>(cx)];
    if ((!allow_unknown && centre_cost < 0) || centre_cost >= 99) return false;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double map_yaw = tf2::getYaw(info.origin.orientation);
    std::vector<geometry_msgs::msg::Point> polygon;
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (const auto &p : survey_footprint_)
    {
        const double dx = pose.pose.position.x + std::cos(yaw)*p.x - std::sin(yaw)*p.y - info.origin.position.x;
        const double dy = pose.pose.position.y + std::sin(yaw)*p.x + std::cos(yaw)*p.y - info.origin.position.y;
        geometry_msgs::msg::Point cell;
        cell.x = (std::cos(map_yaw)*dx + std::sin(map_yaw)*dy) / info.resolution;
        cell.y = (-std::sin(map_yaw)*dx + std::cos(map_yaw)*dy) / info.resolution;
        polygon.push_back(cell);
        min_x = std::min(min_x, cell.x);
        min_y = std::min(min_y, cell.y);
        max_x = std::max(max_x, cell.x);
        max_y = std::max(max_y, cell.y);
    }
    if (min_x < 0 || min_y < 0 || max_x >= info.width || max_y >= info.height) return false;

    // Check every touched cell, including obstacles under the middle of the footprint
    for (int y = std::floor(min_y); y <= std::floor(max_y); ++y)
    {
        for (int x = std::floor(min_x); x <= std::floor(max_x); ++x)
        {
            const int cost = map.data[y * info.width + x];
            // Unknown isn't an obstacle, Nav2 will check new observations while following the route
            if ((cost >= 0 && cost < 100) || (allow_unknown && cost == -1)) continue;
            bool separated = false;
            for (std::size_t i = 0; i < polygon.size(); ++i)
            {
                const auto &a = polygon[i];
                const auto &b = polygon[(i + 1) % polygon.size()];
                const double nx = b.y - a.y, ny = a.x - b.x;
                double low = std::numeric_limits<double>::infinity(), high = -low;
                for (const auto &v : polygon)
                {
                    const double projection = nx*v.x + ny*v.y;
                    low = std::min(low, projection);
                    high = std::max(high, projection);
                }
                const double centre = nx*(x+0.5) + ny*(y+0.5);
                const double extent = (std::abs(nx) + std::abs(ny))/2;
                if (centre+extent < low || centre-extent > high) { separated = true; break; }
            }
            // Inflation already accounts for clearance, so only raw obstacles block the polygon
            if (!separated) return false;
        }
    }
    return true;
}

bool HuskyMissionExecutorNode::surveyPathClear(const nav_msgs::msg::Path &path)
{
    if (path.header.frame_id != "husky1_map" || path.poses.size() < 2) return false;
    const double age = (now() - rclcpp::Time(global_costmap_->header.stamp)).seconds();
    if (age < 0 || age > 2.0) return false;
    const auto &end = path.poses.back().pose.position;
    const auto &goal = approach_pose_.pose.position;
    if (std::hypot(end.x-goal.x, end.y-goal.y) > 0.25) return false;
    double previous_yaw;
    try { previous_yaw = tf2::getYaw(getCurrentPose("husky1_map", navigation_pose_max_age_).pose.orientation); }
    catch (const std::exception &) { return false; }
    bool first_segment = true;
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
        const auto &a = path.poses[i-1].pose.position;
        const auto &b = path.poses[i].pose.position;
        const double dx = b.x-a.x, dy = b.y-a.y;
        if (!std::isfinite(dx) || !std::isfinite(dy)) return false;
        if (std::hypot(dx, dy) < 1e-6) continue;
        double yaw = std::atan2(dy, dx);
        // A backwards approach doesn't need a half turn just to follow the same route
        if (std::abs(std::remainder(yaw-previous_yaw, 2*M_PI)) > M_PI/2) yaw += M_PI;
        const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / (global_costmap_->info.resolution/2))));
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.orientation.z = std::sin(yaw/2);
        pose.pose.orientation.w = std::cos(yaw/2);
        if (first_segment)
        {
            const double turn = std::remainder(yaw-previous_yaw, 2*M_PI);
            const int turn_steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn)/(M_PI/36))));
            auto turning = pose;
            turning.pose.position = a;
            for (int step = 0; step <= turn_steps; ++step)
            {
                const double heading = previous_yaw + turn*step/turn_steps;
                turning.pose.orientation.z = std::sin(heading/2);
                turning.pose.orientation.w = std::cos(heading/2);
                if (!surveyFootprintClear(turning, true))
                {
                    departure_blocked_ = true;
                    return false;
                }
            }
            first_segment = false;
        }
        previous_yaw = yaw;
        for (int step = 0; step <= steps; ++step)
        {
            pose.pose.position.x = a.x + dx*step/steps;
            pose.pose.position.y = a.y + dy*step/steps;
            if (!surveyFootprintClear(pose, true))
            {
                // Husky can be clear while parked but have no room to turn onto the route
                const auto &start = path.poses.front().pose.position;
                if (std::hypot(pose.pose.position.x-start.x, pose.pose.position.y-start.y) <= 0.35)
                    departure_blocked_ = true;
                RCLCPP_INFO(get_logger(), "Approach path blocked at (%.2f, %.2f), yaw %.2f, centre cost %d, segment %zu/%zu",
                    pose.pose.position.x, pose.pose.position.y, yaw, costmapValue(pose), i, path.poses.size()-1);
                return false;
            }
        }
    }
    // Keep the arrival heading along the checked route, the scanner can see behind Husky too
    approach_pose_.pose.orientation.z = std::sin(previous_yaw/2);
    approach_pose_.pose.orientation.w = std::cos(previous_yaw/2);
    return surveyFootprintClear(approach_pose_, true);
}

// Visit the mapped trees and keep navigation, localisation and DBH retries separate

void HuskyMissionExecutorNode::setupTreeSurvey()
{
    tree_standoff_ = declare_parameter("tree_standoff_m", 1.5);
    measurement_min_range_ = declare_parameter("tree_measurement_min_m", 1.0);
    measurement_max_range_ = declare_parameter("tree_measurement_max_m", 4.0);
    const double goal_margin = declare_parameter("tree_goal_margin_m", 0.35);
    if (!std::isfinite(measurement_min_range_) || !std::isfinite(measurement_max_range_) ||
        !std::isfinite(goal_margin) || goal_margin < 0 ||
        2*goal_margin >= measurement_max_range_-measurement_min_range_ ||
        measurement_min_range_ <= 0 || measurement_min_range_ >= measurement_max_range_ ||
        measurement_min_range_ > tree_standoff_ ||
        measurement_max_range_ < tree_standoff_ || measurement_max_range_ > 4.0)
        throw std::invalid_argument("Invalid tree measurement range");
    // Nav2 can stop 30 cm short, so don't put the goal right on the measurement limit
    for (double preferred : {tree_standoff_, tree_standoff_ + 0.5, tree_standoff_ + 1.0})
    {
        const double radius = std::clamp(preferred, measurement_min_range_+goal_margin, measurement_max_range_-goal_margin);
        if (measurement_radii_.empty() || radius-measurement_radii_.back() > 1e-6)
            measurement_radii_.push_back(radius);
    }
    // Try a slightly closer view after a failed reading, still leaving room for Nav2's tolerance
    const double closer = std::max(measurement_min_range_+goal_margin, tree_standoff_-0.25);
    if (closer < measurement_radii_.front()-1e-6) measurement_radii_.push_back(closer);
    sector_radius_plans_.resize(measurement_radii_.size()*8);
    dbh_timeout_ = declare_parameter("dbh_timeout_s", 10.0);
    tree_motion_timeout_ = declare_parameter("tree_motion_timeout_s", 60.0);
    tree_progress_timeout_ = declare_parameter("tree_progress_timeout_s", 12.0);
    tree_planning_timeout_ = declare_parameter("tree_planning_timeout_s", 3.0);
    tree_navigation_bt_ = declare_parameter<std::string>("tree_navigation_bt", "");
    for (double value : {tree_standoff_, dbh_timeout_, tree_motion_timeout_,
                         tree_progress_timeout_, tree_planning_timeout_})
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid tree survey parameter");

    tree_nav_ = rclcpp_action::create_client<Navigate>(this, "/husky1/navigate_to_pose");
    tree_planner_ = rclcpp_action::create_client<PlanPath>(this, "/husky1/compute_path_to_pose");
    tree_backup_ = rclcpp_action::create_client<BackUp>(this, "/husky1/backup");
    footprint_subscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/husky1/local_costmap/published_footprint", rclcpp::QoS(1),
        std::bind(&HuskyMissionExecutorNode::footprintCb, this, std::placeholders::_1));
    survey_local_map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/husky1/local_costmap/costmap", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
            const auto &info = map->info;
            geometry_msgs::msg::PoseStamped origin;
            origin.header = map->header;
            origin.pose = info.origin;
            if (!map->header.frame_id.empty() && validTargetPose(origin) &&
                std::isfinite(info.resolution) && info.resolution > 0 && info.width > 0 && info.height > 0 &&
                map->data.size() == static_cast<std::size_t>(info.width)*info.height)
                survey_local_map_ = map;
        });
    locate_client_ = create_client<LocateTree>("/husky1/locate_tree");
    sampling_target_publisher_ = create_publisher<rs1_interfaces::msg::TreeSamplingTarget>(
        "/husky1/tree_sampling_target", rclcpp::QoS(1).transient_local().reliable());
    dbh_client_ = create_client<MeasureDBH>("/husky1/measure_dbh");
    survey_timer_ = create_wall_timer(100ms, std::bind(&HuskyMissionExecutorNode::surveyTick, this));
    survey_timer_->cancel();
    survey_server_ = rclcpp_action::create_server<MeasureTrees>(
        this, "/husky1/measure_trees",
        [this](const rclcpp_action::GoalUUID &, MeasureTrees::Goal::ConstSharedPtr goal) {
            const auto &survey = goal->survey;
            const auto &map = survey.tree_map;
            if (husky_state_ != HuskyState::IDLE || (!map.trees.empty() && map.header.frame_id.empty()) ||
                !validSurveyProgress(map.trees.size(), survey.measurements, survey.pending_views))
                return rclcpp_action::GoalResponse::REJECT;
            for (const auto &tree : map.trees)
            {
                const auto &p = tree.location;
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                    return rclcpp_action::GoalResponse::REJECT;
            }
            husky_state_ = HuskyState::MEASURING_TREES;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<SurveyHandle> goal) {
            return goal == survey_goal_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
        },
        [this](std::shared_ptr<SurveyHandle> goal) {
            survey_goal_ = goal;
            readings_ = goal->get_goal()->survey.measurements;
            pending_views_ = goal->get_goal()->survey.pending_views;
            assisted_measurement_ = goal->get_goal()->resume_from_current_pose;
            assisted_tried_.clear();
            assisted_failures_.clear();
            visit_order_.clear();
            visit_ = 0;
            unresolved_.clear();
            blocked_at_.reset();
            resetTreeApproach();
            survey_step_ = SurveyStep::Plan;
            step_started_ = std::chrono::steady_clock::now();
            survey_timer_->reset();
        });
}

void HuskyMissionExecutorNode::surveyTick()
{
    if (!survey_goal_) return;
    if (survey_step_ == SurveyStep::Moving)
    {
        if (navigating_tree_ && !stopping_motion_)
        {
            try
            {
                const auto position = getCurrentPose("husky1_map", navigation_pose_max_age_).pose.position;
                if (!progress_position_ || std::hypot(position.x-progress_position_->x, position.y-progress_position_->y) >= 0.20)
                {
                    progress_position_ = position;
                    progress_at_ = std::chrono::steady_clock::now();
                }
            }
            catch (const std::exception &) {}
        }
        const bool stalled = navigating_tree_ && timedOut(progress_at_, tree_progress_timeout_);
        // Keep ownership until Nav2 actually stops, even if our first cancel request fails
        if ((survey_goal_->is_canceling() || stalled || timedOut(step_started_, motion_limit_)) &&
            cancel_motion_ && !stopping_motion_)
        {
            try
            {
                cancel_motion_();
                stopping_motion_ = true;
                if (!survey_goal_->is_canceling())
                    surveyFeedback(stalled ? "Approach stopped making progress; stopping Nav2" : "Approach timed out; stopping Nav2");
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting to stop Husky: %s", e.what());
            }
        }
        return;
    }
    try
    {
        if (survey_step_ == SurveyStep::Reading || survey_step_ == SurveyStep::Locating)
        {
            // A service request can't be cancelled so collect its reply before handing control back
            if (timedOut(step_started_, dbh_timeout_))
            {
                if (survey_step_ == SurveyStep::Locating) locate_client_->remove_pending_request(*dbh_request_id_);
                else dbh_client_->remove_pending_request(*dbh_request_id_);
                dbh_request_id_.reset();
                finishSurvey("Timed out waiting for tree perception");
            }
            return;
        }
        if (survey_goal_->is_canceling()) return finishSurvey("Tree survey cancelled");
        switch (survey_step_)
        {
            case SurveyStep::Plan: planTreeVisits(); break;
            case SurveyStep::Approach: approachTree(); break;
            case SurveyStep::AssistedSelection: chooseAssistedTree(); break;
            case SurveyStep::Recover: recoverTreeApproach(); break;
            case SurveyStep::TryCurrentPose: checkMeasurementPose(true); break;
            case SurveyStep::CheckPose: checkMeasurementPose(); break;
            default: break;
        }
    }
    catch (const std::exception &e)
    {
        waitForSurvey(e.what());
    }
}

void HuskyMissionExecutorNode::planTreeVisits()
{
    const auto &map = survey_goal_->get_goal()->survey.tree_map;
    if (readings_.size() == map.trees.size()) return finishSurvey("All trees measured", true);
    auto current = getCurrentPose(map.header.frame_id).pose.position;
    std::vector<bool> measured(map.trees.size(), false);
    for (const auto &reading : readings_) measured[reading.tree_index] = true;

    // Keep the original indices so a reading still points to the same tree after a resume
    std::vector<uint32_t> remaining;
    for (std::size_t i = 0; i < map.trees.size(); ++i)
    {
        if (!measured[i]) remaining.push_back(i);
    }

    // Start each search at the tree we just picked
    while (!remaining.empty())
    {
        const auto distance = [&](uint32_t index) {
            const auto &tree = map.trees[index].location;
            return std::hypot(tree.x - current.x, tree.y - current.y);
        };
        const auto nearest = std::min_element(remaining.begin(), remaining.end(),
            [&](uint32_t a, uint32_t b) { return distance(a) < distance(b); });
        current = map.trees[*nearest].location;
        visit_order_.push_back(*nearest);
        remaining.erase(nearest);
    }
    survey_step_ = SurveyStep::Approach;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::publishSamplingTarget(bool active)
{
    if (visit_ >= visit_order_.size()) return;
    const auto &survey = survey_goal_->get_goal()->survey;
    rs1_interfaces::msg::TreeSamplingTarget target;
    target.header = survey.tree_map.header;
    target.mission_id = survey.mission_id;
    target.tree_index = visit_order_[visit_];
    target.centre = refined_trunk_ ? *refined_trunk_ : survey.tree_map.trees[target.tree_index].location;
    target.refined = refined_trunk_.has_value();
    target.active = active;
    target.min_range_m = measurement_min_range_;
    target.max_range_m = measurement_max_range_;
    sampling_target_publisher_->publish(target);
}

void HuskyMissionExecutorNode::nextAssistedTree(const std::string &reason)
{
    if (survey_goal_->is_canceling()) return finishSurvey("Tree survey cancelled");
    const auto index = visit_order_[visit_];
    const auto failure = "Tree " + std::to_string(index) + ": " + reason;
    RCLCPP_WARN(get_logger(), "Assisted sampling: %s", failure.c_str());
    if (!assisted_failures_.empty()) assisted_failures_ += "; ";
    assisted_failures_ += failure;
    assisted_tried_.push_back(index);
    // Do fresh TF lookups on the timer, where readiness failures are retried safely
    survey_step_ = SurveyStep::AssistedSelection;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::chooseAssistedTree()
{
    const auto &map = survey_goal_->get_goal()->survey.tree_map;
    // A shifted map marker must not hide another usable tree beside the operator
    const auto robot = getCurrentPose(map.header.frame_id, navigation_pose_max_age_).pose.position;
    auto next = visit_order_.end();
    double nearest = std::numeric_limits<double>::infinity();
    for (auto candidate = visit_order_.begin()+visit_; candidate != visit_order_.end(); ++candidate)
    {
        if (std::find(assisted_tried_.begin(), assisted_tried_.end(), *candidate) != assisted_tried_.end()) continue;
        const auto &point = map.trees[*candidate].location;
        const double distance = std::hypot(point.x-robot.x, point.y-robot.y);
        const double search_limit = measurement_max_range_ + neighbourBound(map, *candidate, point, 1.8);
        if (distance <= search_limit && distance < nearest)
        {
            next = candidate;
            nearest = distance;
        }
    }
    if (next == visit_order_.end()) return finishSurvey(assisted_failures_ + "; reposition and Resume");
    // Keep every unfinished tree in the order for the later autonomous visits
    std::iter_swap(visit_order_.begin()+visit_, next);
    resetTreeApproach();
    survey_step_ = SurveyStep::Approach;
    step_started_ = std::chrono::steady_clock::now();
}

geometry_msgs::msg::Point HuskyMissionExecutorNode::currentTrunk() const
{
    const auto &map = survey_goal_->get_goal()->survey.tree_map;
    const auto &tree = refined_trunk_ ? *refined_trunk_ : map.trees[visit_order_[visit_]].location;
    const auto transform = tf_buffer_->lookupTransform("husky1_map", map.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::Point trunk;
    tf2::doTransform(tree, trunk, transform);
    return trunk;
}

void HuskyMissionExecutorNode::approachTree()
{
    if (visit_ == visit_order_.size())
        return unresolved_.empty() ? finishSurvey("All trees measured", true) : finishSurvey("Still to measure: " + unresolved_);
    const auto robot_pose = getCurrentPose("husky1_map", navigation_pose_max_age_).pose;
    const auto robot = robot_pose.position;
    const auto trunk = currentTrunk();
    restoreTreeView();
    if (assisted_measurement_)
    {
        // The operator already drove here, so check the trunk before any Nav2 readiness checks
        const auto &map = survey_goal_->get_goal()->survey.tree_map;
        const auto index = visit_order_[visit_];
        const auto &mapped = map.trees[index].location;
        const double distance = std::hypot(robot.x-trunk.x, robot.y-trunk.y);
        const double search_limit = measurement_max_range_ + neighbourBound(map, index, mapped, 1.8);
        publishSamplingTarget();
        if (distance > search_limit)
            return nextAssistedTree("No unmeasured tree nearby: mapped distance " + metres(distance) +
                ", search limit " + metres(search_limit) + "; move at least " + metres(distance-search_limit) + " closer");
        return locateTrunk();
    }
    if (!global_costmap_) return waitForSurvey("Waiting for Husky's costmap");
    const auto age = (now() - rclcpp::Time(global_costmap_->header.stamp)).seconds();
    if (age < 0 || age > 2.0) return waitForSurvey("Waiting for a fresh Husky costmap");
    if (view_arrival_stamp_)
    {
        if (rclcpp::Time(global_costmap_->header.stamp) <= *view_arrival_stamp_)
            return waitForSurvey("Updating obstacle map from the new view");
        view_arrival_stamp_.reset();
    }
    const auto footprint_age = (now() - footprint_stamp_).seconds();
    if (survey_footprint_.empty() || footprint_age < 0 || footprint_age > 2.0)
        return waitForSurvey("Waiting for Nav2's padded footprint");
    if (!tree_nav_->action_server_is_ready()) return waitForSurvey("Waiting for NavigateToPose");
    if (!tree_planner_->action_server_is_ready()) return waitForSurvey("Waiting for Nav2 path planning");

    const double distance = std::hypot(robot.x - trunk.x, robot.y - trunk.y);
    const bool first_attempt = std::none_of(tried_approaches_.begin(), tried_approaches_.end(), [](bool tried) { return tried; });
    if (first_attempt && !first_diameter_) first_bearing_ = std::atan2(robot.y - trunk.y, robot.x - trunk.x);
    if (!refined_trunk_ && first_attempt && distance <= 3.6) return locateTrunk();
    const unsigned int planning_budget = failed_measurement_views_.empty() ? 24 : 8*measurement_radii_.size();
    if (planned_approaches_ >= planning_budget || (planned_approaches_ > 0 && planned_approaches_%8 == 0 && best_approach_))
        return finishApproachSearch();
    double best = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < tried_approaches_.size(); ++i)
    {
        if (tried_approaches_[i]) continue;
        const unsigned int bearing_index = i % 32;
        const unsigned int sector = ((bearing_index + 2) / 4) % 8;
        const unsigned int radius_index = i / 32;
        const double radius = refined_trunk_ && !choosing_viewpoint_ ?
            measurement_radii_[radius_index] : tree_standoff_ + 1.3;
        if (radius < measurement_radii_.front() && failed_measurement_views_.empty()) continue;
        const double angle = first_bearing_ + 2*M_PI*bearing_index/32;
        if (choosing_viewpoint_ && std::cos(angle-viewpoint_bearing_) > 0.5) continue;
        if (!choosing_viewpoint_ && first_diameter_ && std::abs(std::cos(angle-first_view_bearing_)) > 0.71) continue;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "husky1_map";
        pose.header.stamp = now();
        pose.pose.position.x = trunk.x + radius * std::cos(angle);
        pose.pose.position.y = trunk.y + radius * std::sin(angle);
        // A failed scan needs a different position, rotating at the same spot won't reveal more trunk
        if (!choosing_viewpoint_ && std::any_of(failed_measurement_views_.begin(), failed_measurement_views_.end(),
            [&](const auto &view) { return std::hypot(pose.pose.position.x-view.x, pose.pose.position.y-view.y) < 0.60; })) continue;
        bool clear_heading = false;
        const double current_yaw = tf2::getYaw(robot_pose.orientation);
        for (int heading = 0; heading < 16; ++heading)
        {
            const double yaw = current_yaw + heading*M_PI/8;
            pose.pose.orientation.z = std::sin(yaw/2);
            pose.pose.orientation.w = std::cos(yaw/2);
            if (surveyFootprintClear(pose, true)) { clear_heading = true; break; }
        }
        if (!clear_heading) continue;
        // Spread the planning budget across sides and radii before refining nearby angles
        const int cost = costmapValue(pose);
        const double view_cost = first_diameter_ ? 6.0*std::abs(std::cos(angle-first_view_bearing_)) : 0.0;
        const double score = 1000.0*failed_sectors_[sector] + 100.0*sector_plans_[sector] +
            10.0*sector_radius_plans_[radius_index*8+sector] +
            std::hypot(pose.pose.position.x-robot.x, pose.pose.position.y-robot.y) +
            3.0*(cost < 0 ? 1.0 : cost/100.0) + 2.0*std::abs(radius-tree_standoff_) + view_cost;
        if (score < best)
        {
            best = score;
            approach_ = i;
            approach_pose_ = pose;
        }
    }
    if (!std::isfinite(best)) return finishApproachSearch();
    tried_approaches_[approach_] = true;
    const unsigned int sector = ((approach_%32 + 2) / 4) % 8;
    ++sector_plans_[sector];
    ++sector_radius_plans_[(approach_/32)*8+sector];
    PlanPath::Goal plan;
    plan.goal = approach_pose_;
    plan.planner_id = "GridBased";
    surveyFeedback("Checking approach path to tree " + std::to_string(visit_order_[visit_]));
    sendSurveyMotion<PlanPath>(tree_planner_, plan);
}

void HuskyMissionExecutorNode::resetApproachSearch()
{
    tried_approaches_.assign(refined_trunk_ && !choosing_viewpoint_ ? 32*measurement_radii_.size() : 32, false);
    sector_plans_.fill(0);
    std::fill(sector_radius_plans_.begin(), sector_radius_plans_.end(), 0);
    planned_approaches_ = 0;
    departure_blocked_ = false;
    best_approach_.reset();
    best_approach_score_ = std::numeric_limits<double>::infinity();
}

void HuskyMissionExecutorNode::finishApproachSearch()
{
    if (best_approach_)
    {
        approach_ = best_approach_index_;
        approach_pose_ = *best_approach_;
        best_approach_.reset();
        planned_approaches_ = 0;
        departure_blocked_ = false;
        best_approach_score_ = std::numeric_limits<double>::infinity();
        Navigate::Goal navigate;
        navigate.pose = approach_pose_;
        navigate.pose.header.stamp = now();
        navigate.behavior_tree = tree_navigation_bt_;
        const std::string stage = choosing_viewpoint_ ? "Driving to inspect another side of tree " :
            (refined_trunk_ ? "Driving to measure tree " : "Driving to locate tree ");
        surveyFeedback(stage + std::to_string(visit_order_[visit_]));
        sendSurveyMotion<Navigate>(tree_nav_, navigate);
        return;
    }
    if (refined_trunk_ && !moved_viewpoint_ && !choosing_viewpoint_)
    {
        // A wider view is also useful when the goals looked clear but their paths failed
        choosing_viewpoint_ = true;
        moved_viewpoint_ = true;
        const auto robot = getCurrentPose("husky1_map", navigation_pose_max_age_).pose.position;
        const auto trunk = currentTrunk();
        viewpoint_bearing_ = std::atan2(robot.y-trunk.y, robot.x-trunk.x);
        resetApproachSearch();
        survey_step_ = SurveyStep::Approach;
        return;
    }
    recoverTreeApproach();
}

void HuskyMissionExecutorNode::resetTreeApproach()
{
    refined_trunk_.reset();
    first_diameter_.reset();
    approach_ = 0;
    navigation_failures_ = 0;
    localisation_failures_ = 0;
    measurement_failures_ = 0;
    failed_sectors_.fill(0);
    choosing_viewpoint_ = false;
    resetApproachSearch();
    rejected_paths_ = 0;
    moved_viewpoint_ = false;
    view_arrival_stamp_.reset();
    retreat_count_ = 0;
    drive_failed_ = false;
    retreat_start_.reset();
    failed_measurement_views_.clear();
    settled_pose_.reset();
}

void HuskyMissionExecutorNode::restoreTreeView()
{
    const auto index = visit_order_[visit_];
    const auto view = std::find_if(pending_views_.begin(), pending_views_.end(),
        [index](const auto &pending) { return pending.tree_index == index; });
    if (view == pending_views_.end()) return;
    // Store positions in the survey frame so a changed map transform doesn't change the view
    const auto &frame = survey_goal_->get_goal()->survey.tree_map.header.frame_id;
    const auto transform = tf_buffer_->lookupTransform("husky1_map", frame, tf2::TimePointZero);
    geometry_msgs::msg::Point trunk, viewpoint;
    tf2::doTransform(view->trunk, trunk, transform);
    tf2::doTransform(view->viewpoint, viewpoint, transform);
    first_diameter_ = view->dbh_m;
    first_view_bearing_ = std::atan2(viewpoint.y-trunk.y, viewpoint.x-trunk.x);
    first_bearing_ = first_view_bearing_;
}

void HuskyMissionExecutorNode::nextApproach(const std::string &reason)
{
    if (assisted_measurement_) return nextAssistedTree(reason);
    RCLCPP_INFO(get_logger(), "Tree %u: %s (navigation %u/3, localisation %u/3, DBH %u/3)",
        visit_order_[visit_], reason.c_str(), navigation_failures_, localisation_failures_, measurement_failures_);
    tried_approaches_[approach_] = true;
    ++failed_sectors_[((approach_%32 + 2) / 4) % 8];
    planned_approaches_ = 0;
    best_approach_.reset();
    best_approach_score_ = std::numeric_limits<double>::infinity();
    const bool exhausted = navigation_failures_ >= 3 || localisation_failures_ >= 3 || measurement_failures_ >= 3;
    if (!exhausted && retreat_count_ < 2 &&
        std::all_of(tried_approaches_.begin(), tried_approaches_.end(), [](bool tried) { return tried; }))
        return recoverTreeApproach();
    if (std::all_of(tried_approaches_.begin(), tried_approaches_.end(), [](bool tried) { return tried; }) || exhausted)
    {
        return deferTree(reason);
    }
    survey_step_ = SurveyStep::Approach;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::deferTree(const std::string &reason)
{
    // Come back to unresolved trees on Resume after visiting the rest of the map
    if (!unresolved_.empty()) unresolved_ += ", ";
    unresolved_ += std::to_string(visit_order_[visit_]) + " (" + reason + ")";
    ++visit_;
    resetTreeApproach();
    survey_step_ = SurveyStep::Approach;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::locateTrunk()
{
    if (!locate_client_->service_is_ready()) return waitForSurvey("Waiting for /husky1/locate_tree");
    const auto parent = survey_goal_;
    const auto &map = parent->get_goal()->survey.tree_map;
    const auto index = visit_order_[visit_];
    auto request = std::make_shared<LocateTree::Request>();
    request->map_header = map.header;
    request->expected = map.trees[index].location;
    request->search_radius_m = neighbourBound(map, index, request->expected, 1.8);
    publishSamplingTarget();
    const auto robot = getCurrentPose(map.header.frame_id, navigation_pose_max_age_).pose.position;
    const double mapped_distance = std::hypot(robot.x-request->expected.x, robot.y-request->expected.y);
    RCLCPP_INFO(get_logger(), "Locating tree %u: mapped distance %.2f m, map search radius %.2f m, expected (%.2f, %.2f) in %s",
        index, mapped_distance, request->search_radius_m, request->expected.x, request->expected.y, map.header.frame_id.c_str());
    surveyFeedback("Locating tree " + std::to_string(index) + " at mapped distance " + metres(mapped_distance));
    auto pending = locate_client_->async_send_request(request, [this, parent, request](rclcpp::Client<LocateTree>::SharedFuture future) {
        if (survey_goal_ != parent) return;
        dbh_request_id_.reset();
        if (parent->is_canceling()) return finishSurvey("Tree survey cancelled");
        const auto reply = future.get();
        const auto &p = reply->centre;
        const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        const double correction = std::hypot(p.x-request->expected.x, p.y-request->expected.y);
        if (!reply->success || !finite || correction > request->search_radius_m)
        {
            ++localisation_failures_;
            if (drive_failed_) return recoverTreeApproach();
            const auto reason = !reply->success ? reply->message : !finite ? "Locator returned invalid coordinates" :
                "Trunk correction " + metres(correction) + " exceeds identity limit " + metres(request->search_radius_m);
            return nextApproach("Could not identify trunk: " + reason);
        }
        refined_trunk_ = p;
        publishSamplingTarget();
        RCLCPP_INFO(get_logger(), "Located tree %u: map correction %.2f m, trunk (%.2f, %.2f) in %s",
            visit_order_[visit_], std::hypot(p.x-request->expected.x, p.y-request->expected.y),
            p.x, p.y, request->map_header.frame_id.c_str());
        navigation_failures_ = 0;
        approach_ = 0;
        resetApproachSearch();
        rejected_paths_ = 0;
        // After manual repositioning or a coarse approach, this spot may already be good enough
        settled_pose_.reset();
        survey_step_ = drive_failed_ ? SurveyStep::Recover : SurveyStep::TryCurrentPose;
        step_started_ = std::chrono::steady_clock::now();
    });
    dbh_request_id_ = pending.request_id;
    survey_step_ = SurveyStep::Locating;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::handleBlockedDrive()
{
    drive_failed_ = true;
    const auto index = visit_order_[visit_];
    try
    {
        const auto pose = getCurrentPose("husky1_map", navigation_pose_max_age_);
        const auto robot = pose.pose.position;
        // Changing trees won't help if Nav2 keeps stopping at the same spot
        if (blocked_at_ && blocked_tree_ != index && !surveyRobotClear(pose) &&
            std::hypot(robot.x - blocked_at_->x, robot.y - blocked_at_->y) < 0.25)
            return finishSurvey("Nav2 stopped at the same position approaching two different trees; reposition Husky");
        blocked_at_ = robot;
        blocked_tree_ = index;

        // Use the current view if Nav2 got within scanning range before it stopped
        const auto trunk = currentTrunk();
        if (!refined_trunk_ && localisation_failures_ < 3 &&
            std::hypot(robot.x - trunk.x, robot.y - trunk.y) <= 3.6 && locate_client_->service_is_ready())
            return locateTrunk();
    }
    catch (const std::runtime_error &)
    {
        // The latest pose may already be stale when Nav2 returns
    }
    recoverTreeApproach();
}

void HuskyMissionExecutorNode::recoverTreeApproach()
{
    // Retry fresh-pose lookup on the timer, including when Nav2's result arrived between TF updates
    if (survey_step_ != SurveyStep::Recover)
    {
        survey_step_ = SurveyStep::Recover;
        step_started_ = std::chrono::steady_clock::now();
        return;
    }
    // An inaccessible goal doesn't mean Husky itself is stuck
    const auto current = getCurrentPose("husky1_map", navigation_pose_max_age_);
    if (!surveyMapsFresh()) return waitForSurvey("Waiting for fresh costmaps before recovery");
    if (surveyRobotClear(current) && !departure_blocked_)
    {
        if (!drive_failed_) return deferTree("No safe reachable stand-off pose");
        drive_failed_ = false;
        if (navigation_failures_ >= 3 || localisation_failures_ >= 3)
            return deferTree("Approach attempts exhausted while Husky is clear");
        const auto trunk = currentTrunk();
        const double distance = std::hypot(current.pose.position.x-trunk.x, current.pose.position.y-trunk.y);
        if (refined_trunk_ && distance >= measurement_min_range_ && distance <= measurement_max_range_)
        {
            // Nav2 may stop at a perfectly usable view before reaching the requested goal
            approach_pose_ = current;
            settled_pose_.reset();
            survey_step_ = SurveyStep::CheckPose;
            step_started_ = std::chrono::steady_clock::now();
            return;
        }
        return nextApproach("Approach failed but Husky is clear; trying another view");
    }
    // One retreat at this spot, with one more available if Husky later gets stuck somewhere else
    const bool new_position = !retreat_start_ || std::hypot(current.pose.position.x-retreat_start_->x,
        current.pose.position.y-retreat_start_->y) >= 1.0;
    if (retreat_count_ < 2 && new_position)
    {
        if (!tree_backup_->action_server_is_ready()) return waitForSurvey("Waiting for Nav2 BackUp");
        // Nav2 checks the whole retreat for collisions and stops if the way behind isn't clear
        ++retreat_count_;
        retreat_start_ = current.pose.position;
        BackUp::Goal goal;
        goal.target.x = -0.50;
        goal.speed = 0.10;
        goal.time_allowance.sec = 10;
        surveyFeedback("Backing away before choosing another approach");
        sendSurveyMotion<BackUp>(tree_backup_, goal);
        return;
    }
    finishSurvey("Husky is still blocked after retreat; reposition Husky");
}

template<typename Action>
void HuskyMissionExecutorNode::sendSurveyMotion(
    const typename rclcpp_action::Client<Action>::SharedPtr &client, const typename Action::Goal &goal)
{
    const auto parent = survey_goal_;
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, parent, client](auto handle) {
        if (parent != survey_goal_) return;
        if (!handle)
        {
            if constexpr (std::is_same_v<Action, BackUp>) return finishSurvey("Nav2 rejected retreat; reposition Husky");
            if constexpr (std::is_same_v<Action, PlanPath>)
            {
                ++planned_approaches_;
                survey_step_ = SurveyStep::Approach;
                step_started_ = std::chrono::steady_clock::now();
                return;
            }
            ++navigation_failures_;
            return nextApproach("Nav2 rejected approach");
        }
        cancel_motion_ = [this, client, handle, parent]() {
            client->async_cancel_goal(handle, [this, parent, handle](auto reply) {
                if (survey_goal_ == parent && handle->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING &&
                    (!reply || reply->goals_canceling.empty())) stopping_motion_ = false;
            });
        };
    };
    options.result_callback = [this, parent](const auto &result) {
        if (survey_goal_ != parent) return;
        cancel_motion_ = {};
        if (parent->is_canceling()) return finishSurvey("Tree survey cancelled");
        if constexpr (std::is_same_v<Action, PlanPath>)
        {
            ++planned_approaches_;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && surveyPathClear(result.result->path))
            {
                // Compare actual routes so the nearest-looking goal doesn't win a large detour
                double length = 0.0, cost = 0.0;
                const auto &poses = result.result->path.poses;
                for (std::size_t i = 1; i < poses.size(); ++i)
                {
                    const auto &a = poses[i-1].pose.position;
                    const auto &b = poses[i].pose.position;
                    const double step = std::hypot(b.x-a.x, b.y-a.y);
                    const int value = costmapValue(poses[i]);
                    length += step;
                    cost += step*(value < 0 ? 1.0 : value/100.0);
                }
                const unsigned int sector = ((approach_%32 + 2) / 4) % 8;
                const double radius = refined_trunk_ && !choosing_viewpoint_ ? measurement_radii_[approach_/32] : tree_standoff_+1.3;
                const double angle = first_bearing_ + 2*M_PI*(approach_%32)/32;
                const double view_cost = first_diameter_ ? 6.0*std::abs(std::cos(angle-first_view_bearing_)) : 0.0;
                const double score = length + 3.0*cost + 2.0*std::abs(radius-tree_standoff_) +
                    1000.0*failed_sectors_[sector] + view_cost;
                if (score < best_approach_score_)
                {
                    best_approach_score_ = score;
                    best_approach_ = approach_pose_;
                    best_approach_index_ = approach_;
                }
            }
            else
            {
                ++rejected_paths_;
                surveyFeedback("Rejected unreachable approach to tree " + std::to_string(visit_order_[visit_]));
            }
            survey_step_ = SurveyStep::Approach;
            step_started_ = std::chrono::steady_clock::now();
            return;
        }
        if constexpr (std::is_same_v<Action, BackUp>)
        {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
            {
                bool clear_after_retreat = false;
                try
                {
                    const auto current = getCurrentPose("husky1_map", navigation_pose_max_age_);
                    const auto &p = current.pose.position;
                    clear_after_retreat = retreat_start_ &&
                        std::hypot(p.x-retreat_start_->x, p.y-retreat_start_->y) >= 0.20 && surveyRobotClear(current);
                }
                catch (const std::exception &) {}
                // A partial retreat can be enough to find a new path once Nav2 has stopped
                if (!clear_after_retreat) return finishSurvey("Retreat blocked or timed out; reposition Husky");
            }
            drive_failed_ = false;
            resetApproachSearch();
            rejected_paths_ = 0;
            view_arrival_stamp_ = now();
            survey_step_ = SurveyStep::Approach;
            step_started_ = std::chrono::steady_clock::now();
            return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            ++navigation_failures_;
            if constexpr (std::is_same_v<Action, Navigate>) return handleBlockedDrive();
            return nextApproach();
        }
        if constexpr (std::is_same_v<Action, Navigate>)
        {
            blocked_at_.reset();
            drive_failed_ = false;
        }
        survey_step_ = SurveyStep::CheckPose;
        step_started_ = std::chrono::steady_clock::now();
    };
    stopping_motion_ = false;
    navigating_tree_ = std::is_same_v<Action, Navigate>;
    progress_position_.reset();
    progress_at_ = std::chrono::steady_clock::now();
    motion_limit_ = tree_motion_timeout_;
    if constexpr (std::is_same_v<Action, PlanPath>) motion_limit_ = tree_planning_timeout_;
    if constexpr (std::is_same_v<Action, BackUp>) motion_limit_ = 15.0;
    settled_pose_.reset();
    client->async_send_goal(goal, options);
    survey_step_ = SurveyStep::Moving;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::checkMeasurementPose(bool try_current_pose)
{
    const auto plan_from_here = [this]() {
        settled_pose_.reset();
        survey_step_ = SurveyStep::Approach;
        step_started_ = std::chrono::steady_clock::now();
    };
    const auto current = getCurrentPose("husky1_map", navigation_pose_max_age_);
    const auto &robot = current.pose;
    if (choosing_viewpoint_)
    {
        choosing_viewpoint_ = false;
        resetApproachSearch();
        rejected_paths_ = 0;
        view_arrival_stamp_ = now();
        survey_step_ = SurveyStep::Approach;
        step_started_ = std::chrono::steady_clock::now();
        return;
    }
    const auto trunk = currentTrunk();
    const double dx = trunk.x - robot.position.x;
    const double dy = trunk.y - robot.position.y;
    if (!refined_trunk_) return locateTrunk();
    const double distance = std::hypot(dx, dy);
    if (distance < measurement_min_range_ || distance > measurement_max_range_)
    {
        if (assisted_measurement_) return nextApproach("Outside the measurement range: located trunk is " +
            metres(distance) + " away, allowed " + metres(measurement_min_range_) + " to " + metres(measurement_max_range_) +
            (distance > measurement_max_range_ ? "; move at least " + metres(distance-measurement_max_range_) + " closer" :
                "; move at least " + metres(measurement_min_range_-distance) + " farther away"));
        if (try_current_pose) return plan_from_here();
        ++navigation_failures_;
        return nextApproach("Stopped outside the measurement range");
    }

    // Refresh stale maps before deciding that the parked footprint needs a retreat
    // A parked reading needs good scans, not permission to drive through the costmap
    // Keep the usual footprint checks for every autonomous approach
    if (!assisted_measurement_ && !surveyMapsFresh()) return waitForSurvey("Waiting for fresh costmaps before measuring");
    if (!assisted_measurement_ && !surveyRobotClear(current))
    {
        if (try_current_pose) return plan_from_here();
        return recoverTreeApproach();
    }
    if (std::any_of(failed_measurement_views_.begin(), failed_measurement_views_.end(),
        [&](const auto &view) { return std::hypot(robot.position.x-view.x, robot.position.y-view.y) < 0.30; }))
    {
        if (try_current_pose) return plan_from_here();
        return nextApproach("Still at the failed DBH view; choosing another position");
    }
    // Wait for Husky to settle, the 360 degree scanner doesn't need a face-tree heading
    if (!settled_pose_ || std::hypot(robot.position.x-settled_pose_->pose.position.x,
            robot.position.y-settled_pose_->pose.position.y) > 0.02 ||
        std::abs(std::remainder(tf2::getYaw(robot.orientation)-tf2::getYaw(settled_pose_->pose.orientation), 2*M_PI)) > 0.03)
        settled_pose_ = current;
    if ((rclcpp::Time(current.header.stamp)-rclcpp::Time(settled_pose_->header.stamp)).seconds() < 0.5)
        return waitForSurvey("Waiting for Husky to settle before measuring");
    if (!dbh_client_->service_is_ready()) return waitForSurvey("Waiting for /husky1/measure_dbh");
    restoreTreeView();
    const double bearing = std::atan2(-dy, -dx);
    if (first_diameter_ && std::abs(std::cos(bearing - first_view_bearing_)) > 0.71)
    {
        if (assisted_measurement_) return nextApproach("Second view needed from another side of the same tree");
        if (try_current_pose) return plan_from_here();
        return nextApproach("Second bearing too close to the first");
    }
    if (try_current_pose) RCLCPP_INFO(get_logger(), "Using current pose to measure tree %u at %.2f m",
        visit_order_[visit_], distance);
    requestDBH(bearing);
}

void HuskyMissionExecutorNode::requestDBH(double bearing)
{
    // Reaching a usable measurement view ends this navigation attempt
    navigation_failures_ = 0;
    const auto view = getCurrentPose("husky1_map", navigation_pose_max_age_).pose.position;
    const auto parent = survey_goal_;
    const auto &map = parent->get_goal()->survey.tree_map;
    const auto survey_view = getCurrentPose(map.header.frame_id, navigation_pose_max_age_).pose.position;
    const auto index = visit_order_[visit_];
    auto request = std::make_shared<MeasureDBH::Request>();
    request->map_header = map.header;
    request->tree_index = index;
    request->tree = map.trees[index];
    request->tree.location = *refined_trunk_;
    if (map.trees.size() > 1)
        request->max_center_offset_m = neighbourBound(map, index, *refined_trunk_,
            std::numeric_limits<double>::infinity());
    surveyFeedback("Measuring tree " + std::to_string(index));
    auto pending = dbh_client_->async_send_request(request, [this, parent, index, bearing, view, survey_view](rclcpp::Client<MeasureDBH>::SharedFuture future) {
        if (parent != survey_goal_) return;
        dbh_request_id_.reset();
        const auto reply = future.get();
        if (!reply->success || !std::isfinite(reply->dbh_m) || reply->dbh_m <= 0)
        {
            ++measurement_failures_;
            failed_measurement_views_.push_back(view);
            return nextApproach("No valid DBH: " + reply->message);
        }
        if (!first_diameter_ && reply->dbh_m >= 0.45)
        {
            // A broad trunk can give a repeatable width from the wrong arc
            first_diameter_ = reply->dbh_m;
            first_view_bearing_ = bearing;
            first_bearing_ = bearing;
            rs1_interfaces::msg::PendingTreeMeasurement pending;
            pending.tree_index = index;
            pending.dbh_m = reply->dbh_m;
            pending.trunk = *refined_trunk_;
            pending.viewpoint = survey_view;
            pending_views_.push_back(pending);
            // A valid first reading can continue autonomously, but survives the next pause
            assisted_measurement_ = false;
            if (parent->is_canceling()) return finishSurvey("First view saved; tree survey cancelled");
            approach_ = 0;
            resetApproachSearch();
            rejected_paths_ = 0;
            return nextApproach("Second view needed");
        }
        if (first_diameter_ && std::abs(reply->dbh_m - *first_diameter_) > 0.10 * *first_diameter_)
        {
            ++measurement_failures_;
            failed_measurement_views_.push_back(view);
            return nextApproach("The two views disagree on trunk width");
        }

        rs1_interfaces::msg::TreeMeasurement reading;
        reading.tree_index = index;
        reading.dbh_m = first_diameter_ ? (*first_diameter_ + reply->dbh_m)/2 : reply->dbh_m;
        readings_.push_back(reading);
        publishSamplingTarget(false);
        pending_views_.erase(std::remove_if(pending_views_.begin(), pending_views_.end(),
            [index](const auto &pending) { return pending.tree_index == index; }), pending_views_.end());
        assisted_measurement_ = false;
        if (parent->is_canceling()) return finishSurvey("Measurement saved; tree survey cancelled");
        ++visit_;
        resetTreeApproach();
        survey_step_ = SurveyStep::Approach;
        step_started_ = std::chrono::steady_clock::now();
    });
    dbh_request_id_ = pending.request_id;
    survey_step_ = SurveyStep::Reading;
    step_started_ = std::chrono::steady_clock::now();
}

void HuskyMissionExecutorNode::surveyFeedback(const std::string &stage)
{
    auto feedback = std::make_shared<MeasureTrees::Feedback>();
    if (visit_ < visit_order_.size()) feedback->tree_index = visit_order_[visit_];
    feedback->measured_count = readings_.size();
    feedback->total_count = survey_goal_->get_goal()->survey.tree_map.trees.size();
    feedback->stage = stage;
    survey_goal_->publish_feedback(feedback);
}

void HuskyMissionExecutorNode::waitForSurvey(const std::string &reason)
{
    if (timedOut(step_started_, waypoint_wait_timeout_)) finishSurvey(reason);
    else surveyFeedback(reason);
}

void HuskyMissionExecutorNode::finishSurvey(const std::string &message, bool success)
{
    RCLCPP_INFO(get_logger(), "Tree survey: %s (%zu measured, %zu pending views)",
        message.c_str(), readings_.size(), pending_views_.size());
    auto result = std::make_shared<MeasureTrees::Result>();
    result->measurements = readings_;
    result->pending_views = pending_views_;
    result->message = message;
    if (survey_goal_->is_canceling()) survey_goal_->canceled(result);
    else if (success) survey_goal_->succeed(result);
    else survey_goal_->abort(result);
    survey_timer_->cancel();
    survey_goal_.reset();
    husky_state_ = HuskyState::IDLE;
}

}  // namespace rs1_mission
