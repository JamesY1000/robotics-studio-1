// This guy should handle the execution of mission commands for just the parrot alone:
// For now it's just:
// Fly from base to ROI
// Produce a traversable route for Husky

// CURRENTLY NOT IN SCOPE:
// 2.
// Descends below canopy
// Detects and maps trees
// Generates and sends tree map back to mission control or straight to husky?
// Fly from ROI back to base

// Note** - information generated between robots is shared directly


#include "parrot_mission_executor.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;

namespace rs1_mission
{

ParrotMissionExecutorNode::ParrotMissionExecutorNode() : Node("parrot_mission_executor_node"),
    parrot_state_{ParrotState::IDLE}
{
    // Set arrival and ground-path tolerances
    roi_tolerance_ = this->declare_parameter<double>("roi_tolerance", 1.0);
    ground_path_goal_tolerance_ = this->declare_parameter<double>("ground_path_goal_tolerance", 0.5);
    if (!std::isfinite(roi_tolerance_) || roi_tolerance_ <= 0.0 ||
        !std::isfinite(ground_path_goal_tolerance_) || ground_path_goal_tolerance_ < 0.0)
    {
        throw std::invalid_argument("ROI and path endpoint tolerances must be finite and valid");
    }

    // Create tf buffers and listeners
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // Clients for GetCurrentPose service, ComputePathToPose action and SetGroundPath service
    get_current_pose_client_ = this->create_client<rs1_interfaces::srv::GetCurrentPose>(
        "/husky1/get_current_pose");

    compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        this, "/parrot1/compute_path_to_pose");

    set_ground_path_client_ = this->create_client<rs1_interfaces::srv::SetGroundPath>(
        "/husky1/set_ground_path");

    // Nav2pose client
    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "/parrot1/navigate_to_pose"
    );

    navigate_to_pose_goal_options_.goal_response_callback = std::bind(
        &ParrotMissionExecutorNode::navigateToPoseResponseCb,
        this, std::placeholders::_1
    );

    navigate_to_pose_goal_options_.result_callback = std::bind(
        &ParrotMissionExecutorNode::navigateToPoseResultCb,
        this, std::placeholders::_1
    );

    // Sub to costmap to check handle exploration fallback
    global_costmap_subscription_ =this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/parrot1/global_costmap/costmap",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
        {
            global_costmap_ = map;
            global_costmap_received_at_ =
                std::chrono::steady_clock::now();
        }
    );


    // ExploreToPose client
    explore_to_pose_client_ = rclcpp_action::create_client<rs1_interfaces::action::ExploreToPose>(
    this,
    "/parrot1/explore_to_pose"
    ); // TODO (james): Change the hardcoded names to global variables we store somewhere - maybe in rs1_interfaces

    explore_to_pose_goal_options_.goal_response_callback = std::bind(
    &ParrotMissionExecutorNode::exploreToPoseResponseCb, this, std::placeholders::_1);

    explore_to_pose_goal_options_.result_callback = std::bind(
    &ParrotMissionExecutorNode::exploreToPoseResultCb, this, std::placeholders::_1);

    // TraverseToROI server
    traverse_to_roi_server_ = rclcpp_action::create_server<rs1_interfaces::action::TraverseToROI>(
    this,
    "/parrot1/traverse_to_roi",
    std::bind(&ParrotMissionExecutorNode::traverseToROIGoalRequestCb, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ParrotMissionExecutorNode::traverseToROICancelRequestCb, this, std::placeholders::_1),
    std::bind(&ParrotMissionExecutorNode::traverseToROIGoalAcceptedCb, this, std::placeholders::_1)
    );

  RCLCPP_INFO(this->get_logger(), "Parrot mission executor initialised to IDLE");
}

rclcpp_action::GoalResponse ParrotMissionExecutorNode::traverseToROIGoalRequestCb(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Goal> goal)
{
    // Reject goal if parrot is busy, if goal is invalid, or if nav2 is unavailable
    if (parrot_state_ != ParrotState::IDLE)
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Parrot is currently busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validTargetPose(goal->target_roi))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Target ROI is invalid");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!navigate_to_pose_client_->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: NavigateToPose server for Parrot is unavailable");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Set the parrot state to TRAVERSING_TO_ROI
    parrot_state_ = ParrotState::TRAVERSING_TO_ROI;

    // Accept and return goalhandle
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ParrotMissionExecutorNode::traverseToROICancelRequestCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle)
{
    // Only cancel the current mission
    if (!goal_handle || goal_handle != traverse_to_roi_goal_handle_ || !goal_handle->is_active())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // Once sent, the service request cannot be recalled from husky
    if (ground_path_handoff_pending_)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot cancel Parrot during ground-path handoff; cancel Husky's mission separately");
        return rclcpp_action::CancelResponse::REJECT;
    }

    // ROS marks the goal CANCELING after this callback returns
    cancel_timer_ = this->create_wall_timer(50ms,
        std::bind(&ParrotMissionExecutorNode::cancelTraversalToROI, this));
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ParrotMissionExecutorNode::traverseToROIGoalAcceptedCb(std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle)
{
    traverse_to_roi_goal_handle_ = goal_handle;

    // Start traversal to roi
    traverseToROI();
}

// Responses received from nav2's NavigateToPose calls
void ParrotMissionExecutorNode::navigateToPoseResponseCb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    action_goal_pending_ = false;

    // Check goal handle isn't empty
    if (!goal_handle)
    {
        abortTraversalToROI("Nav2 rejected the direct navigation goal");
        return;
    }
    navigate_to_pose_goal_handle_ = goal_handle;
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        cancelTraversalToROI();
        return;
    }

    // Otherwise just log that goal was accepted
    RCLCPP_DEBUG(this->get_logger(), "Direct navigation goal accepted");
}

void ParrotMissionExecutorNode::navigateToPoseResultCb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    // Check goal handle isn't empty
    if (!traverse_to_roi_goal_handle_ || !navigate_to_pose_goal_handle_ ||
        result.goal_id != navigate_to_pose_goal_handle_->get_goal_id())
    {
        return;
    }

    // Stop here if the mission was canceled while this action was running
    navigate_to_pose_goal_handle_.reset();
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        finishCancelledTraversal();
        return;
    }

    // Handle each result case for the NavigateToPose action: SUCCEEDED, ABORTED, CANCELLED
    switch (result.code)
    {
        // Check arrival before generating the ground path
        case rclcpp_action::ResultCode::SUCCEEDED:
            generateGroundPath();
            break;

        // If direct navigation fails, try exploration to roi
        case rclcpp_action::ResultCode::ABORTED:
            tryExplorationFallback(); // Check whether or not it failed due to roi lying outside global costmap
            break;

        case rclcpp_action::ResultCode::CANCELED:
            abortTraversalToROI("Direct navigation was canceled");
            break;

        default:
            abortTraversalToROI(
                "Direct navigation returned an unexpected result");
            break;
    }
}

// Responses received from parrot's ExploreToPose server calls
void ParrotMissionExecutorNode::exploreToPoseResponseCb(rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr goal_handle)
{
    action_goal_pending_ = false;

    // Check that goal handle is not empty
    if (!goal_handle)
    {
        abortTraversalToROI("Parrot's ExploreToPose rejected navigation goal");
        return;
    }

    explore_to_pose_goal_handle_ = goal_handle;
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        cancelTraversalToROI();
        return;
    }

    // Otherwise just log that goal was accepteed
    RCLCPP_DEBUG(this->get_logger(), "Parrot's ExploreToPose goal was accepted");
}

void ParrotMissionExecutorNode::exploreToPoseResultCb(const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::WrappedResult &result)
{
    // Check the goal handle is not nempty
    if (!traverse_to_roi_goal_handle_ || !explore_to_pose_goal_handle_ ||
        result.goal_id != explore_to_pose_goal_handle_->get_goal_id())
    {
        return;
    }

    // Stop here if the mission was canceled while this action was running
    explore_to_pose_goal_handle_.reset();
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        finishCancelledTraversal();
        return;
    }

    const std::string detail = result.result ? result.result->message : std::string{};

    // Handle each result case for the ExploreToPose action: SUCCEEDED, ABORTED, CANCELLED
    switch (result.code)
    {
        // Check arrival before generating the ground path for husky
        case rclcpp_action::ResultCode::SUCCEEDED:
            generateGroundPath();
            break;

        // Pass through exploration logging if available
        case rclcpp_action::ResultCode::ABORTED:
            abortTraversalToROI(detail.empty() ? "Exploration failed; no reason supplied" : "Exploration failed: " + detail);
            break;

        case rclcpp_action::ResultCode::CANCELED:
            abortTraversalToROI(detail.empty() ? "Exploration canceled; no reason supplied" : "Exploration canceled: " + detail);
            break;

        default:
            abortTraversalToROI("Parrot's exploreToPose server returned unexpected result");
            break;
    }
}

void ParrotMissionExecutorNode::getCurrentPoseResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    rclcpp::Client<rs1_interfaces::srv::GetCurrentPose>::SharedFuture future)
{
    // Ignore response if mission has already ended
    if (!mission_goal || mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_active()) return;

    get_current_pose_request_id_ = -1;
    if (mission_goal->is_canceling())
    {
        finishCancelledTraversal();
        return;
    }

    // Read the response
    rs1_interfaces::srv::GetCurrentPose::Response::SharedPtr response;

    try
    {
        response = future.get();
    }
    catch(const std::exception& e)
    {
        abortTraversalToROI(std::string("Could not read Husky pose response") + e.what());
        return;
    }

    // Check whether husky provided it's pose
    if (!response->success)
    {
        abortTraversalToROI("Could not obtain Husky pose: " + response->message);
        return;
    }

    // Check husky's pose is valid and in parrot's map frame id
    const auto &husky_pose = response->pose;

    if (husky_pose.header.frame_id != "parrot1_map" || !validTargetPose(husky_pose))
    {
        abortTraversalToROI("Husky returned an invalid pose or frame");
        return;
    }

    // Check husky's pose is recent
    const rclcpp::Time stamp(husky_pose.header.stamp, this->get_clock()->get_clock_type());
    const double age = (this->now() - stamp).seconds();

    // Let's just say 2s for now
    if (stamp.nanoseconds() == 0 || age < 0.0 || age > 2.0)
    {
        abortTraversalToROI("Husky returned a stale pose");
        return;
    }

    // Compute ground path
    husky_current_pose_ = husky_pose;
    computeGroundPath(husky_current_pose_);
}

void ParrotMissionExecutorNode::computePathToPoseResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr planner_goal
)
{
    // Check mission, mission goal, and planner goals are valid and active
    if (!mission_goal || mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_active())
    {
        return;
    }

    action_goal_pending_ = false;
    if (!planner_goal)
    {
        abortTraversalToROI("ComputePathToPose rejected the goal");
        return;
    }

    compute_path_to_pose_goal_handle_ = planner_goal;
    if (mission_goal->is_canceling())
    {
        cancelTraversalToROI();
    }
}

void ParrotMissionExecutorNode::computePathToPoseResultCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult &result)
{
    // Check mission is active
    if (!mission_goal || mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_active())
    {
        return;
    }

    // Ignore old planner results and finish cancellation before sending any path
    if (!compute_path_to_pose_goal_handle_ || result.goal_id != compute_path_to_pose_goal_handle_->get_goal_id()) return;
    compute_path_to_pose_goal_handle_.reset();
    if (mission_goal->is_canceling())
    {
        finishCancelledTraversal();
        return;
    }

    // Log if ComputePathToPose action failed
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
        abortTraversalToROI("Ground-path planning failed or was canceled");
        return;
    }

    // Grab path from successful action result
    const auto &ground_path = result.result->path;

    // Validate ground path and psoes within
    if (ground_path.poses.empty()) {
        abortTraversalToROI("Planner returned an empty path");
        return;
    }

    if (ground_path.header.frame_id != "parrot1_map") {
        abortTraversalToROI("Planner returned a path outside parrot1_map");
        return;
    }


    for (const auto &pose : ground_path.poses) {
        if (pose.header.frame_id != ground_path.header.frame_id ||
            !validTargetPose(pose))
        {
            abortTraversalToROI("Planner returned an invalid path pose");
            return;
        }
    }

    // Check the ground path finishes near the requested ROI
    const auto &endpoint = ground_path.poses.back().pose.position;
    const auto &target = mission_goal->get_goal()->target_roi.pose.position;
    const double endpoint_error = std::hypot(endpoint.x - target.x, endpoint.y - target.y);
    if (!std::isfinite(endpoint_error) || endpoint_error > ground_path_goal_tolerance_)
    {
        abortTraversalToROI("Planner reported success but ground path does not reach the requested ROI");
        return;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "Computed ground path with %zu poses in %s",
        ground_path.poses.size(),
        ground_path.header.frame_id.c_str());

    // Send path off to Husky via the SetGroundPath service
    sendGroundPath(ground_path);
}

void ParrotMissionExecutorNode::setGroundPathResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    rclcpp::Client<rs1_interfaces::srv::SetGroundPath>::SharedFuture future)
{
    // Check mission is active - ignore responses from old missions
    if (!mission_goal || mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_active()) return;

    ground_path_handoff_pending_ = false;

    // Abort mission if service response is unretrievable
    rs1_interfaces::srv::SetGroundPath::Response::SharedPtr response;
    try
    {
        response = future.get();
    }
    catch(const std::exception& e)
    {
        abortTraversalToROI(std::string("Could not read Husky handoff response: ") + e.what());
        return;
    }

    // Abort mission if Husky rejected the ground path
    if (!response->accepted)
    {
        abortTraversalToROI("Husky rejected the ground path: " + response->message);
        return;
    }

    auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
    result->message = "Reached ROI; ground path accepted by Husky";

    // If Husky accepts the ground path, parrot has completed its phase 1 tasks:
    // TraverseToROI
    mission_goal->succeed(result);
    returnToIdle();

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
}

void ParrotMissionExecutorNode::traverseToROI()
{
    // Explore first if the ROI is outside the current global costmap
    try
    {
        if (targetOutsideGlobalCostmap())
        {
            exploreToROI();
            return;
        }
    }
    catch (const std::exception &e)
    {
        abortTraversalToROI(e.what());
        return;
    }

    // Update feedback for mission coordinator
    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = "Navigating directly to ROI";
    traverse_to_roi_goal_handle_->publish_feedback(feedback);

    // Create a nav2pose goal using the target_roi received from mission coordinator
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose = traverse_to_roi_goal_handle_->get_goal()->target_roi;
    goal.pose.header.stamp = this->now();

    // Send goal async to the nav2pose server
    try
    {
        action_goal_pending_ = true;
        navigate_to_pose_client_->async_send_goal(goal, navigate_to_pose_goal_options_);
    }
    catch (const std::exception &e)
    {
        action_goal_pending_ = false;
        abortTraversalToROI(std::string("Could not send direct navigation goal: ") + e.what());
    }
}

void ParrotMissionExecutorNode::tryExplorationFallback()
{
    // Only switch to exploration if the ROI is outside the costmap
    try
    {
        if (!targetOutsideGlobalCostmap())
        {
            abortTraversalToROI("Direct navigation failed with the ROI inside the global costmap");
            return;
        }
    }
    catch (const std::exception &e)
    {
        abortTraversalToROI(e.what());
        return;
    }

    exploreToROI();
}

bool ParrotMissionExecutorNode::targetOutsideGlobalCostmap() const
{
    // We need a map to decide whether the ROI is outside it
    if (!global_costmap_)
    {
        throw std::runtime_error("No global costmap available to check the ROI");
    }

    const auto &map = *global_costmap_;

    // Reject old map data
    const double map_age = (this->now() - rclcpp::Time(map.header.stamp, this->get_clock()->get_clock_type())).seconds();

    if (map_age < 0.0 || map_age > 2.0 || std::chrono::steady_clock::now() - global_costmap_received_at_ > 2s)
    {
        throw std::runtime_error("Global costmap is stale");
    }

    const auto &target = traverse_to_roi_goal_handle_->get_goal()->target_roi;

    // Coordinates can only be compared directly in the same frame
    if (target.header.frame_id != map.header.frame_id)
    {
        throw std::runtime_error("ROI and global costmap frames differ");
    }

    const auto &info = map.info;
    const auto &origin = info.origin.position;
    const auto &q = info.origin.orientation;

    // Check that the map describes valid planar bounds
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

    if (!std::isfinite(info.resolution) || info.resolution <= 0.0 ||
        info.width == 0 || info.height == 0 ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
        !std::isfinite(norm_squared) ||
        std::abs(norm_squared - 1.0) > 1e-3 ||
        std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6)
    {
        throw std::runtime_error("Global costmap has invalid 2D bounds");
    }

    // Express the ROI relative to the grid origin and orientation
    const double yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);

    const double dx = target.pose.position.x - origin.x;
    const double dy = target.pose.position.y - origin.y;

    const double grid_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double grid_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    const double width = static_cast<double>(info.width) * info.resolution;
    const double height = static_cast<double>(info.height) * info.resolution;

    return grid_x < 0.0 || grid_x >= width || grid_y < 0.0 || grid_y >= height;
}


void ParrotMissionExecutorNode::exploreToROI()
{
    // Check the exploration action server is available
    if (!explore_to_pose_client_->action_server_is_ready())
    {
        abortTraversalToROI("ROI is outside the global costmap but ExploreToPose is unavailable");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "ROI is outside the global costmap; starting exploration");

    // Update feedback for mission coordinator
    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = "Exploring to ROI";
    traverse_to_roi_goal_handle_->publish_feedback(feedback);

    // Create an exploreToPose goal using the target_roi received from mission coordinator
    rs1_interfaces::action::ExploreToPose::Goal explore_to_pose_goal;
    explore_to_pose_goal.target_pose = traverse_to_roi_goal_handle_->get_goal()->target_roi;
    explore_to_pose_goal.target_pose.header.stamp = this->now();

    auto options = explore_to_pose_goal_options_;
    const auto mission_goal = traverse_to_roi_goal_handle_;
    options.feedback_callback = [this, mission_goal, last_stage = std::string{}]
    (
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr,
        const std::shared_ptr<const rs1_interfaces::action::ExploreToPose::Feedback> feedback) mutable
        {
            if (!feedback || !mission_goal ||
                mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_active() || mission_goal->is_canceling() ||
                feedback->stage.empty() || feedback->stage == last_stage)
            {
                return;
            }

            auto mission_feedback =
                std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
            mission_feedback->stage = "Exploring to ROI: " + feedback->stage;
            mission_goal->publish_feedback(mission_feedback);
            last_stage = feedback->stage;
        };

    // Send goal asynchronously to the ExploreToPose server
    try
    {
        action_goal_pending_ = true;
        explore_to_pose_client_->async_send_goal(explore_to_pose_goal, options);
    }
    catch(const std::exception& e)
    {
        action_goal_pending_ = false;
        abortTraversalToROI(std::string("Could not send goal to ExploreToPose: ") + e.what());
    }


}

void ParrotMissionExecutorNode::generateGroundPath()
{
    const auto mission_goal = traverse_to_roi_goal_handle_;

    // Check goal is active and in parrot frame
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) {
        return;
    }

    if (mission_goal->get_goal()->target_roi.header.frame_id != "parrot1_map") {
        abortTraversalToROI("Ground planning requires the ROI in parrot1_map");
        return;
    }

    // Check Parrot actually reached the ROI before requesting Husky's pose
    try
    {
        const auto &target = mission_goal->get_goal()->target_roi;
        const auto transform = tf_buffer_->lookupTransform(
            target.header.frame_id, "parrot1_base_link", tf2::TimePointZero);
        const double pose_age = (this->now() - rclcpp::Time(
            transform.header.stamp, this->get_clock()->get_clock_type())).seconds();
        const double distance = std::hypot(
            transform.transform.translation.x - target.pose.position.x,
            transform.transform.translation.y - target.pose.position.y);
        if (pose_age < 0.0 || pose_age > 2.0 || !std::isfinite(distance) || distance > roi_tolerance_)
        {
            abortTraversalToROI("Navigation reported success but current Parrot pose does not confirm arrival at the ROI");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Parrot has successfully reached the ROI (%.2f m from target)", distance);
    }
    catch (const tf2::TransformException &error)
    {
        abortTraversalToROI(std::string("Could not independently verify Parrot arrival: ") + error.what());
        return;
    }

    // Check the GetCurrentPose service for Husky is ready
    if (!get_current_pose_client_->service_is_ready()) {
        abortTraversalToROI("Husky get_current_pose service is unavailable");
        return;
    }

    parrot_state_ = ParrotState::GENERATING_GROUND_PATH;

    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = "Requesting Husky's current pose";
    mission_goal->publish_feedback(feedback);

    // Request Husky's current pose
    auto request = std::make_shared<rs1_interfaces::srv::GetCurrentPose::Request>();
    request->frame_id = "parrot1_map"; // TODO (james): Parametrise this or don't hardcode this

    // Create the response callback for GetCurrentPose service
    rclcpp::Client<rs1_interfaces::srv::GetCurrentPose>::CallbackType response_callback = std::bind(
        &ParrotMissionExecutorNode::getCurrentPoseResponseCb,
        this,
        mission_goal,
        std::placeholders::_1
    );

    try
    {
        get_current_pose_request_id_ =
            get_current_pose_client_->async_send_request(request, response_callback).request_id;
    }
    catch(const std::exception& e)
    {
        abortTraversalToROI(std::string("Could not request Husky pose: ") + e.what());
    }


    // Request path from Husky's pose to ROI using nav2's ComputePathToPose

    // Send the path to husky as a service (for now, just send it direclty in parrot1_map frame and let husky convert it)



}

void ParrotMissionExecutorNode::sendGroundPath(const nav_msgs::msg::Path &ground_path)
{
    const auto mission_goal = traverse_to_roi_goal_handle_;

    // Check mission is active
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) {
        return;
    }

    // Check SetGroundPath service is available
    if (!set_ground_path_client_->service_is_ready()) {
        abortTraversalToROI("Husky SetGroundPath service is unavailable");
        return;
    }

    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = "Sending ground path to Husky";
    mission_goal->publish_feedback(feedback);

    // Pass through in the request the ground path
    auto request = std::make_shared<rs1_interfaces::srv::SetGroundPath::Request>();
    request->path = ground_path;

    rclcpp::Client<rs1_interfaces::srv::SetGroundPath>::CallbackType response_callback = std::bind(
        &ParrotMissionExecutorNode::setGroundPathResponseCb,
        this,
        mission_goal,
        std::placeholders::_1
    );

    try
    {
        // Finish this handoff before accepting any new mission or cancellation
        ground_path_handoff_pending_ = true;
        set_ground_path_client_->async_send_request(request, response_callback);
    }
    catch(const std::exception& e)
    {
        ground_path_handoff_pending_ = false;
        abortTraversalToROI(std::string("Could not send ground path to Husky: ") + e.what());
    }

}

// Helpers
bool ParrotMissionExecutorNode::validTargetPose(const geometry_msgs::msg::PoseStamped &target_pose) const
{
    // Check the supplied target pose is a valid pose - check frame id, position, orientation values
    if (target_pose.header.frame_id.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Supplied target pose has empty frame id");
        return false;
    }
    if (!std::isfinite(target_pose.pose.position.x) ||
        !std::isfinite(target_pose.pose.position.y) ||
        !std::isfinite(target_pose.pose.position.z))
    {
        RCLCPP_ERROR(this->get_logger(), "Supplied target pose has invalid position");
        return false;
    }

    const double norm_squared =
        target_pose.pose.orientation.x * target_pose.pose.orientation.x +
        target_pose.pose.orientation.y * target_pose.pose.orientation.y +
        target_pose.pose.orientation.z * target_pose.pose.orientation.z +
        target_pose.pose.orientation.w * target_pose.pose.orientation.w;

    if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 1e-3)
    {
        RCLCPP_ERROR(this->get_logger(), "Supplied target pose has invalid orientation");
        return false;
    }

    return true;


}

void ParrotMissionExecutorNode::abortTraversalToROI(const std::string &message)
{
    // Check that the goal handle exists
    if (!traverse_to_roi_goal_handle_)
    {
        return;
    }

    // A rejected child goal can arrive after cancellation was accepted
    if (traverse_to_roi_goal_handle_->is_canceling())
    {
        finishCancelledTraversal();
        return;
    }

    // Send failed result back to whoever requested the mission
    auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
    result->message = message;
    traverse_to_roi_goal_handle_->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());

    // Reset goal handle and parrot state once aborted
    returnToIdle();
}

void ParrotMissionExecutorNode::cancelActionResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response)
{
    // Acknowledging cancellation does not mean the action has stopped yet
    if (!mission_goal || mission_goal != traverse_to_roi_goal_handle_ || !mission_goal->is_canceling()) return;
    if (!response || response->goals_canceling.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Action cancellation was not accepted; waiting for its final result");
    }
}

void ParrotMissionExecutorNode::cancelTraversalToROI()
{
    // Run once ROS has marked the mission CANCELING
    if (!traverse_to_roi_goal_handle_ || !traverse_to_roi_goal_handle_->is_canceling()) return;
    if (cancel_timer_) cancel_timer_->cancel();

    // The goal-response callback will resume cancellation once a handle arrives
    if (action_goal_pending_) return;

    auto response_callback = std::bind(&ParrotMissionExecutorNode::cancelActionResponseCb,
        this, traverse_to_roi_goal_handle_, std::placeholders::_1);

    try
    {
        // Only one of these actions runs at a time
        if (navigate_to_pose_goal_handle_)
        {
            navigate_to_pose_client_->async_cancel_goal(navigate_to_pose_goal_handle_, response_callback);
        }
        else if (explore_to_pose_goal_handle_)
        {
            explore_to_pose_client_->async_cancel_goal(explore_to_pose_goal_handle_, response_callback);
        }
        else if (compute_path_to_pose_goal_handle_)
        {
            compute_path_to_pose_client_->async_cancel_goal(compute_path_to_pose_goal_handle_, response_callback);
        }
        else
        {
            // Waiting for Husky's pose does not require stopping another action
            finishCancelledTraversal();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not cancel current action: %s; waiting for its final result", e.what());
    }
}

void ParrotMissionExecutorNode::finishCancelledTraversal()
{
    // Keep ownership until any running or pending action has finished
    if (!traverse_to_roi_goal_handle_ || !traverse_to_roi_goal_handle_->is_canceling() ||
        action_goal_pending_ || navigate_to_pose_goal_handle_ || explore_to_pose_goal_handle_ ||
        compute_path_to_pose_goal_handle_) return;

    auto result = std::make_shared<rs1_interfaces::action::TraverseToROI::Result>();
    result->message = "Parrot traversal to ROI was canceled";
    traverse_to_roi_goal_handle_->canceled(result);
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    returnToIdle();
}

void ParrotMissionExecutorNode::returnToIdle()
{
    // Discard any outstanding pose request so it cannot restart this mission
    if (get_current_pose_request_id_ >= 0)
    {
        get_current_pose_client_->remove_pending_request(get_current_pose_request_id_);
        get_current_pose_request_id_ = -1;
    }
    if (cancel_timer_) cancel_timer_->cancel();
    cancel_timer_.reset();

    // Reset the goal handles and parrot state to IDLE
    navigate_to_pose_goal_handle_.reset();
    explore_to_pose_goal_handle_.reset();
    compute_path_to_pose_goal_handle_.reset();
    action_goal_pending_ = false;
    ground_path_handoff_pending_ = false;
    traverse_to_roi_goal_handle_.reset();
    parrot_state_ = ParrotState::IDLE;
}

void ParrotMissionExecutorNode::computeGroundPath(const geometry_msgs::msg::PoseStamped &start_pose)
{
    const auto mission_goal = traverse_to_roi_goal_handle_;

    // Check goal is active
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) {
        return;
    }

    // Both endpoints must be valid and expressed in Parrot's map.
    const auto &roi = mission_goal->get_goal()->target_roi;

    // Check the provided husky pose is in the parrot frame and is valid
    if (start_pose.header.frame_id != "parrot1_map" || roi.header.frame_id != "parrot1_map" || !validTargetPose(start_pose) || !validTargetPose(roi))
    {
        abortTraversalToROI("Ground planning requires valid poses in parrot1_map");
        return;
    }

    // Check nav2's ComputePathToPose action server is available
    if (!compute_path_to_pose_client_->action_server_is_ready()) {
        abortTraversalToROI("Parrot ComputePathToPose server is unavailable");
        return;
    }

    auto feedback = std::make_shared<rs1_interfaces::action::TraverseToROI::Feedback>();
    feedback->stage = "Computing ground path";
    mission_goal->publish_feedback(feedback);

    // Create the ComputePathToPose goal using the supplied husky pose
    // This action binds the current mission to ignore responses from old missions
    nav2_msgs::action::ComputePathToPose::Goal goal;
    goal.start = start_pose;
    goal.goal = roi;
    goal.goal.header.stamp = start_pose.header.stamp;
    goal.use_start = true;
    goal.planner_id = "GridBased";

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions options;

    options.goal_response_callback = std::bind(
        &ParrotMissionExecutorNode::computePathToPoseResponseCb,
        this,
        mission_goal,
        std::placeholders::_1);

    options.result_callback = std::bind(
        &ParrotMissionExecutorNode::computePathToPoseResultCb,
        this,
        mission_goal,
        std::placeholders::_1);

        try
        {
            action_goal_pending_ = true;
            compute_path_to_pose_client_->async_send_goal(goal, options);
        }
        catch(const std::exception& e)
        {
            action_goal_pending_ = false;
            abortTraversalToROI(std::string("Could not request ground path: ") + e.what());
        }
}


}  // namespace rs1_mission
