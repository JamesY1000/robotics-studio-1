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

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

    // Sampling (phase 2) parameters
    detection_rate_hz_ = this->declare_parameter<double>("detection_rate_hz", 1.0);
    detection_range_ = this->declare_parameter<double>("detection_range", 20.0);
    camera_hfov_ = this->declare_parameter<double>("camera_hfov", 2.0944);
    swath_overlap_ = this->declare_parameter<double>("swath_overlap", 0.2);
    min_swath_spacing_ = this->declare_parameter<double>("min_swath_spacing", 2.0);
    merge_distance_ = this->declare_parameter<double>("merge_distance", 4.0);
    final_merge_distance_ = this->declare_parameter<double>("final_merge_distance", 5.5);
    roi_tree_margin_ = this->declare_parameter<double>("roi_tree_margin", 0.0);
    min_observations_ = this->declare_parameter<int>("min_observations", 2);
    if (!std::isfinite(detection_rate_hz_) || detection_rate_hz_ <= 0.0 ||
        !std::isfinite(detection_range_) || detection_range_ <= 0.0 ||
        !std::isfinite(camera_hfov_) || camera_hfov_ <= 0.0 || camera_hfov_ >= 3.14159265358979323846 ||
        !std::isfinite(swath_overlap_) || swath_overlap_ < 0.0 || swath_overlap_ >= 1.0 ||
        !std::isfinite(min_swath_spacing_) || min_swath_spacing_ <= 0.0 ||
        !std::isfinite(merge_distance_) || merge_distance_ <= 0.0 ||
        !std::isfinite(final_merge_distance_) || final_merge_distance_ < merge_distance_ ||
        !std::isfinite(roi_tree_margin_) || roi_tree_margin_ < 0.0 ||
        min_observations_ < 0)
    {
        throw std::invalid_argument("Sampling parameters must be finite and within range");
    }

    // Exploration fallback when the Nav2 costmap is incomplete over the ROI
    max_fallback_hops_ = this->declare_parameter<int>("max_fallback_hops", 3);
    // Default off: fly straight to the ROI with the velocity follower instead of
    // waiting on Nav2/explore_lite, which is unreliable below the canopy.
    explore_before_sweep_ = this->declare_parameter<bool>("explore_before_sweep", false);
    if (max_fallback_hops_ < 0)
    {
        throw std::invalid_argument("max_fallback_hops must be non-negative");
    }

    // Sweep geometry + inbound-transit parameters.  Nav2 flies the lanes one
    // NavigateToPose goal per waypoint; the lane-end U-turn is a rounded corner
    // placed in clear air past the ROI edge.
    turn_lead_m_ = this->declare_parameter<double>("turn_lead_m", 3.0);
    turn_offset_m_ = this->declare_parameter<double>("turn_offset_m", 3.0);
    roi_enter_margin_ = this->declare_parameter<double>("roi_enter_margin", 5.0);
    if (!std::isfinite(turn_lead_m_) || turn_lead_m_ < 0.0 ||
        !std::isfinite(turn_offset_m_) || turn_offset_m_ <= 0.0 ||
        !std::isfinite(roi_enter_margin_) || roi_enter_margin_ < 0.0)
    {
        throw std::invalid_argument("Sweep parameters are invalid");
    }

    // Lidar outage watchdog: the sweep stalls silently when the lidar dies.
    scan_outage_timeout_ = this->declare_parameter<double>("scan_outage_timeout", 30.0);
    if (!std::isfinite(scan_outage_timeout_) || scan_outage_timeout_ <= 0.0)
    {
        throw std::invalid_argument("scan_outage_timeout must be finite and positive");
    }

    // Waypoint stall watchdog: Nav2's planner_server owns the global costmap and
    // can block for minutes on a large SLAM-grown costmap, in which case the
    // NavigateToPose goal never returns.  Skip the waypoint instead of hanging.
    waypoint_stall_timeout_ =
        this->declare_parameter<double>("waypoint_stall_timeout", 60.0);
    waypoint_progress_epsilon_ =
        this->declare_parameter<double>("waypoint_progress_epsilon", 1.0);
    if (!std::isfinite(waypoint_stall_timeout_) || waypoint_stall_timeout_ <= 0.0 ||
        !std::isfinite(waypoint_progress_epsilon_) || waypoint_progress_epsilon_ <= 0.0)
    {
        throw std::invalid_argument("waypoint stall parameters must be finite and positive");
    }

    // Create tf buffers and listeners
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // Clients for GetCurrentPose service, ComputePathToPose action and SetGroundPath service
    get_current_pose_client_ = this->create_client<rs1_interfaces::srv::GetCurrentPose>(
        "/husky1/get_current_pose");

    compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        this, "/parrot1/compute_path_to_pose");

    set_ground_path_client_ = this->create_client<rs1_interfaces::srv::SetGroundPath>(
        "/husky1/set_ground_path");

    // Phase 2 clients/publisher for ROI sampling
    detect_client_ = this->create_client<rs1_interfaces::srv::DetectYolo>("/yolo/detect");
    process_client_ = this->create_client<rs1_interfaces::srv::ProcessTreeLocations>(
        "/yolo/process_tree_locations");
    set_clearance_client_ = this->create_client<rs1_interfaces::srv::SetClearance>(
        "/parrot1/set_clearance");

    tree_map_pub_ = this->create_publisher<rs1_interfaces::msg::TreeMap>(
        "/parrot1/tree_map", rclcpp::QoS(1).reliable().transient_local());

    // Lidar outage watchdog: track the freshest scan and abort the sweep if it
    // stops arriving (the GPU lidar or its bridge can die mid-flight).
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/parrot1/scan", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ParrotMissionExecutorNode::scanCb, this, std::placeholders::_1));
    lidar_watchdog_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ParrotMissionExecutorNode::lidarWatchdogCb, this));
    sweep_stall_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ParrotMissionExecutorNode::sweepStallWatchdogCb, this));

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

    // SampleROI server (phase 2)
    sample_roi_server_ = rclcpp_action::create_server<rs1_interfaces::action::SampleROI>(
    this,
    "/parrot1/sample_roi",
    std::bind(&ParrotMissionExecutorNode::sampleROIGoalRequestCb, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ParrotMissionExecutorNode::sampleROICancelRequestCb, this, std::placeholders::_1),
    std::bind(&ParrotMissionExecutorNode::sampleROIGoalAcceptedCb, this, std::placeholders::_1)
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

    // Clear any sampling state
    stopDetection();
    sweep_nav_goal_handle_.reset();
    sample_roi_goal_handle_.reset();
    tree_map_.clear();
    total_waypoints_ = 0;
    waypoints_remaining_ = 0;
    exploring_for_sweep_ = false;
    fallback_hops_ = 0;
    stopSweep();

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


// ---------------------------------------------------------------------------
// Phase 2: ROI sampling (lawnmower sweep + tree detection)
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse ParrotMissionExecutorNode::sampleROIGoalRequestCb(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const rs1_interfaces::action::SampleROI::Goal> goal)
{
    if (parrot_state_ != ParrotState::IDLE)
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting SampleROI goal: Parrot is currently busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal || goal->roi_origin.header.frame_id.empty() ||
        !validROIBox(goal->roi_min, goal->roi_max))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting SampleROI goal: ROI box is invalid");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!navigate_to_pose_client_->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting SampleROI goal: NavigateToPose server unavailable");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!detect_client_->service_is_ready() || !process_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting SampleROI goal: perception services unavailable");
        return rclcpp_action::GoalResponse::REJECT;
    }

    parrot_state_ = ParrotState::TRAVERSE_INSIDE_ROI;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool ParrotMissionExecutorNode::validROIBox(
    const geometry_msgs::msg::Point & min, const geometry_msgs::msg::Point & max) const
{
    return std::isfinite(min.x) && std::isfinite(min.y) &&
           std::isfinite(max.x) && std::isfinite(max.y) &&
           min.x < max.x && min.y < max.y;
}

bool ParrotMissionExecutorNode::prepareSweepBox()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal) return false;
    const auto goal = mission_goal->get_goal();

    // The sweep is planned and controlled in parrot1_odom, NOT parrot1_map.
    // The EKF fuses the exact Gazebo pose, so parrot1_odom == world (numerically),
    // whereas SLAM's parrot1_map -> parrot1_odom diverges badly on this repetitive
    // forest (3 Hz lidar): mid-sweep it reached a ~72 m / 164 m z correction,
    // which made the velocity follower fly the drone physically out of the ROI
    // while its map pose looked fine.
    sweep_frame_ = "parrot1_odom";
    sweep_z_ = goal->roi_origin.pose.position.z;

    const std::string source = goal->roi_origin.header.frame_id;
    if (source == sweep_frame_ || source == "world" || source.empty())
    {
        // "world" is numerically identical to parrot1_odom (the EKF relays the
        // ground-truth pose), so it also needs no transform.
        sweep_min_ = goal->roi_min;
        sweep_max_ = goal->roi_max;
        return validROIBox(sweep_min_, sweep_max_);
    }

    try
    {
        const auto tf = tf_buffer_->lookupTransform(sweep_frame_, source, tf2::TimePointZero);
        // Transform all four corners and re-derive an axis-aligned box, so a
        // yawed source frame still yields a valid sweep rectangle.
        const auto to_map = [&](const geometry_msgs::msg::Point & in)
        {
            geometry_msgs::msg::PoseStamped pin, pout;
            pin.header.frame_id = source;
            pin.pose.position = in;
            pin.pose.orientation.w = 1.0;
            tf2::doTransform(pin, pout, tf);
            return pout.pose.position;
        };
        const auto a = to_map(goal->roi_min);
        const auto b = to_map(goal->roi_max);
        geometry_msgs::msg::Point corner;
        corner.z = 0.0;
        corner.x = goal->roi_min.x; corner.y = goal->roi_max.y;
        const auto c = to_map(corner);
        corner.x = goal->roi_max.x; corner.y = goal->roi_min.y;
        const auto d = to_map(corner);
        sweep_min_.x = std::min({a.x, b.x, c.x, d.x});
        sweep_min_.y = std::min({a.y, b.y, c.y, d.y});
        sweep_min_.z = 0.0;
        sweep_max_.x = std::max({a.x, b.x, c.x, d.x});
        sweep_max_.y = std::max({a.y, b.y, c.y, d.y});
        sweep_max_.z = 0.0;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Could not transform the ROI box from %s into parrot1_map: %s", source.c_str(), e.what());
        return false;
    }

    return validROIBox(sweep_min_, sweep_max_);
}

rclcpp_action::CancelResponse ParrotMissionExecutorNode::sampleROICancelRequestCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> goal_handle)
{
    if (!goal_handle || goal_handle != sample_roi_goal_handle_ || !goal_handle->is_active())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    // ROS marks the goal CANCELING after this callback returns
    cancel_timer_ = this->create_wall_timer(50ms,
        std::bind(&ParrotMissionExecutorNode::cancelSampling, this));
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ParrotMissionExecutorNode::sampleROIGoalAcceptedCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> goal_handle)
{
    sample_roi_goal_handle_ = goal_handle;
    sampleROI();
}

void ParrotMissionExecutorNode::sampleROI()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) return;

    // Normalise the box into the executor's planning frame (parrot1_map) so
    // Nav2 and the explorer both accept it, whatever frame the caller used.
    if (!prepareSweepBox())
    {
        abortSampling("Could not transform the ROI box into parrot1_map");
        return;
    }

    tree_map_.clear();
    total_waypoints_ = 0;
    waypoints_remaining_ = 0;
    exploring_for_sweep_ = false;
    fallback_hops_ = 0;

    // The default run does not teleport, so the drone usually has to fly in from
    // outside the ROI.  Transit at the higher transit clearance (crossing the
    // canopy) and only descend to the search clearance once inside the box,
    // instead of skimming the trees for the whole inbound leg.
    geometry_msgs::msg::PoseStamped here;
    const bool have_here = currentPose(here);
    double outside_x = 0.0;
    double outside_y = 0.0;
    if (have_here)
    {
        outside_x = std::max(0.0, std::max(sweep_min_.x - here.pose.position.x,
                                         here.pose.position.x - sweep_max_.x));
        outside_y = std::max(0.0, std::max(sweep_min_.y - here.pose.position.y,
                                         here.pose.position.y - sweep_max_.y));
    }
    descend_on_entry_ = have_here && std::hypot(outside_x, outside_y) > roi_enter_margin_;
    if (descend_on_entry_)
    {
        RCLCPP_INFO(this->get_logger(),
            "Transiting to the ROI at the transit clearance; the drone will descend to "
            "the search clearance on arrival");
        publishSamplingFeedback("Transiting to ROI");
        restoreTransitClearance();
        beginSweep();
        return;
    }

    publishSamplingFeedback("Setting search clearance");
    setSearchClearance();
}

void ParrotMissionExecutorNode::setSearchClearance()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active()) return;

    if (!set_clearance_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "SetClearance service unavailable; sweeping at the current clearance");
        beginSweep();
        return;
    }

    auto request = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
    request->clearance = mission_goal->get_goal()->search_clearance;
    set_clearance_client_->async_send_request(request,
        std::bind(&ParrotMissionExecutorNode::setSearchClearanceResponseCb, this, std::placeholders::_1));
}

void ParrotMissionExecutorNode::setSearchClearanceResponseCb(
    rclcpp::Client<rs1_interfaces::srv::SetClearance>::SharedFuture future)
{
    try
    {
        const auto response = future.get();
        if (!response->success)
        {
            RCLCPP_WARN(this->get_logger(), "Search clearance not applied: %s", response->message.c_str());
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Search clearance request failed: %s", e.what());
    }

    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active()) return;

    // The Nav2 costmap is usually incomplete around a fresh ROI, so explore
    // toward it first (weighted frontier exploration, as in phase 1) to grow
    // the map before attempting the sweep.
    if (explore_before_sweep_)
    {
        publishSamplingFeedback("Exploring to ROI (incomplete costmap)");
        exploreToSweepTarget();
        return;
    }
    beginSweep();
}

void ParrotMissionExecutorNode::beginSweep()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) return;

    // Arm the lidar watchdog: count from now so a lidar that never starts (or
    // dies right at the start) still trips the abort.
    latest_scan_time_ = this->now();

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    // Start at the ROI corner nearest the drone.  The default run does not
    // teleport, so the drone begins wherever it happens to be and then flies in:
    // picking the near corner turns that into a short transit instead of a
    // complete traverse of the box.
    geometry_msgs::msg::PoseStamped from;
    const bool have_from = currentPose(from);
    const double from_x = have_from ? from.pose.position.x : 0.5 * (sweep_min_.x + sweep_max_.x);
    const double from_y = have_from ? from.pose.position.y : 0.5 * (sweep_min_.y + sweep_max_.y);
    computeSwaths(poses, from_x, from_y);
    if (!poses.empty())
    {
        RCLCPP_INFO(this->get_logger(),
            "Sweeping from the ROI corner nearest (%.1f, %.1f): %.1f m transit to the "
            "first waypoint (%.1f, %.1f)",
            from_x, from_y,
            std::hypot(poses.front().pose.position.x - from_x,
                       poses.front().pose.position.y - from_y),
            poses.front().pose.position.x, poses.front().pose.position.y);
    }
    if (poses.empty())
    {
        abortSampling("Could not generate any sweep waypoints for the ROI");
        return;
    }

    total_waypoints_ = poses.size();
    waypoints_remaining_ = static_cast<int16_t>(std::min<std::size_t>(poses.size(), 32767));
    consecutive_stalls_ = 0;
    resetSweepProgress();

    publishSamplingFeedback("Sweeping ROI");

    // Start continuous detection while the drone flies the sweep
    if (!detection_timer_)
    {
        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / detection_rate_hz_));
        detection_timer_ = this->create_wall_timer(
            std::max(period, std::chrono::milliseconds(50)),
            std::bind(&ParrotMissionExecutorNode::detectionTimerCb, this));
    }

    sweep_waypoints_ = poses;

    // One NavigateToPose per waypoint instead of a single NavigateThroughPoses
    // for the whole box: the costmap is usually incomplete over a fresh ROI,
    // so a whole-box plan fails, but a one-leg plan succeeds and the frontier
    // explorer can fill the map ahead of the drone between legs.
    sweep_index_ = 0;
    navigateToNextWaypoint();
}

void ParrotMissionExecutorNode::computeSwaths(
    std::vector<geometry_msgs::msg::PoseStamped> & poses, double from_x, double from_y) const
{
    poses.clear();
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal) return;

    const auto goal = mission_goal->get_goal();
    const double x_min = sweep_min_.x;
    const double x_max = sweep_max_.x;
    const double y_min = sweep_min_.y;
    const double y_max = sweep_max_.y;
    if (!(x_max > x_min) || !(y_max > y_min)) return;

    constexpr double kPi = 3.14159265358979323846;

    // Auto spacing: the camera looks FORWARD, so a tree is only detected within
    // detection_range of a lane.  Space adjacent lanes by detection_range with
    // overlap so every tree is within range of some lane.  (Previously this used
    // the full 2*R*tan(hfov/2) forward-footprint width -- ~55 m here -- which
    // gave a large ROI only 2 lanes and left most of it unswept.)
    double spacing = goal->swath_spacing;
    if (!(spacing > 0.0))
    {
        spacing = detection_range_ * (1.0 - swath_overlap_);
    }
    spacing = std::max(spacing, min_swath_spacing_);

    const std::string & frame = sweep_frame_;
    const double z = sweep_z_;

    auto make_pose = [&](double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame;
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        poses.push_back(pose);
    };

    // Sweep along the longer axis to minimise the number of turns.
    const bool along_x = (x_max - x_min) >= (y_max - y_min);
    constexpr std::size_t kMaxSwaths = 200;

    // Extend each lane past the ROI edge and add a rounded U-turn waypoint so
    // the turnaround happens in clear air, not while pivoting against the tree
    // canopy that typically grows right up to the ROI boundary.
    const double lead = turn_lead_m_;
    const double bulge = turn_offset_m_;

    // Lane axis = the direction a lane runs in.  Order axis = the direction the
    // lanes are stacked in.  Start from whichever ROI corner is nearest the
    // drone: the default run does not teleport, so the drone flies in from
    // wherever it is and this turns that into the shortest possible transit.
    const double lane_lo = along_x ? x_min : y_min;
    const double lane_hi = along_x ? x_max : y_max;
    const double ord_lo = along_x ? y_min : x_min;
    const double ord_hi = along_x ? y_max : x_max;
    const double from_lane = along_x ? from_x : from_y;
    const double from_ord = along_x ? from_y : from_x;
    // Start at the low end of the lane axis when the drone is nearer that side.
    const bool start_low_lane =
        std::abs(from_lane - lane_lo) < std::abs(from_lane - lane_hi);
    // Visit lanes from the far side first when the drone is nearer that side.
    const bool start_high_ord =
        std::abs(from_ord - ord_hi) < std::abs(from_ord - ord_lo);
    const double yaw_high = along_x ? 0.0 : (kPi * 0.5);  // travelling toward lane_hi
    const double yaw_low = along_x ? kPi : (-kPi * 0.5);  // travelling toward lane_lo

    auto emit = [&](double lane_pos, double ord_pos, double lane_yaw)
    {
        make_pose(along_x ? lane_pos : ord_pos, along_x ? ord_pos : lane_pos, lane_yaw);
    };

    const double span = ord_hi - ord_lo;
    std::size_t swaths = static_cast<std::size_t>(std::ceil(span / spacing));
    swaths = std::clamp<std::size_t>(swaths, 1, kMaxSwaths);
    const double step = span / static_cast<double>(swaths);

    // Centre of the k-th lane in visit order (nearest corner first).
    auto lane_offset = [&](std::size_t k)
    {
        const std::size_t i = start_high_ord ? (swaths - 1 - k) : k;
        return ord_lo + (static_cast<double>(i) + 0.5) * step;
    };

    // Emit both endpoints of lane k; consecutive lanes alternate direction.
    auto emit_lane = [&](std::size_t k)
    {
        const bool toward_high =
            (k % 2 == 0) ? start_low_lane : !start_low_lane;
        const double ord = lane_offset(k);
        if (toward_high)
        {
            emit(lane_lo - lead, ord, yaw_high);
            emit(lane_hi + lead, ord, yaw_high);
        }
        else
        {
            emit(lane_hi + lead, ord, yaw_low);
            emit(lane_lo - lead, ord, yaw_low);
        }
        return toward_high;
    };

    for (std::size_t k = 0; k < swaths; ++k)
    {
        const bool toward_high = emit_lane(k);

        if (k + 1 < swaths)
        {
            // Rounded U-turn: swing out past the ROI edge in clear air before
            // lining up on the next lane.  The drone finishes lane k on the side
            // it was heading toward, so the bulge goes out that side.
            const double ord_mid = 0.5 * (lane_offset(k) + lane_offset(k + 1));
            const double lane_out = toward_high ? (lane_hi + lead + bulge)
                                                : (lane_lo - lead - bulge);
            emit(lane_out, ord_mid, toward_high ? yaw_high : yaw_low);
        }
    }
}

void ParrotMissionExecutorNode::detectionTimerCb()
{
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active()) return;
    if (detection_request_pending_) return;
    if (!detect_client_->service_is_ready()) return;

    auto request = std::make_shared<rs1_interfaces::srv::DetectYolo::Request>();
    request->start_yolo = true;
    detection_request_pending_ = true;
    detect_client_->async_send_request(request,
        std::bind(&ParrotMissionExecutorNode::detectResponseCb, this, std::placeholders::_1));
}

void ParrotMissionExecutorNode::detectResponseCb(
    rclcpp::Client<rs1_interfaces::srv::DetectYolo>::SharedFuture future)
{
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active())
    {
        detection_request_pending_ = false;
        return;
    }

    rs1_interfaces::srv::DetectYolo::Response::SharedPtr response;
    try
    {
        response = future.get();
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Detection failed: %s", e.what());
        detection_request_pending_ = false;
        return;
    }

    if (response->detections.bounding_boxes.empty() || !process_client_->service_is_ready())
    {
        detection_request_pending_ = false;
        return;
    }

    auto request = std::make_shared<rs1_interfaces::srv::ProcessTreeLocations::Request>();
    request->start_lidar = true;
    request->detections = response->detections;
    process_client_->async_send_request(request,
        std::bind(&ParrotMissionExecutorNode::processResponseCb, this, std::placeholders::_1));
}

void ParrotMissionExecutorNode::processResponseCb(
    rclcpp::Client<rs1_interfaces::srv::ProcessTreeLocations>::SharedFuture future)
{
    detection_request_pending_ = false;
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active()) return;

    rs1_interfaces::srv::ProcessTreeLocations::Response::SharedPtr response;
    try
    {
        response = future.get();
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Tree localisation failed: %s", e.what());
        return;
    }

    for (const auto &tree : response->tree_data) mergeTree(tree);
    publishSamplingFeedback("Sweeping ROI");
}

void ParrotMissionExecutorNode::mergeTree(const rs1_interfaces::msg::TreeLocation & tree)
{
    const double tx = tree.location.x;
    const double ty = tree.location.y;

    // Drop trees outside the ROI box.  Both the tree locations and
    // sweep_min_/sweep_max_ are in parrot1_odom.  This discards the inbound
    // transit's detections (the drone crosses the forest to the corner) and
    // neighbours just over the box boundary that the camera/lidar catch
    // mid-lane.
    const double margin = roi_tree_margin_;
    if (tx < sweep_min_.x - margin || tx > sweep_max_.x + margin ||
        ty < sweep_min_.y - margin || ty > sweep_max_.y + margin)
    {
        return;
    }

    const double score = std::isfinite(tree.score) && tree.score > 0.0 ? tree.score : 1.0;

    // Position-based association: merge into the nearest existing tree within the
    // gate REGARDLESS of predicted type, so per-inference YOLO labels of the SAME
    // tree aggregate into one score-weighted type vote (the species decision the
    // user wants).  The centroid is trunk-anchored, so distinct trees are usually
    // farther apart than the gate.
    for (auto & merged : tree_map_)
    {
        if (std::hypot(merged.x - tx, merged.y - ty) <= merge_distance_)
        {
            merged.addObservation(tree.tree_type, tx, ty, tree.location.z, score);
            return;
        }
    }

    MergedTree merged;
    merged.tree_type = tree.tree_type;
    merged.x = tx;
    merged.y = ty;
    merged.z = tree.location.z;
    merged.observations = 1;
    merged.type_votes[tree.tree_type] = score;
    merged.type_x_sum[tree.tree_type] = tx * score;
    merged.type_y_sum[tree.tree_type] = ty * score;
    merged.type_z_sum[tree.tree_type] = tree.location.z * score;
    tree_map_.push_back(merged);
}

void ParrotMissionExecutorNode::publishSamplingFeedback(const std::string & stage)
{
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active()) return;

    auto feedback = std::make_shared<rs1_interfaces::action::SampleROI::Feedback>();
    feedback->stage = stage;
    if (total_waypoints_ > 0)
    {
        const int16_t remaining = std::max<int16_t>(waypoints_remaining_, 0);
        const double done = static_cast<double>(total_waypoints_ - static_cast<std::size_t>(remaining));
        feedback->progress = static_cast<float>(
            std::clamp(done / static_cast<double>(total_waypoints_), 0.0, 1.0));
    }
    else
    {
        feedback->progress = 0.0f;
    }
    feedback->trees_found = static_cast<uint32_t>(tree_map_.size());
    sample_roi_goal_handle_->publish_feedback(feedback);
}

void ParrotMissionExecutorNode::navigateToNextWaypoint()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) return;

    if (sweep_index_ >= sweep_waypoints_.size())
    {
        RCLCPP_INFO(this->get_logger(), "ROI sweep finished with %zu tree(s)", tree_map_.size());
        finishSampling();
        return;
    }

    if (!navigate_to_pose_client_->action_server_is_ready())
    {
        retrySweepWithExploration("NavigateToPose server unavailable");
        return;
    }

    const auto & wp = sweep_waypoints_[sweep_index_];
    nav2_msgs::action::NavigateToPose::Goal goal;

    // Nav2's global costmap lives in parrot1_map, but the sweep waypoint is
    // planned in parrot1_odom, so the goal must be expressed in the map frame
    // before it is sent.  slam_toolbox only republishes parrot1_map ->
    // parrot1_odom when its map updates, so a now-stamped goal handed to Nav2
    // as-is makes the planner look the transform up in the future and abort the
    // leg ("Extrapolation Error ... would require extrapolation into the
    // future"); zero-stamping the goal header does NOT help because Nav2's
    // planner restamps the goal to "now" itself.  Transforming here with the
    // LATEST available map transform (TimePointZero) puts the goal straight into
    // the costmap frame, where Nav2's goal transform is the identity and no
    // map<->odom lookup happens at all.
    geometry_msgs::msg::PoseStamped goal_pose = wp;
    try
    {
        const auto tf = tf_buffer_->lookupTransform(
            "parrot1_map", goal_pose.header.frame_id, tf2::TimePointZero,
            std::chrono::seconds(1));
        tf2::doTransform(goal_pose, goal_pose, tf);
        goal_pose.header.frame_id = "parrot1_map";
    }
    catch (const std::exception & e)
    {
        // Map frame not available yet (SLAM still initialising).  The EKF pins
        // parrot1_odom == world and the map starts at identity, so relabel the
        // odom pose.  Accuracy does not matter while the map is settling.
        RCLCPP_WARN(this->get_logger(),
            "Could not transform the sweep waypoint into parrot1_map (%s); using the odom pose",
            e.what());
        goal_pose.header.frame_id = "parrot1_map";
    }
    goal.pose = goal_pose;

    auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = std::bind(
        &ParrotMissionExecutorNode::sweepWaypointResponseCb, this, std::placeholders::_1);
    options.result_callback = std::bind(
        &ParrotMissionExecutorNode::sweepWaypointResultCb, this, std::placeholders::_1);
    options.feedback_callback = [this, mission_goal](
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>)
        {
            maybeDescendOnEntry();
            publishSamplingFeedback("Sweeping ROI");
        };

    RCLCPP_INFO(this->get_logger(),
        "Sweep waypoint %zu/%zu (%.1f, %.1f)",
        sweep_index_ + 1, sweep_waypoints_.size(),
        wp.pose.position.x, wp.pose.position.y);

    resetSweepProgress();
    try
    {
        action_goal_pending_ = true;
        navigate_to_pose_client_->async_send_goal(goal, options);
    }
    catch (const std::exception &e)
    {
        action_goal_pending_ = false;
        abortSampling(std::string("Could not send the sweep waypoint goal: ") + e.what());
    }
}

void ParrotMissionExecutorNode::resetSweepProgress()
{
    sweep_progress_anchor_valid_ = false;
    last_sweep_progress_time_ = this->now();
}

void ParrotMissionExecutorNode::sweepWaypointResponseCb(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    action_goal_pending_ = false;
    if (!goal_handle)
    {
        retrySweepWithExploration("Nav2 rejected the sweep waypoint goal");
        return;
    }

    sweep_nav_goal_handle_ = goal_handle;
    if (sample_roi_goal_handle_ && sample_roi_goal_handle_->is_canceling())
    {
        cancelSampling();
    }
}

void ParrotMissionExecutorNode::sweepWaypointResultCb(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
    if (!sample_roi_goal_handle_ || !sweep_nav_goal_handle_ ||
        result.goal_id != sweep_nav_goal_handle_->get_goal_id()) return;

    sweep_nav_goal_handle_.reset();
    if (sample_roi_goal_handle_->is_canceling())
    {
        finishCancelledSampling();
        return;
    }

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            ++sweep_index_;
            fallback_hops_ = 0;
            consecutive_stalls_ = 0;
            waypoints_remaining_ = static_cast<int16_t>(
                std::min<std::size_t>(sweep_waypoints_.size() - sweep_index_, 32767));
            publishSamplingFeedback("Sweeping ROI");
            maybeDescendOnEntry();
            navigateToNextWaypoint();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            retrySweepWithExploration("sweep waypoint aborted by Nav2");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            abortSampling("ROI sweep was canceled");
            break;

        default:
            retrySweepWithExploration("unexpected result code");
            break;
    }
}

void ParrotMissionExecutorNode::finishSampling()
{
    stopSweep();
    stopDetection();
    parrot_state_ = ParrotState::GENERATE_TREE_MAP;
    waypoints_remaining_ = 0;
    publishSamplingFeedback("Publishing tree map");
    const std::size_t published = publishTreeMap();
    restoreTransitClearance();

    auto result = std::make_shared<rs1_interfaces::action::SampleROI::Result>();
    result->tree_count = static_cast<uint32_t>(published);
    result->message = "ROI sweep complete; tree map published for Husky";
    if (sample_roi_goal_handle_) sample_roi_goal_handle_->succeed(result);
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    returnToIdle();
}

std::size_t ParrotMissionExecutorNode::publishTreeMap()
{
    // Final dedup: the incremental merge gate leaves a few entries when the
    // same tree's centroid wanders farther than merge_distance_ (wide-canopy
    // oaks detected from both sides).  Absorb remaining near pairs once,
    // most-observed entry first, so each tree is published once.
    std::vector<MergedTree> finalized;
    {
        std::vector<MergedTree> sorted = tree_map_;
        std::sort(sorted.begin(), sorted.end(),
            [](const MergedTree & a, const MergedTree & b) { return a.observations > b.observations; });
        for (const auto & merged : sorted)
        {
            MergedTree * target = nullptr;
            double best = final_merge_distance_;
            for (auto & kept : finalized)
            {
                const double d = std::hypot(kept.x - merged.x, kept.y - merged.y);
                if (d <= best)
                {
                    best = d;
                    target = &kept;
                }
            }
            if (target != nullptr)
            {
                target->absorb(merged);
            }
            else
            {
                finalized.push_back(merged);
            }
        }
    }

    rs1_interfaces::msg::TreeMap msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "parrot1_odom";
    std::size_t dropped = 0;
    msg.trees.reserve(finalized.size());
    for (const auto & merged : finalized)
    {
        // Single-shot detections are usually bad anchors or false positives:
        // require a minimum number of sightings before trusting a tree.
        if (merged.observations < static_cast<uint32_t>(min_observations_))
        {
            ++dropped;
            continue;
        }
        rs1_interfaces::msg::TreeLocation tree;
        tree.tree_type = merged.tree_type;
        tree.location.x = merged.x;
        tree.location.y = merged.y;
        tree.location.z = merged.z;
        tree.observations = merged.observations;
        msg.trees.push_back(tree);
    }

    if (tree_map_pub_) tree_map_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
        "Published tree map with %zu tree(s) (%zu dropped below %d observation(s))",
        msg.trees.size(), dropped, min_observations_);
    return msg.trees.size();
}

void ParrotMissionExecutorNode::stopDetection()
{
    if (detection_timer_)
    {
        detection_timer_->cancel();
        detection_timer_.reset();
    }
    detection_request_pending_ = false;
}

void ParrotMissionExecutorNode::restoreTransitClearance()
{
    if (!sample_roi_goal_handle_ || !set_clearance_client_ || !set_clearance_client_->service_is_ready())
    {
        return;
    }

    auto request = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
    request->clearance = sample_roi_goal_handle_->get_goal()->transit_clearance;
    set_clearance_client_->async_send_request(request);
}

void ParrotMissionExecutorNode::maybeDescendOnEntry()
{
    // The "descend to the search clearance once inside the ROI" transition is
    // driven from the sweep waypoint feedback: within roi_enter_margin_ of the
    // box counts as entered.  Sends the request directly (NOT via
    // setSearchClearance(), which would re-plan the sweep from the start).
    if (!descend_on_entry_ || !sample_roi_goal_handle_ ||
        !sample_roi_goal_handle_->is_active())
    {
        return;
    }

    geometry_msgs::msg::PoseStamped here;
    if (!currentPose(here)) return;

    const double escape_x = std::max(
        0.0, std::max(sweep_min_.x - here.pose.position.x,
                      here.pose.position.x - sweep_max_.x));
    const double escape_y = std::max(
        0.0, std::max(sweep_min_.y - here.pose.position.y,
                      here.pose.position.y - sweep_max_.y));
    if (std::hypot(escape_x, escape_y) > roi_enter_margin_) return;

    descend_on_entry_ = false;
    RCLCPP_INFO(this->get_logger(), "Entered the ROI; starting the sweep lanes");

    if (set_clearance_client_ && set_clearance_client_->service_is_ready())
    {
        auto request = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
        request->clearance = sample_roi_goal_handle_->get_goal()->search_clearance;
        set_clearance_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Descending to the search clearance for the sweep");
    }
}

void ParrotMissionExecutorNode::abortSampling(const std::string & message)
{
    if (!sample_roi_goal_handle_) return;
    if (sample_roi_goal_handle_->is_canceling())
    {
        finishCancelledSampling();
        return;
    }

    stopSweep();
    stopDetection();
    restoreTransitClearance();

    // Publish whatever was found so the Husky still receives partial data.
    if (!tree_map_.empty()) publishTreeMap();

    auto result = std::make_shared<rs1_interfaces::action::SampleROI::Result>();
    result->message = message;
    result->tree_count = static_cast<uint32_t>(tree_map_.size());
    sample_roi_goal_handle_->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    returnToIdle();
}

void ParrotMissionExecutorNode::cancelSampling()
{
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_canceling()) return;
    if (cancel_timer_) cancel_timer_->cancel();

    // The goal-response callback will resume cancellation once a handle arrives
    if (action_goal_pending_) return;

    try
    {
        if (sweep_nav_goal_handle_)
        {
            navigate_to_pose_client_->async_cancel_goal(
                sweep_nav_goal_handle_,
                std::bind(&ParrotMissionExecutorNode::sampleROICancelResponseCb,
                    this, sample_roi_goal_handle_, std::placeholders::_1));
        }
        else if (explore_to_pose_goal_handle_)
        {
            explore_to_pose_client_->async_cancel_goal(
                explore_to_pose_goal_handle_,
                std::bind(&ParrotMissionExecutorNode::sampleROIExploreCancelResponseCb,
                    this, sample_roi_goal_handle_, std::placeholders::_1));
        }
        else
        {
            stopSweep();
            finishCancelledSampling();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Could not cancel the ROI sweep: %s; waiting for its final result", e.what());
    }
}

void ParrotMissionExecutorNode::finishCancelledSampling()
{
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_canceling() ||
        action_goal_pending_ || sweep_nav_goal_handle_ ||
        explore_to_pose_goal_handle_) return;

    stopSweep();
    stopDetection();
    restoreTransitClearance();

    auto result = std::make_shared<rs1_interfaces::action::SampleROI::Result>();
    result->message = "Parrot ROI sampling was canceled";
    result->tree_count = static_cast<uint32_t>(tree_map_.size());
    sample_roi_goal_handle_->canceled(result);
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    returnToIdle();
}

void ParrotMissionExecutorNode::sampleROICancelResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> mission_goal,
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response)
{
    if (!mission_goal || mission_goal != sample_roi_goal_handle_ || !mission_goal->is_canceling()) return;
    if (!response || response->goals_canceling.empty())
    {
        RCLCPP_WARN(this->get_logger(), "ROI sweep cancellation was not accepted; waiting for its final result");
    }
}

void ParrotMissionExecutorNode::exploreToSweepTarget()
{
    const auto mission_goal = sample_roi_goal_handle_;
    if (!mission_goal || !mission_goal->is_active() || mission_goal->is_canceling()) return;

    if (!explore_to_pose_client_->action_server_is_ready())
    {
        // Mid-sweep: keep the lane plan and carry on from the current waypoint.
        // beginSweep() recomputes the swaths from the nearest corner, which
        // re-flies every lane already covered (observed on the 80x100 m ROI: it
        // threw away 8/14 waypoints of progress).
        if (sweep_index_ < sweep_waypoints_.size())
        {
            RCLCPP_WARN(this->get_logger(),
                "ExploreToPose server unavailable; resuming the sweep at waypoint %zu/%zu "
                "instead of re-planning the whole box",
                sweep_index_ + 1, sweep_waypoints_.size());
            navigateToNextWaypoint();
            return;
        }
        RCLCPP_WARN(this->get_logger(),
            "ExploreToPose server unavailable; attempting the sweep without exploration");
        beginSweep();
        return;
    }

    // Drive the weighted frontier explorer toward the centre of the ROI. This
    // is the same exploration used in phase 1; it grows the costmap so the
    // sweep can be planned.
    //
    // explore_lite REJECTS any goal whose frame is not its costmap global frame
    // ("parrot1_map"): it checks `goal.target_pose.header.frame_id ==
    // costmap_client_.getGlobalFrameID()`.  The sweep is planned in
    // parrot1_odom, so the ROI centre must be transformed into the map frame
    // first - sending it in parrot1_odom made every exploration goal bounce
    // with "target frame must be 'parrot1_map'".
    geometry_msgs::msg::PoseStamped roi_centre_odom;
    roi_centre_odom.header.frame_id = sweep_frame_;
    roi_centre_odom.header.stamp = this->now();
    roi_centre_odom.pose.orientation.w = 1.0;
    if (sweep_index_ < sweep_waypoints_.size())
    {
        // Mid-sweep: steer the frontier explorer toward the NEXT waypoint we
        // still need to reach, so the costmap grows along the lane we are about
        // to fly rather than somewhere in the middle of the ROI.
        roi_centre_odom.pose.position = sweep_waypoints_[sweep_index_].pose.position;
    }
    else
    {
        roi_centre_odom.pose.position.x = 0.5 * (sweep_min_.x + sweep_max_.x);
        roi_centre_odom.pose.position.y = 0.5 * (sweep_min_.y + sweep_max_.y);
        roi_centre_odom.pose.position.z = sweep_z_;
    }

    const std::string explore_frame = "parrot1_map";
    geometry_msgs::msg::PoseStamped explore_target;
    try
    {
        const auto tf = tf_buffer_->lookupTransform(
            explore_frame, sweep_frame_, tf2::TimePointZero,
            std::chrono::seconds(1));
        tf2::doTransform(roi_centre_odom, explore_target, tf);
        explore_target.header.frame_id = explore_frame;
    }
    catch (const std::exception & e)
    {
        // No map frame yet (e.g. SLAM still initialising): the EKF pins
        // parrot1_odom == world, so fall back to the odom pose but relabel it
        // so explore_lite accepts it.  Position accuracy is not critical here -
        // this goal only steers frontier expansion toward the ROI.
        RCLCPP_WARN(this->get_logger(),
            "Could not transform the exploration target into %s (%s); using the odom pose",
            explore_frame.c_str(), e.what());
        explore_target = roi_centre_odom;
        explore_target.header.frame_id = explore_frame;
    }

    rs1_interfaces::action::ExploreToPose::Goal explore_goal;
    explore_goal.target_pose = explore_target;

    auto options = rclcpp_action::Client<rs1_interfaces::action::ExploreToPose>::SendGoalOptions();
    options.goal_response_callback = std::bind(
        &ParrotMissionExecutorNode::exploreForSweepResponseCb, this, std::placeholders::_1);
    options.result_callback = std::bind(
        &ParrotMissionExecutorNode::exploreForSweepResultCb, this, std::placeholders::_1);
    options.feedback_callback = [this, mission_goal](
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr,
        const std::shared_ptr<const rs1_interfaces::action::ExploreToPose::Feedback> feedback)
        {
            if (!feedback || !mission_goal || mission_goal != sample_roi_goal_handle_ ||
                !mission_goal->is_active() || mission_goal->is_canceling()) return;
            publishSamplingFeedback("Exploring to ROI: " + feedback->stage);
        };

    RCLCPP_INFO(this->get_logger(),
        "Exploring toward the ROI (%.1f, %.1f) to extend the costmap before sweeping",
        explore_goal.target_pose.pose.position.x, explore_goal.target_pose.pose.position.y);

    exploring_for_sweep_ = true;
    try
    {
        action_goal_pending_ = true;
        explore_to_pose_client_->async_send_goal(explore_goal, options);
    }
    catch (const std::exception &e)
    {
        action_goal_pending_ = false;
        exploring_for_sweep_ = false;
        abortSampling(std::string("Could not send the ROI exploration goal: ") + e.what());
    }
}

void ParrotMissionExecutorNode::exploreForSweepResponseCb(
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr goal_handle)
{
    action_goal_pending_ = false;
    if (!goal_handle)
    {
        exploring_for_sweep_ = false;
        abortSampling("ExploreToPose rejected the ROI exploration goal");
        return;
    }

    explore_to_pose_goal_handle_ = goal_handle;
    if (sample_roi_goal_handle_ && sample_roi_goal_handle_->is_canceling())
    {
        cancelSampling();
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "ROI exploration goal accepted");
}

void ParrotMissionExecutorNode::exploreForSweepResultCb(
    const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::WrappedResult & result)
{
    if (!sample_roi_goal_handle_ || !explore_to_pose_goal_handle_ ||
        result.goal_id != explore_to_pose_goal_handle_->get_goal_id()) return;

    explore_to_pose_goal_handle_.reset();
    exploring_for_sweep_ = false;

    if (sample_roi_goal_handle_->is_canceling())
    {
        finishCancelledSampling();
        return;
    }

    const std::string detail = result.result ? result.result->message : std::string{};
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (sweep_index_ < sweep_waypoints_.size())
            {
                // Mid-sweep resume: keep the lane plan already computed and carry
                // on from the waypoint we were heading for, rather than re-planning
                // the whole box (which would restart at the nearest corner and
                // re-fly lanes we already covered).
                RCLCPP_INFO(this->get_logger(), "Exploration reached the waypoint; resuming the sweep");
                navigateToNextWaypoint();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Exploration reached the ROI; starting the sweep");
                beginSweep();
            }
            break;

        case rclcpp_action::ResultCode::ABORTED:
            retrySweepWithExploration(detail.empty() ? "exploration failed" : detail);
            break;

        case rclcpp_action::ResultCode::CANCELED:
            finishCancelledSampling();
            break;

        default:
            retrySweepWithExploration("exploration returned an unexpected result code");
            break;
    }
}

void ParrotMissionExecutorNode::retrySweepWithExploration(const std::string & reason)
{
    if (sample_roi_goal_handle_ && sample_roi_goal_handle_->is_canceling())
    {
        finishCancelledSampling();
        return;
    }

    if (fallback_hops_ >= max_fallback_hops_)
    {
        abortSampling("ROI sweep failed and exploration retries are exhausted: " + reason);
        return;
    }

    ++fallback_hops_;
    RCLCPP_WARN(this->get_logger(),
        "ROI sweep leg failed (%s); falling back to exploration toward the ROI (%d/%d)",
        reason.c_str(), fallback_hops_, max_fallback_hops_);
    publishSamplingFeedback("Exploring to ROI (incomplete costmap)");
    exploreToSweepTarget();
}

void ParrotMissionExecutorNode::sampleROIExploreCancelResponseCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> mission_goal,
    rclcpp_action::Client<rs1_interfaces::action::ExploreToPose>::CancelResponse::SharedPtr response)
{
    if (!mission_goal || mission_goal != sample_roi_goal_handle_ || !mission_goal->is_canceling()) return;
    if (!response || response->goals_canceling.empty())
    {
        RCLCPP_WARN(this->get_logger(), "ROI exploration cancellation was not accepted; waiting for its final result");
    }
}

bool ParrotMissionExecutorNode::currentPose(geometry_msgs::msg::PoseStamped & pose) const
{
    try
    {
        const auto tf = tf_buffer_->lookupTransform(
            "parrot1_odom", "parrot1_base_link", tf2::TimePointZero);
        pose.header = tf.header;
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

void ParrotMissionExecutorNode::scanCb(const sensor_msgs::msg::LaserScan &)
{
    latest_scan_time_ = this->now();
}

void ParrotMissionExecutorNode::lidarWatchdogCb()
{
    // Only guard the sampling sweep (phase 2).  Phase 1 navigation and idle are
    // not lidar-critical.
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active() ||
        sample_roi_goal_handle_->is_canceling())
    {
        return;
    }

    // No scan received since the sweep started yet: count from the sweep start.
    if (latest_scan_time_.nanoseconds() == 0)
    {
        return;
    }

    const double age = (this->now() - latest_scan_time_).seconds();
    if (age >= scan_outage_timeout_)
    {
        abortSampling(
            "Sweep aborted: lidar scan stopped for " +
            std::to_string(static_cast<int>(age)) +
            " s. The GPU lidar (or its ros_gz bridge) has died; tree anchoring "
            "and Nav2 obstacle avoidance are impossible.");
    }
}


void ParrotMissionExecutorNode::sweepStallWatchdogCb()
{
    // Only guard the per-waypoint Nav2 sweep while a waypoint goal is in flight.
    // The exploration fallback has its own lifecycle and must not be cancelled
    // from here.
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active() ||
        sample_roi_goal_handle_->is_canceling())
    {
        return;
    }
    if (exploring_for_sweep_ || !sweep_nav_goal_handle_) return;
    if (last_sweep_progress_time_.nanoseconds() == 0) return;

    // Progress is measured from the drone's own pose, which is independent of
    // whatever Nav2 reports.  Nav2's feedback is useless here: bt_navigator
    // keeps ticking and publishing it while the planner is blocked, and
    // distance_remaining is filled in by the BT, so it can be absent entirely.
    geometry_msgs::msg::PoseStamped pose;
    if (!currentPose(pose)) return;

    if (!sweep_progress_anchor_valid_)
    {
        // First tick for this waypoint: start the clock.
        sweep_progress_anchor_ = pose.pose.position;
        sweep_progress_anchor_valid_ = true;
        last_sweep_progress_time_ = this->now();
        return;
    }

    const double moved = std::hypot(
        pose.pose.position.x - sweep_progress_anchor_.x,
        pose.pose.position.y - sweep_progress_anchor_.y);
    if (moved >= waypoint_progress_epsilon_)
    {
        sweep_progress_anchor_ = pose.pose.position;
        last_sweep_progress_time_ = this->now();
        return;
    }

    const double idle = (this->now() - last_sweep_progress_time_).seconds();
    if (idle < waypoint_stall_timeout_) return;

    ++consecutive_stalls_;
    RCLCPP_WARN(this->get_logger(),
        "No progress toward waypoint %zu/%zu for %.0f s (the Nav2 planner or its "
        "global costmap is wedged); skipping it (stall %d/%d)",
        sweep_index_ + 1, sweep_waypoints_.size(), idle,
        consecutive_stalls_, max_consecutive_stalls_);

    // Cancel whatever is in flight before moving on.  This is non-blocking: a
    // wedged planner_server will not answer the cancel, which is exactly why the
    // waypoint is being abandoned.
    stopSweep();

    if (consecutive_stalls_ >= max_consecutive_stalls_)
    {
        abortSampling(
            "Sweep aborted: Nav2 stopped making progress on " +
            std::to_string(consecutive_stalls_) + " consecutive waypoints (planner "
            "server / global costmap wedged). The partial tree map covers only the "
            "lanes flown so far.");
        return;
    }

    ++sweep_index_;
    fallback_hops_ = 0;
    waypoints_remaining_ = static_cast<int16_t>(
        std::min<std::size_t>(sweep_waypoints_.size() - sweep_index_, 32767));
    publishSamplingFeedback("Sweeping ROI");
    navigateToNextWaypoint();
}


void ParrotMissionExecutorNode::stopSweep()
{
    if (sweep_nav_goal_handle_)
    {
        navigate_to_pose_client_->async_cancel_goal(sweep_nav_goal_handle_);
        sweep_nav_goal_handle_.reset();
    }
    if (explore_to_pose_goal_handle_)
    {
        explore_to_pose_client_->async_cancel_goal(explore_to_pose_goal_handle_);
        explore_to_pose_goal_handle_.reset();
    }
    exploring_for_sweep_ = false;
}


}  // namespace rs1_mission
