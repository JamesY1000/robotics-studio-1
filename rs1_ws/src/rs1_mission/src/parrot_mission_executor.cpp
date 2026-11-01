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
    start_double_pass_ = this->declare_parameter<bool>("start_double_pass", true);
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

    // Sweep follower: "nav2" flies the boustrophedon through NavigateThroughPoses,
    // "velocity" drives the lanes directly with the internal velocity controller.
    // "velocity" only works with a terrain-following build that follows terrain
    // without an executing nav2 goal; the hover-mode follower (HOVER drops every
    // cmd_vel) leaves it stalled, so it is kept as a legacy opt-out only.
    sweep_controller_ = this->declare_parameter<std::string>("sweep_controller", "nav2");
    if (sweep_controller_ != "nav2")
    {
        RCLCPP_WARN(
            this->get_logger(),
            "sweep_controller='%s': the velocity follower is legacy and will not move "
            "while terrain following is in hover mode. Use 'nav2'.",
            sweep_controller_.c_str());
    }
    sweep_control_hz_ = this->declare_parameter<double>("sweep_control_hz", 10.0);
    max_sweep_speed_ = this->declare_parameter<double>("max_sweep_speed", 3.0);
    sweep_speed_gain_ = this->declare_parameter<double>("sweep_speed_gain", 1.2);
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 1.8);
    sweep_yaw_gain_ = this->declare_parameter<double>("sweep_yaw_gain", 2.0);
    heading_gate_ = this->declare_parameter<double>("heading_gate", 0.7);
    waypoint_tolerance_ = this->declare_parameter<double>("waypoint_tolerance", 1.2);
    waypoint_skip_distance_ = this->declare_parameter<double>("waypoint_skip_distance", 2.5);
    waypoint_skip_time_ = this->declare_parameter<double>("waypoint_skip_time", 4.0);
    skip_ticks_threshold_ = static_cast<int>(std::ceil(waypoint_skip_time_ * sweep_control_hz_));
    // Must be short: at 3 m/s a wedge is obvious within a second or two, and the
    // old 12 s let the drone hover in the branches long enough that it "gave up"
    // on the lane before ever trying a recovery.
    no_progress_timeout_ = this->declare_parameter<double>("no_progress_timeout", 6.0);
    no_progress_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, no_progress_timeout_ * sweep_control_hz_)));
    max_consecutive_skips_ = this->declare_parameter<int>("max_consecutive_skips", 3);
    // How many rungs of the duck/pop-up ladder to try before giving the leg up.
    max_recovery_attempts_ = this->declare_parameter<int>("max_recovery_attempts", 4);
    hard_wedge_timeout_ = this->declare_parameter<double>("hard_wedge_timeout", 5.0);
    hard_wedge_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, hard_wedge_timeout_ * sweep_control_hz_)));
    popup_clearance_ = this->declare_parameter<double>("popup_clearance", 8.0);
    popup_step_m_ = this->declare_parameter<double>("popup_step", 2.0);
    popup_max_clearance_ = this->declare_parameter<double>("popup_max_clearance", 12.0);
    duck_clearance_ = this->declare_parameter<double>("duck_clearance", 0.8);
    // Short grace windows: the altitude change itself now takes ~1-2 s
    // (max_vertical_speed 2 m/s), so a duck that has not helped within 5 s is
    // not going to help, and waiting longer only leaves the drone parked in the
    // branches (and long enough for the watchdog to dump the lane).
    duck_grace_time_ = this->declare_parameter<double>("duck_grace_time", 5.0);
    duck_grace_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, duck_grace_time_ * sweep_control_hz_)));
    popup_grace_time_ = this->declare_parameter<double>("popup_grace_time", 10.0);
    popup_grace_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, popup_grace_time_ * sweep_control_hz_)));
    alt_recover_travel_m_ = this->declare_parameter<double>("alt_recover_travel", 2.5);
    roi_escape_margin_ = this->declare_parameter<double>("roi_escape_margin", 20.0);
    roi_escape_timeout_ = this->declare_parameter<double>("roi_escape_timeout", 8.0);
    roi_escape_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, roi_escape_timeout_ * sweep_control_hz_)));
    roi_enter_margin_ = this->declare_parameter<double>("roi_enter_margin", 5.0);
    roi_transit_timeout_ = this->declare_parameter<double>("roi_transit_timeout", 120.0);
    roi_transit_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, roi_transit_timeout_ * sweep_control_hz_)));
    forward_cone_ = this->declare_parameter<double>("forward_cone", 0.6);
    min_gap_width_ = this->declare_parameter<double>("min_gap_width", 0.25);
    scan_stale_timeout_ = this->declare_parameter<double>("scan_stale_timeout", 2.0);
    scan_outage_timeout_ = this->declare_parameter<double>("scan_outage_timeout", 30.0);
    scan_outage_warn_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, 5.0 * sweep_control_hz_)));
    scan_outage_ticks_threshold_ = static_cast<int>(
        std::ceil(std::max(1.0, scan_outage_timeout_ * sweep_control_hz_)));
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 5.0);
    // Avoidance geometry is scaled for ~3 m/s AND a 3 Hz lidar (the scan is up
    // to 1 m stale between frames).  The old 1.2 m stop distance gave 0.4 s of
    // reaction time at 1 m/s - at 3 m/s it is sub-frame, so everything here is
    // pushed out until there is at least one full scan of margin.
    obstacle_stop_distance_ = this->declare_parameter<double>("obstacle_stop_distance", 2.5);
    avoid_distance_ = this->declare_parameter<double>("avoid_distance", 5.0);
    avoid_offset_m_ = this->declare_parameter<double>("avoid_offset", 3.5);
    avoid_rate_ = this->declare_parameter<double>("avoid_rate", 1.2);
    avoid_disengage_margin_ = this->declare_parameter<double>("avoid_disengage_margin", 1.5);
    // Floor on the clearance-proportional slowdown near an obstacle (1.0 = no
    // slowdown, 0.5 = half speed at point-blank range).
    avoid_min_slowdown_ = this->declare_parameter<double>("avoid_min_slowdown", 0.5);
    side_cone_ = this->declare_parameter<double>("side_cone", 1.2);
    steering_cone_ = this->declare_parameter<double>("steering_cone", 0.6);
    turn_lead_m_ = this->declare_parameter<double>("turn_lead_m", 3.0);
    turn_offset_m_ = this->declare_parameter<double>("turn_offset_m", 3.0);
    if (!std::isfinite(sweep_control_hz_) || sweep_control_hz_ <= 0.0 ||
        !std::isfinite(max_sweep_speed_) || max_sweep_speed_ <= 0.0 ||
        !std::isfinite(sweep_speed_gain_) || sweep_speed_gain_ <= 0.0 ||
        !std::isfinite(max_yaw_rate_) || max_yaw_rate_ <= 0.0 ||
        !std::isfinite(sweep_yaw_gain_) || sweep_yaw_gain_ <= 0.0 ||
        !std::isfinite(heading_gate_) || heading_gate_ <= 0.0 || heading_gate_ > 3.14159265358979323846 ||
        !std::isfinite(waypoint_tolerance_) || waypoint_tolerance_ <= 0.0 ||
        !std::isfinite(waypoint_skip_distance_) || waypoint_skip_distance_ <= 0.0 ||
        !std::isfinite(waypoint_skip_time_) || waypoint_skip_time_ < 0.0 ||
        !std::isfinite(forward_cone_) || forward_cone_ <= 0.0 ||
        !std::isfinite(obstacle_stop_distance_) || obstacle_stop_distance_ < 0.0 ||
        !std::isfinite(min_gap_width_) || min_gap_width_ < 0.0 ||
        !std::isfinite(scan_stale_timeout_) || scan_stale_timeout_ <= 0.0 ||
        !std::isfinite(scan_outage_timeout_) || scan_outage_timeout_ <= 0.0 ||
        !std::isfinite(lookahead_distance_) || lookahead_distance_ <= 0.0 ||
        !std::isfinite(avoid_distance_) || avoid_distance_ <= 0.0 ||
        !std::isfinite(avoid_offset_m_) || avoid_offset_m_ <= 0.0 ||
        !std::isfinite(avoid_rate_) || avoid_rate_ <= 0.0 ||
        !std::isfinite(avoid_disengage_margin_) || avoid_disengage_margin_ < 0.0 ||
        !std::isfinite(avoid_min_slowdown_) || avoid_min_slowdown_ <= 0.0 ||
        avoid_min_slowdown_ > 1.0 ||
        !std::isfinite(side_cone_) || side_cone_ <= forward_cone_ ||
        !std::isfinite(steering_cone_) || steering_cone_ <= 0.0 ||
        !std::isfinite(turn_lead_m_) || turn_lead_m_ < 0.0 ||
        !std::isfinite(turn_offset_m_) || turn_offset_m_ <= 0.0 ||
        max_consecutive_skips_ < 1 ||
        max_recovery_attempts_ < 1 ||
        !std::isfinite(duck_grace_time_) || duck_grace_time_ <= 0.0 ||
        !std::isfinite(popup_grace_time_) || popup_grace_time_ <= 0.0 ||
        !std::isfinite(hard_wedge_timeout_) || hard_wedge_timeout_ <= 0.0 ||
        !std::isfinite(popup_clearance_) || popup_clearance_ <= 0.0 ||
        !std::isfinite(popup_step_m_) || popup_step_m_ < 0.0 ||
        !std::isfinite(popup_max_clearance_) || popup_max_clearance_ < popup_clearance_ ||
        !std::isfinite(duck_clearance_) || duck_clearance_ <= 0.0 ||
        !std::isfinite(alt_recover_travel_m_) || alt_recover_travel_m_ <= 0.0)
    {
        throw std::invalid_argument("Sweep follower parameters are invalid");
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

    // Phase 2 clients/publisher for ROI sampling
    navigate_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this, "/parrot1/navigate_through_poses");

    navigate_through_poses_goal_options_.goal_response_callback = std::bind(
        &ParrotMissionExecutorNode::navigateThroughPosesResponseCb, this, std::placeholders::_1);
    navigate_through_poses_goal_options_.result_callback = std::bind(
        &ParrotMissionExecutorNode::navigateThroughPosesResultCb, this, std::placeholders::_1);
    navigate_through_poses_goal_options_.feedback_callback = std::bind(
        &ParrotMissionExecutorNode::navigateThroughPosesFeedbackCb, this,
        std::placeholders::_1, std::placeholders::_2);

    detect_client_ = this->create_client<rs1_interfaces::srv::DetectYolo>("/yolo/detect");
    process_client_ = this->create_client<rs1_interfaces::srv::ProcessTreeLocations>(
        "/yolo/process_tree_locations");
    set_clearance_client_ = this->create_client<rs1_interfaces::srv::SetClearance>(
        "/parrot1/set_clearance");

    tree_map_pub_ = this->create_publisher<rs1_interfaces::msg::TreeMap>(
        "/parrot1/tree_map", rclcpp::QoS(1).reliable().transient_local());

    // Velocity sweep follower: drive /parrot1/cmd_vel (terrain following adds z)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/parrot1/cmd_vel", rclcpp::QoS(10));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/parrot1/scan", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ParrotMissionExecutorNode::scanCb, this, std::placeholders::_1));

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
    navigate_through_poses_goal_handle_.reset();
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
    if (sweep_controller_ == "nav2" && !navigate_through_poses_client_->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting SampleROI goal: NavigateThroughPoses server unavailable");
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
        if (descend_resume_sweep_)
        {
            descend_resume_sweep_ = false;
            return;
        }
        if (ascend_resume_sweep_)
        {
            ascend_resume_sweep_ = false;
            return;
        }
        beginSweep();
        return;
    }

    auto request = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
    request->clearance = mission_goal->get_goal()->search_clearance;
    commanded_clearance_m_ = static_cast<double>(request->clearance);
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

    // Descending after a pop-up recovery (or ascending after a duck): the sweep
    // is already in flight, so do NOT re-plan it from the drone's current
    // position. Re-running beginSweep() here regenerated the whole boustrophedon
    // from wherever the drone happened to be and made it wander off the lanes.
    if (descend_resume_sweep_)
    {
        descend_resume_sweep_ = false;
        return;
    }
    if (ascend_resume_sweep_)
    {
        ascend_resume_sweep_ = false;
        return;
    }

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

    // Nav2 flies the lanes by default now that the parrot obstacle layer keeps
    // the lidar origin inside the costmap at flight altitude. "velocity" remains
    // available as an opt-out for tests below the canopy.
    if (sweep_controller_ != "nav2")
    {
        beginVelocitySweep();
        return;
    }

    nav2_msgs::action::NavigateThroughPoses::Goal goal;
    goal.poses = poses;

    try
    {
        action_goal_pending_ = true;
        navigate_through_poses_client_->async_send_goal(goal, navigate_through_poses_goal_options_);
    }
    catch (const std::exception &e)
    {
        action_goal_pending_ = false;
        abortSampling(std::string("Could not send the ROI sweep goal: ") + e.what());
    }
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
    // The extra start double pass flies lane 0 twice, so from lane 1 onwards the
    // alternation is shifted by one or the drone would cross the box diagonally.
    auto emit_lane = [&](std::size_t k)
    {
        const std::size_t parity = (start_double_pass_ && k > 0) ? (k - 1) : k;
        const bool toward_high =
            (parity % 2 == 0) ? start_low_lane : !start_low_lane;
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
        bool exit_high = toward_high;

        if (start_double_pass_ && k == 0)
        {
            // Extra out-and-back over the first lane so trees right at the
            // sweep start are seen twice instead of once.  Only the return leg
            // is emitted here; emit_lane already flew the outbound leg.
            emit(toward_high ? (lane_lo - lead) : (lane_hi + lead), lane_offset(0),
                 toward_high ? yaw_low : yaw_high);
            exit_high = !toward_high;
        }

        if (k + 1 < swaths)
        {
            // Rounded U-turn: swing out past the ROI edge in clear air before
            // lining up on the next lane.
            const double ord_mid = 0.5 * (lane_offset(k) + lane_offset(k + 1));
            const double lane_out = exit_high ? (lane_hi + lead + bulge)
                                              : (lane_lo - lead - bulge);
            emit(lane_out, ord_mid, exit_high ? yaw_high : yaw_low);
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

void ParrotMissionExecutorNode::navigateThroughPosesFeedbackCb(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
    if (!feedback || !goal_handle || !navigate_through_poses_goal_handle_ ||
        goal_handle->get_goal_id() != navigate_through_poses_goal_handle_->get_goal_id()) return;

    waypoints_remaining_ = feedback->number_of_poses_remaining;
    publishSamplingFeedback("Sweeping ROI");
}

void ParrotMissionExecutorNode::navigateThroughPosesResponseCb(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle)
{
    action_goal_pending_ = false;
    if (!goal_handle)
    {
        retrySweepWithExploration("Nav2 rejected the ROI sweep goal");
        return;
    }

    navigate_through_poses_goal_handle_ = goal_handle;
    if (sample_roi_goal_handle_ && sample_roi_goal_handle_->is_canceling())
    {
        cancelSampling();
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "ROI sweep goal accepted");
}

void ParrotMissionExecutorNode::navigateThroughPosesResultCb(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result)
{
    if (!sample_roi_goal_handle_ || !navigate_through_poses_goal_handle_ ||
        result.goal_id != navigate_through_poses_goal_handle_->get_goal_id()) return;

    navigate_through_poses_goal_handle_.reset();
    if (sample_roi_goal_handle_->is_canceling())
    {
        finishCancelledSampling();
        return;
    }

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ROI sweep finished with %zu tree(s)", tree_map_.size());
            finishSampling();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            retrySweepWithExploration("sweep aborted by Nav2");
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
    commanded_clearance_m_ = static_cast<double>(request->clearance);
    set_clearance_client_->async_send_request(request);
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
        if (navigate_through_poses_goal_handle_)
        {
            navigate_through_poses_client_->async_cancel_goal(
                navigate_through_poses_goal_handle_,
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
        action_goal_pending_ || navigate_through_poses_goal_handle_ ||
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
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::CancelResponse::SharedPtr response)
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
        RCLCPP_WARN(this->get_logger(),
            "ExploreToPose server unavailable; attempting the sweep without exploration");
        beginSweep();
        return;
    }

    // Drive the weighted frontier explorer toward the centre of the ROI. This
    // is the same exploration used in phase 1; it grows the costmap so the
    // sweep can be planned.
    rs1_interfaces::action::ExploreToPose::Goal explore_goal;
    explore_goal.target_pose.header.frame_id = sweep_frame_;
    explore_goal.target_pose.header.stamp = this->now();
    explore_goal.target_pose.pose.position.x = 0.5 * (sweep_min_.x + sweep_max_.x);
    explore_goal.target_pose.pose.position.y = 0.5 * (sweep_min_.y + sweep_max_.y);
    explore_goal.target_pose.pose.position.z = sweep_z_;
    explore_goal.target_pose.pose.orientation.w = 1.0;

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
            RCLCPP_INFO(this->get_logger(), "Exploration reached the ROI; resuming the sweep");
            beginSweep();
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

double ParrotMissionExecutorNode::distanceToCurrentWaypoint(
    const geometry_msgs::msg::PoseStamped & pose) const
{
    if (sweep_index_ >= sweep_waypoints_.size()) return std::numeric_limits<double>::infinity();
    const auto &t = sweep_waypoints_[sweep_index_].pose.position;
    return std::hypot(t.x - pose.pose.position.x, t.y - pose.pose.position.y);
}

void ParrotMissionExecutorNode::beginVelocitySweep()
{
    sweep_index_ = 0;
    velocity_sweep_active_ = true;
    segment_ready_ = false;
    avoid_offset_ = 0.0;
    avoid_side_ = 0;
    avoid_engaged_ = false;
    avoid_clear_ticks_ = 0;
    avoid_stuck_ticks_ = 0;
    reverse_ticks_ = 0;
    reversing_ = false;
    skip_ticks_ = 0;
    no_progress_ticks_ = 0;
    leg_best_along_ = -1e18;
    consecutive_skips_ = 0;
    hard_wedge_ticks_ = 0;
    popup_active_ = false;
    descend_resume_sweep_ = false;
    duck_active_ = false;
    ascend_resume_sweep_ = false;
    duck_grace_ticks_ = 0;
    popup_grace_ticks_ = 0;
    alt_recover_active_ = false;
    alt_recover_target_dist_ = 1e18;
    wedge_attempts_ = 0;
    roi_escape_ticks_ = 0;
    roi_transit_ticks_ = 0;
    entered_roi_ = false;

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / sweep_control_hz_));
    if (sweep_timer_) sweep_timer_->cancel();
    sweep_timer_ = this->create_wall_timer(
        std::max(period, std::chrono::milliseconds(20)),
        std::bind(&ParrotMissionExecutorNode::sweepControlCb, this));

    RCLCPP_INFO(this->get_logger(),
        "Flying the ROI sweep with the velocity follower (%zu waypoints)", sweep_waypoints_.size());
}

void ParrotMissionExecutorNode::sweepControlCb()
{
    if (!velocity_sweep_active_) return;
    if (!sample_roi_goal_handle_ || !sample_roi_goal_handle_->is_active() ||
        sample_roi_goal_handle_->is_canceling())
    {
        stopSweep();
        return;
    }

    constexpr double kPi = 3.14159265358979323846;

    geometry_msgs::msg::PoseStamped current;
    if (!currentPose(current)) return;

    // Lidar watchdog: a dead GPU lidar leaves stale scans that drive phantom
    // avoidance, and tree anchoring needs the lidar.  Warn early, abort late.
    if (scanFresh())
    {
        scan_outage_ticks_ = 0;
        scan_outage_warned_ = false;
    }
    else
    {
        ++scan_outage_ticks_;
        if (!scan_outage_warned_ &&
            scan_outage_ticks_ >= scan_outage_warn_ticks_threshold_)
        {
            scan_outage_warned_ = true;
            RCLCPP_WARN(this->get_logger(),
                "No lidar scan for %.1f s; the GPU lidar may have died "
                "(obstacle avoidance and tree anchoring disabled)",
                static_cast<double>(scan_outage_ticks_) / sweep_control_hz_);
        }
        if (scan_outage_ticks_ >= scan_outage_ticks_threshold_)
        {
            stopSweep();
            abortSampling(
                "Sweep aborted: lidar scan stopped for " +
                std::to_string(static_cast<int>(scan_outage_timeout_)) +
                " s. The GPU lidar sensor has died; tree anchoring is impossible.");
            return;
        }
    }

    // Altitude recovery watchdog: after a duck / pop-up / canopy descent, resume
    // the search clearance once the drone has physically moved a short distance
    // past the obstacle.  Waiting for waypoint arrival leaves it at the altered
    // height for a whole 100 m lane ("too high for too long").
    if (alt_recover_active_ && (duck_active_ || popup_active_))
    {
        const double moved = std::hypot(
            current.pose.position.x - alt_recover_origin_x_,
            current.pose.position.y - alt_recover_origin_y_);
        // Closing on the waypoint, not just displacement: backing out of a
        // wedge or drifting sideways does not count as "past the obstacle",
        // otherwise the recovery ends before the branch is actually cleared
        // and the drone drops straight back into it.
        double target_dist = std::numeric_limits<double>::infinity();
        if (sweep_index_ < sweep_waypoints_.size())
        {
            const auto &t = sweep_waypoints_[sweep_index_].pose.position;
            target_dist = std::hypot(
                t.x - current.pose.position.x, t.y - current.pose.position.y);
        }
        const bool clear_of_obstacle =
            std::isfinite(alt_recover_target_dist_) &&
            target_dist <= alt_recover_target_dist_ - alt_recover_travel_m_;
        if (clear_of_obstacle && moved >= alt_recover_travel_m_ * 0.5)
        {
            alt_recover_active_ = false;
            wedge_attempts_ = 0;
            const bool was_duck = duck_active_;
            duck_active_ = false;
            popup_active_ = false;
            if (was_duck)
            {
                ascend_resume_sweep_ = true;
                setSearchClearance();
                RCLCPP_INFO(this->get_logger(),
                    "Climbed %.1f m of the waypoint after ducking (moved %.1f m); rising to the search clearance",
                    alt_recover_target_dist_ - target_dist, moved);
            }
            else
            {
                descend_resume_sweep_ = true;
                setSearchClearance();
                RCLCPP_INFO(this->get_logger(),
                    "Climbed %.1f m of the waypoint after pop-up (moved %.1f m); descending to the search clearance",
                    alt_recover_target_dist_ - target_dist, moved);
            }
        }
    }

    // Safety net: if the map-frame pose disagrees badly with the ROI box (broken
    // world<->map registration, or a localisation jump), the drone chases
    // waypoints that are not where it believes they are and leaves the ROI
    // entirely.  Abort with a clear reason instead of wandering into blank space.
    {
        const double escape_x = std::max(
            0.0, std::max(sweep_min_.x - current.pose.position.x,
                          current.pose.position.x - sweep_max_.x));
        const double escape_y = std::max(
            0.0, std::max(sweep_min_.y - current.pose.position.y,
                          current.pose.position.y - sweep_max_.y));
        const double escape = std::hypot(escape_x, escape_y);
        if (!entered_roi_)
        {
            // Inbound transit: an untel eported drone starts outside the box and
            // flies in, so being far from the ROI is expected.  Bound how long
            // that may take instead of aborting on distance.
            if (escape <= roi_enter_margin_)
            {
                entered_roi_ = true;
                roi_escape_ticks_ = 0;
                roi_transit_ticks_ = 0;
                RCLCPP_INFO(this->get_logger(), "Entered the ROI; starting the sweep lanes");
                if (descend_on_entry_)
                {
                    descend_on_entry_ = false;
                    if (set_clearance_client_ && set_clearance_client_->service_is_ready() &&
                        sample_roi_goal_handle_)
                    {
                        auto request =
                            std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
                        request->clearance =
                            sample_roi_goal_handle_->get_goal()->search_clearance;
                        set_clearance_client_->async_send_request(request);
                        RCLCPP_INFO(this->get_logger(),
                            "Descending to the search clearance for the sweep");
                    }
                }
            }
            else if (++roi_transit_ticks_ >= roi_transit_ticks_threshold_)
            {
                stopSweep();
                abortSampling(
                    "Sweep aborted: could not reach the ROI within " +
                    std::to_string(static_cast<int>(roi_transit_timeout_)) +
                    " s. Check the world<->parrot1_map frame and the drone's start pose.");
                return;
            }
        }
        else if (escape > roi_escape_margin_)
        {
            if (++roi_escape_ticks_ >= roi_escape_ticks_threshold_)
            {
                stopSweep();
                abortSampling(
                    "Sweep aborted: drone left the ROI by more than " +
                    std::to_string(static_cast<int>(roi_escape_margin_)) +
                    " m, so world and parrot1_map disagree. Check /mission/alignment "
                    "and map->odom before sweeping.");
                return;
            }
        }
        else
        {
            roi_escape_ticks_ = 0;
        }
    }

    // Consume any waypoints we are already standing on.
    bool advanced = false;
    bool arrived = false;
    while (sweep_index_ < sweep_waypoints_.size())
    {
        const auto &t = sweep_waypoints_[sweep_index_].pose.position;
        if (std::hypot(t.x - current.pose.position.x, t.y - current.pose.position.y) <= waypoint_tolerance_)
        {
            // The next leg runs from this waypoint to the following one.
            seg_start_ = t;
            segment_ready_ = true;
            avoid_offset_ = 0.0;
            avoid_engaged_ = false;
            avoid_side_ = 0;
            avoid_clear_ticks_ = 0;
            avoid_stuck_ticks_ = 0;
            reverse_ticks_ = 0;
            reversing_ = false;
            skip_ticks_ = 0;
            no_progress_ticks_ = 0;
            consecutive_skips_ = 0;
            leg_best_along_ = -1e18;
            ++sweep_index_;
            advanced = true;
            arrived = true;
            continue;
        }
        break;
    }

    // If the drone is close to the waypoint but an obstacle (e.g. a tree) keeps
    // it from reaching the tolerance, give up and move on to the next point
    // instead of fighting the collision avoidance forever.
    if (sweep_index_ < sweep_waypoints_.size())
    {
        const auto &t = sweep_waypoints_[sweep_index_].pose.position;
        const double dist_to_target =
            std::hypot(t.x - current.pose.position.x, t.y - current.pose.position.y);
        if (dist_to_target <= waypoint_skip_distance_ &&
            dist_to_target > waypoint_tolerance_ &&
            (avoid_engaged_ || reversing_))
        {
            if (++skip_ticks_ >= skip_ticks_threshold_)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Waypoint unreachable with an obstacle in the way; skipping it");
                seg_start_ = t;
                segment_ready_ = true;
                avoid_offset_ = 0.0;
                avoid_engaged_ = false;
                avoid_side_ = 0;
                avoid_clear_ticks_ = 0;
                avoid_stuck_ticks_ = 0;
                reverse_ticks_ = 0;
                reversing_ = false;
                skip_ticks_ = 0;
                no_progress_ticks_ = 0;
                leg_best_along_ = -1e18;
                ++sweep_index_;
                advanced = true;
                if (++consecutive_skips_ >= max_consecutive_skips_)
                {
                    maybeDuckRecovery();
                }
            }
        }
        else
        {
            skip_ticks_ = 0;
        }
    }

    // Stuck mid-lane safety: if the drone stops making ALONG-TRACK progress
    // toward the current waypoint (e.g. wedged against a trunk with no side
    // route), give up on this leg and move on instead of oscillating back and
    // forth forever.  Progress is measured ALONG the leg, not as Euclidean
    // distance to the endpoint, so legitimately weaving sideways around trees
    // no longer counts as "blocked" - previously this fired on every U-turn and
    // skipped healthy lanes, leaving parts of the ROI unswept.
    if (sweep_index_ < sweep_waypoints_.size())
    {
        const auto &t = sweep_waypoints_[sweep_index_].pose.position;
        double leg_ux = t.x - seg_start_.x;
        double leg_uy = t.y - seg_start_.y;
        const double leg_len = std::hypot(leg_ux, leg_uy);
        double along = 0.0;
        if (leg_len >= 1e-3)
        {
            leg_ux /= leg_len;
            leg_uy /= leg_len;
            along = std::clamp(
                (current.pose.position.x - seg_start_.x) * leg_ux +
                    (current.pose.position.y - seg_start_.y) * leg_uy,
                0.0, leg_len);
        }
        const bool progressing = along > leg_best_along_ + 0.3;
        if (popup_grace_ticks_ > 0 || duck_grace_ticks_ > 0)
        {
            // Climbing after a pop-up (or descending after a duck): hold off
            // skipping so the altitude change has time to clear the wedge
            // before the watchdog fires again.
            if (popup_grace_ticks_ > 0)
            {
                if (progressing)
                {
                    popup_grace_ticks_ = 0;
                }
                else if (--popup_grace_ticks_ == 0 && popup_active_)
                {
                    // The pop-up did not clear the obstructed band either (the
                    // canopy was deeper than the target height).  Come back down
                    // and escalate: the next pop-up climbs one step higher.
                    popup_active_ = false;
                    alt_recover_active_ = false;
                    ++wedge_attempts_;
                    descend_resume_sweep_ = true;
                    setSearchClearance();
                    RCLCPP_WARN(this->get_logger(),
                        "Pop-up did not clear the obstructed band; the next recovery will climb higher");
                }
            }
            if (duck_grace_ticks_ > 0)
            {
                if (progressing)
                {
                    // The duck is working (moving under the branches): hand
                    // control back to the normal no-progress watchdog instead
                    // of timing out to a pop-up mid-lane.
                    duck_grace_ticks_ = 0;
                }
                else if (--duck_grace_ticks_ == 0 && duck_active_)
                {
                    // The duck never cleared the wedge; fall back to popping
                    // above the canopy.
                    duck_active_ = false;
                    ++wedge_attempts_;
                    maybePopUpRecovery();
                }
            }
            if (progressing)
            {
                leg_best_along_ = along;
            }
            no_progress_ticks_ = 0;
        }
        else if (progressing)
        {
            leg_best_along_ = along;
            no_progress_ticks_ = 0;
        }
        else if (++no_progress_ticks_ >= no_progress_ticks_threshold_)
        {
            // NOTE: `reversing_` deliberately does NOT reset this timer any more.
            // A reverse/dodge oscillation makes negative along-track progress, and
            // clearing the timer on every reverse tick meant a genuine wedge could
            // sit there for minutes without ever attempting a recovery - which is
            // exactly the "it waits too long and then gives up on the lane".
            no_progress_ticks_ = 0;
            bool skip_leg = false;
            if (!duck_active_ && !popup_active_)
            {
                if (wedge_attempts_ >= max_recovery_attempts_)
                {
                    // The whole ladder has been tried on this spot; stop wasting
                    // time here and sacrifice the leg.
                    skip_leg = true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "No along-track progress for %.1f s (blocked); attempting an altitude recovery (%d/%d)",
                        no_progress_timeout_, wedge_attempts_ + 1, max_recovery_attempts_);
                    maybeDuckRecovery();
                    // No recovery was possible (service not ready): drop the leg
                    // instead of sitting here forever.
                    skip_leg = !duck_active_ && !popup_active_;
                }
            }
            else
            {
                // A recovery is still holding the drone at the altered height but
                // it has stalled again: count it as a failed rung and escalate
                // (the next attempt ducks and then pops one step higher) rather
                // than abandoning the lane.
                duck_active_ = false;
                popup_active_ = false;
                alt_recover_active_ = false;
                ++wedge_attempts_;
                descend_resume_sweep_ = true;
                setSearchClearance();
                RCLCPP_WARN(this->get_logger(),
                    "Altitude recovery did not clear the obstruction (%d/%d); escalating",
                    wedge_attempts_, max_recovery_attempts_);
                skip_leg = wedge_attempts_ >= max_recovery_attempts_;
            }

            if (skip_leg)
            {
                RCLCPP_WARN(this->get_logger(),
                    "No along-track progress for %.1f s (blocked); skipping leg",
                    no_progress_timeout_);
                seg_start_ = t;
                segment_ready_ = true;
                avoid_offset_ = 0.0;
                avoid_engaged_ = false;
                avoid_side_ = 0;
                avoid_clear_ticks_ = 0;
                avoid_stuck_ticks_ = 0;
                reverse_ticks_ = 0;
                reversing_ = false;
                skip_ticks_ = 0;
                no_progress_ticks_ = 0;
                leg_best_along_ = -1e18;
                wedge_attempts_ = 0;
                ++sweep_index_;
                advanced = true;
                if (++consecutive_skips_ >= max_consecutive_skips_)
                {
                    maybeDuckRecovery();
                }
            }
        }
    }

    if (advanced)
    {
        // If the drone popped up to clear a wedge and has now physically
        // reached a waypoint, descend back to the search clearance to resume
        // trunk-level detection.
        if (arrived && duck_active_)
        {
            duck_active_ = false;
            alt_recover_active_ = false;
            alt_recover_target_dist_ = 1e18;
            wedge_attempts_ = 0;
            ascend_resume_sweep_ = true;
            setSearchClearance();
            RCLCPP_INFO(this->get_logger(),
                "Reached a waypoint after ducking; ascending to search clearance");
        }
        else if (arrived && popup_active_)
        {
            popup_active_ = false;
            alt_recover_active_ = false;
            alt_recover_target_dist_ = 1e18;
            wedge_attempts_ = 0;
            descend_resume_sweep_ = true;
            setSearchClearance();
            RCLCPP_INFO(this->get_logger(),
                "Reached a waypoint after pop-up; descending to search clearance");
        }
        waypoints_remaining_ = static_cast<int16_t>(
            std::min<std::size_t>(sweep_waypoints_.size() - sweep_index_, 32767));
        publishSamplingFeedback("Sweeping ROI");
    }

    if (sweep_index_ >= sweep_waypoints_.size())
    {
        stopSweep();
        finishSampling();
        return;
    }

    const auto &target = sweep_waypoints_[sweep_index_].pose.position;
    if (!segment_ready_)
    {
        seg_start_ = current.pose.position;
        segment_ready_ = true;
    }

    // Follow the swath line: aim at a look-ahead point on the segment, then add
    // a perpendicular avoidance offset that decays back to zero (rejoining the
    // line) once a way around the obstacle opens up.
    double ux = target.x - seg_start_.x;
    double uy = target.y - seg_start_.y;
    const double seg_len = std::hypot(ux, uy);
    if (seg_len < 1e-3)
    {
        ux = 1.0;
        uy = 0.0;
    }
    else
    {
        ux /= seg_len;
        uy /= seg_len;
    }

    const double px = current.pose.position.x;
    const double py = current.pose.position.y;
    const auto &q = current.pose.orientation;
    const double current_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double along = std::clamp(
        (px - seg_start_.x) * ux + (py - seg_start_.y) * uy, 0.0, seg_len);
    const double look = std::min(along + lookahead_distance_, seg_len);
    const double path_x = seg_start_.x + ux * look;
    const double path_y = seg_start_.y + uy * look;
    const double remaining = std::max(0.0, seg_len - along);
    const bool near_end = remaining <= lookahead_distance_;
    const double perp_x = -uy;  // left of travel direction
    const double perp_y = ux;

    // Pick a side to detour to when something is close ahead, and hold that
    // side for the whole encounter so noisy canopy returns cannot flip it back
    // and forth. Engagement uses hysteresis: it only clears once the obstacle
    // stays beyond the avoidance distance plus a margin for several ticks.
    double left = std::numeric_limits<double>::infinity();
    double front = std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    const bool have_scan = scanClearances(left, front, right);
    const bool front_blocked = have_scan && std::isfinite(front) && front < avoid_distance_;

    // Widest contiguous angular gap in the forward half-plane. Unlike the fixed
    // min-range cones, this cannot be poisoned by a single nearby trunk and is
    // used to (a) pick the escape side and (b) declare a route open.
    double gap_centre = 0.0;
    double gap_width = -1.0;
    const bool have_gap = widestGap(gap_centre, gap_width);
    const bool gap_open = have_gap && gap_width >= min_gap_width_;

    if (front_blocked)
    {
        avoid_engaged_ = true;
        avoid_clear_ticks_ = 0;
    }
    else if (avoid_engaged_)
    {
        if (!have_scan || !std::isfinite(front) || front > avoid_distance_ + avoid_disengage_margin_)
        {
            if (++avoid_clear_ticks_ >= 4)
            {
                avoid_engaged_ = false;
                avoid_side_ = 0;
                avoid_clear_ticks_ = 0;
            }
        }
        else
        {
            avoid_clear_ticks_ = 0;
        }
    }

    // Whether the endpoint is visible from here (clear line of sight).
    const double target_bearing = std::remainder(
        std::atan2(target.y - py, target.x - px) - current_yaw, 2.0 * kPi);
    double target_range = std::numeric_limits<double>::infinity();
    const bool target_clear = !scanMinRange(target_bearing, steering_cone_, target_range) ||
        target_range > obstacle_stop_distance_;

    double desired_offset = 0.0;
    if (avoid_engaged_)
    {
        if (avoid_side_ == 0)
        {
            // Head for the widest angular gap; fall back to the roomier fixed
            // side cone only when no contiguous gap was found at all.
            if (have_gap)
            {
                avoid_side_ = (gap_centre >= 0.0) ? 1 : -1;
            }
            else
            {
                avoid_side_ = (left >= right) ? 1 : -1;
            }
        }
        desired_offset = static_cast<double>(avoid_side_) * avoid_offset_m_;
    }
    else if (near_end && !target_clear)
    {
        // Hold the current detour offset while the endpoint is still blocked so
        // the drone keeps going around instead of rejoining the blocked line.
        desired_offset = avoid_offset_;
    }
    avoid_offset_ += std::clamp(desired_offset - avoid_offset_, -avoid_rate_, avoid_rate_);
    avoid_offset_ = std::clamp(avoid_offset_, -avoid_offset_m_, avoid_offset_m_);

    const double steer_x = path_x + perp_x * avoid_offset_;
    const double steer_y = path_y + perp_y * avoid_offset_;
    const double sdx = steer_x - px;
    const double sdy = steer_y - py;
    const double sdist = std::hypot(sdx, sdy);
    const double desired_yaw = (sdist > 1e-3) ? std::atan2(sdy, sdx) : current_yaw;
    const double heading_error = std::remainder(desired_yaw - current_yaw, 2.0 * kPi);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(sweep_yaw_gain_ * heading_error, -max_yaw_rate_, max_yaw_rate_);

    // Reduce speed and add a little yaw damping when the robot is close to a
    // canopy obstacle so it does not oscillate on the obstacle boundary.
    const double obstacle_bias = (std::isfinite(front) && front < avoid_distance_ * 1.5) ? 0.5 : 1.0;
    cmd.angular.z *= obstacle_bias;

    // Allow a slow forward push whenever the chosen detour path has room, even
    // if the exact centerline is temporarily blocked by a trunk or canopy.
    double steer_range = std::numeric_limits<double>::infinity();
    const bool steer_clear =
        !scanMinRange(heading_error, steering_cone_, steer_range) ||
        steer_range > obstacle_stop_distance_;
    const bool side_route_open =
        (std::isfinite(left) && left > obstacle_stop_distance_ * 0.75) ||
        (std::isfinite(right) && right > obstacle_stop_distance_ * 0.75);

    const bool can_drive = std::abs(heading_error) < heading_gate_ &&
        (steer_clear || side_route_open || gap_open);
    if (can_drive)
    {
        const double speed = std::min(max_sweep_speed_, sweep_speed_gain_ * sdist);
        double linear_speed = std::max(0.0, speed);
        if (front_blocked)
        {
            // Slow down in proportion to the remaining clearance instead of the
            // old fixed 45% multiplier.  With avoid_distance at 5 m there is
            // almost always a tree inside that radius in this forest, so the
            // fixed factor pinned the sweep to 0.45*max_sweep_speed (1.35 m/s)
            // and made the higher top speed pointless.
            const double clearance = std::isfinite(front) ? front : avoid_distance_;
            linear_speed *= std::clamp(
                clearance / avoid_distance_, avoid_min_slowdown_, 1.0);
        }
        cmd.linear.x = linear_speed;
    }

    // If the held side turns out to be fully blocked (trunk dead ahead with no
    // room either side at lidar height), flip the side once after a short
    // delay instead of oscillating or freezing in place.
    if (avoid_engaged_ && !steer_clear && !side_route_open)
    {
        if (++avoid_stuck_ticks_ >= 25)
        {
            avoid_side_ *= -1;
            avoid_stuck_ticks_ = 0;
            avoid_clear_ticks_ = 0;
        }
    }
    else
    {
        avoid_stuck_ticks_ = 0;
    }

    // Hard-wedge detection: stuck near the end of a leg with no forward route.
    // The reverse/avoidance oscillation below keeps flipping reversing_ on and
    // off, which suppresses the no-progress watchdog, so this uses its own
    // timer and ducks/ pops up to break the deadlock.
    if (!can_drive && near_end)
    {
        if (++hard_wedge_ticks_ >= hard_wedge_ticks_threshold_)
        {
            hard_wedge_ticks_ = 0;
            if (wedge_attempts_ >= max_recovery_attempts_)
            {
                // The whole ladder has already been tried on this spot.  This
                // branch used to call maybeDuckRecovery() unconditionally, and
                // because the grace-window expiries bump wedge_attempts_ without
                // consulting the limit, the drone re-armed the ladder forever:
                // observed 20 recovery attempts, 0 leg skips, sweep frozen at
                // 44% until the test timed out.  Force the no-progress watchdog
                // to fire so the leg is abandoned through that single path.
                no_progress_ticks_ = no_progress_ticks_threshold_;
            }
            else
            {
                maybeDuckRecovery();
            }
        }
    }
    else
    {
        hard_wedge_ticks_ = 0;
    }

    // When wedged near the end of a leg with no forward route, back out to make
    // room and then drive around, instead of sitting still. Once reversing,
    // keep going until there is clearly clear space ahead so it does not hunt
    // back and forth.
    if (reversing_)
    {
        if (std::isfinite(front) && front > avoid_distance_ + avoid_disengage_margin_)
        {
            reversing_ = false;
        }
        else
        {
            double rear_range = std::numeric_limits<double>::infinity();
            const bool rear_clear = !scanMinRange(kPi, steering_cone_, rear_range) ||
                rear_range > obstacle_stop_distance_ * 0.8;
            if (rear_clear)
            {
                cmd.linear.x = -0.8;
                cmd.angular.z = 0.0;
            }
            else
            {
                reversing_ = false;
            }
        }
    }
    else if (!can_drive && near_end)
    {
        if (++reverse_ticks_ >= 8)
        {
            reversing_ = true;
            reverse_ticks_ = 0;
        }
    }
    else
    {
        reverse_ticks_ = 0;
    }

    cmd_vel_pub_->publish(cmd);
}

void ParrotMissionExecutorNode::maybeDuckRecovery()
{
    // Hysteresis for "the drone is flying above the sweep altitude" so noise
    // around the search clearance does not cancel a duck.
    constexpr double kElevatedClearanceMargin = 0.3;
    // Slip UNDER the oak branches instead of popping over the canopy.  At the
    // search clearance the forward lidar sits in the branch zone and sees oak
    // branches in every direction, but a metre off the deck only the widely
    // spaced trunks remain, so the gap-seeking avoidance can find a route while
    // tree detection stays active.  Ascend again on arrival (the
    // `arrived && duck_active_` branch in sweepControlCb); if the duck does not
    // clear the wedge in time the grace countdown falls back to a pop-up.
    if (duck_active_ || popup_active_)
    {
        return;
    }
    if (!set_clearance_client_ || !set_clearance_client_->service_is_ready())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cannot duck: SetClearance service unavailable");
        return;
    }

    // If the drone is currently flying well ABOVE the search altitude (e.g. it
    // is still up at the transit clearance, or a pop-up left it high), dropping
    // straight to the duck clearance would plunge it through the branches.
    // Return to the search clearance first; ducking only makes sense once it is
    // already down at trunk level.
    //
    // This is judged from the clearance we last COMMANDED, not from a world-z
    // difference against the last waypoint: the ground in v2_forest rises by up
    // to ~7 m across a lane, so a world-z test fired constantly on the uphill
    // half of the ROI and silently turned every duck into a no-op descent to
    // the search clearance - the drone then stayed wedged in the branches with
    // only a warning in the log to show for it.
    double target = duck_clearance_;
    geometry_msgs::msg::PoseStamped here;
    const auto mission_goal = sample_roi_goal_handle_;
    const double search_clearance =
        mission_goal ? static_cast<double>(mission_goal->get_goal()->search_clearance)
                     : duck_clearance_;
    if (commanded_clearance_m_ > 0.0 &&
        commanded_clearance_m_ > search_clearance + kElevatedClearanceMargin)
    {
        target = search_clearance;
        RCLCPP_WARN(this->get_logger(),
            "Currently holding a %.1f m clearance; descending to the search clearance (%.1f m) "
            "before ducking under the branches",
            commanded_clearance_m_, search_clearance);
    }

    duck_active_ = true;
    duck_grace_ticks_ = duck_grace_ticks_threshold_;
    if (currentPose(here))
    {
        alt_recover_origin_x_ = here.pose.position.x;
        alt_recover_origin_y_ = here.pose.position.y;
        alt_recover_target_dist_ = distanceToCurrentWaypoint(here);
        alt_recover_active_ = true;
    }
    auto req = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
    req->clearance = target;
    commanded_clearance_m_ = static_cast<double>(target);
    set_clearance_client_->async_send_request(req);
    RCLCPP_WARN(this->get_logger(),
        "Recovery descent to %.1f m clearance (attempt %d)", target, wedge_attempts_ + 1);
}

void ParrotMissionExecutorNode::maybePopUpRecovery()
{
    // Ascend above the canopy (the drone is a quadcopter; up is always clear)
    // so the lidar sees open space and the follower can reach the next
    // waypoint.  The descend happens on physical arrival (the
    // `arrived && popup_active_` branch in sweepControlCb).
    if (popup_active_)
    {
        return;
    }
    if (!set_clearance_client_ || !set_clearance_client_->service_is_ready())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cannot pop up: SetClearance service unavailable");
        return;
    }
    popup_active_ = true;
    // Give the drone time to climb (max_vertical_speed is now 2 m/s, so 8 m
    // takes ~4 s) before the no-progress watchdog starts skipping legs again.
    popup_grace_ticks_ = popup_grace_ticks_threshold_;
    // Escalate: the first pop-up targets popup_clearance (above the ~6.5 m oak
    // canopy); every further attempt in the same wedge climbs another step, up
    // to popup_max_clearance, so a taller stand is still cleared instead of
    // looping at a height that is inside the branches.
    const double climb_to = std::min(
        popup_max_clearance_,
        popup_clearance_ + popup_step_m_ * static_cast<double>(wedge_attempts_));
    geometry_msgs::msg::PoseStamped here;
    if (currentPose(here))
    {
        alt_recover_origin_x_ = here.pose.position.x;
        alt_recover_origin_y_ = here.pose.position.y;
        alt_recover_target_dist_ = distanceToCurrentWaypoint(here);
        alt_recover_active_ = true;
    }
    auto req = std::make_shared<rs1_interfaces::srv::SetClearance::Request>();
    req->clearance = climb_to;
    commanded_clearance_m_ = static_cast<double>(climb_to);
    set_clearance_client_->async_send_request(req);
    RCLCPP_WARN(this->get_logger(),
        "Popping up to %.1f m to clear the canopy (attempt %d)", climb_to, wedge_attempts_ + 1);
}

bool ParrotMissionExecutorNode::forwardClear(double & min_range) const
{
    min_range = std::numeric_limits<double>::infinity();
    if (!scanFresh()) return true;

    const auto &scan = *latest_scan_;
    if (scan.ranges.empty() || scan.angle_increment == 0.0) return true;

    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        if (std::abs(angle) > forward_cone_) continue;
        const double range = scan.ranges[i];
        if (!std::isfinite(range) || range <= 0.0) continue;
        min_range = std::min(min_range, range);
    }
    if (!std::isfinite(min_range)) return true;
    return min_range > obstacle_stop_distance_;
}

void ParrotMissionExecutorNode::stopSweep()
{
    if (sweep_timer_)
    {
        sweep_timer_->cancel();
        sweep_timer_.reset();
    }
    velocity_sweep_active_ = false;

    // VelocityControl holds the last command, so actively command a stop.
    if (cmd_vel_pub_)
    {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
}

void ParrotMissionExecutorNode::scanCb(const sensor_msgs::msg::LaserScan & msg)
{
    latest_scan_ = std::make_shared<const sensor_msgs::msg::LaserScan>(msg);
    latest_scan_time_ = this->now();
}

bool ParrotMissionExecutorNode::scanFresh() const
{
    if (!latest_scan_ || latest_scan_time_.nanoseconds() == 0) return false;
    const double age = (this->now() - latest_scan_time_).seconds();
    return age >= 0.0 && age <= scan_stale_timeout_;
}

bool ParrotMissionExecutorNode::scanMinRange(double centre, double half_width, double & out) const
{
    out = std::numeric_limits<double>::infinity();
    if (!scanFresh()) return false;

    const auto &scan = *latest_scan_;
    if (scan.ranges.empty() || scan.angle_increment == 0.0) return false;

    constexpr double kTwoPi = 6.28318530717958647692;
    bool any = false;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        if (std::abs(std::remainder(angle - centre, kTwoPi)) > half_width) continue;
        const double range = scan.ranges[i];
        if (!std::isfinite(range) || range <= 0.0) continue;
        any = true;
        out = std::min(out, range);
    }
    return any;
}

bool ParrotMissionExecutorNode::scanClearances(double & left, double & front, double & right) const
{
    if (!scanFresh()) return false;

    // Front sector is the narrow forward cone; the side sectors cover the rest
    // of the forward half-plane so the detour can pick the roomier side.
    const double side_centre = (forward_cone_ + side_cone_) * 0.5;
    const double side_half = (side_cone_ - forward_cone_) * 0.5;
    const bool have_front = scanMinRange(0.0, forward_cone_, front);
    const bool have_left = scanMinRange(side_centre, side_half, left);
    const bool have_right = scanMinRange(-side_centre, side_half, right);
    return have_front || have_left || have_right;
}

bool ParrotMissionExecutorNode::widestGap(double & centre, double & width) const
{
    centre = 0.0;
    width = -1.0;
    if (!scanFresh()) return false;

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kHalfPi = 1.57079632679489661923;
    const auto & scan = *latest_scan_;
    const std::size_t n = scan.ranges.size();
    if (n == 0 || scan.angle_increment == 0.0) return false;

    // A beam is "clear" when its range is finite and beyond the stop distance,
    // so a single nearby trunk no longer marks a whole fixed cone as blocked.
    const double threshold = obstacle_stop_distance_ * 0.9;

    // Restrict the search to the forward half-plane (bearings within +/-90 deg
    // of the heading) so the drone keeps making progress along the lane.
    std::vector<std::pair<double, bool>> beams;
    beams.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        a = std::remainder(a, 2.0 * kPi);  // [-pi, pi]
        if (std::abs(a) > kHalfPi) continue;
        const double r = scan.ranges[i];
        beams.emplace_back(a, std::isfinite(r) && r > threshold);
    }
    if (beams.empty()) return false;

    // Sort by bearing so the wrap across 0 deg forms one contiguous run.
    std::sort(beams.begin(), beams.end(),
              [](const auto & l, const auto & r) { return l.first < r.first; });

    // Longest contiguous run of clear beams.
    std::size_t best_start = 0, best_len = 0;
    std::size_t run_start = 0, run_len = 0;
    for (std::size_t i = 0; i < beams.size(); ++i)
    {
        if (beams[i].second)
        {
            if (run_len == 0) run_start = i;
            ++run_len;
        }
        else
        {
            if (run_len > best_len) { best_len = run_len; best_start = run_start; }
            run_len = 0;
        }
    }
    if (run_len > best_len) { best_len = run_len; best_start = run_start; }
    if (best_len == 0) return false;

    const double a0 = beams[best_start].first;
    const double a1 = beams[best_start + best_len - 1].first;
    width = std::abs(a1 - a0) + scan.angle_increment;
    centre = 0.5 * (a0 + a1);
    return true;
}


}  // namespace rs1_mission
