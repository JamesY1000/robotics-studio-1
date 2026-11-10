// This node handles the overall mission state machine.
// We call a service or action to start the mission.
// Starting the mission causes the mission state to go from IDLE to phase 1. Get to ROI
// When in phase 1, it calls action servers in both the parrot and husky mission executors to execute the TraverseToROI action.
// It receives updates on the status of both robots and their current task statuses, and updates the overall mission status
// It should go something like from:
// 1. Get to ROI
// 2. Sample trees
// 3. Get to home
// Once both robots are back at their start positions, we finish the mission

#include "mission_coordinator.hpp"
#include "tree_survey.hpp"

#include <exception>
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <string>
#include <stdexcept>
#include <cmath>
#include <tf2/utils.h>

namespace rs1_mission
{

MissionCoordinatorNode::MissionCoordinatorNode() : Node("mission_coordinator_node"),
    mission_state_(MissionState::IDLE),
    parrot_succeded_(false),
    husky_succeded_(false)
{

    // Saved ROI remains in its configured frame; transform before sending goals.
    roi_pose_.header.frame_id = this->declare_parameter<std::string>("roi_frame", "world");
    session_id_ = this->declare_parameter<std::string>("session_id", "manual");
    instance_id_ = std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    status_pub_ = this->create_publisher<rs1_interfaces::msg::MissionStatus>(
        "/mission/status", rclcpp::QoS(1).reliable().transient_local());
    status_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() { publishStatus(); });
    safety_sub_ = create_subscription<rs1_interfaces::msg::SafetyState>(
        "/safety/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](const rs1_interfaces::msg::SafetyState &msg) {
            // Notice a heartbeat gap even if the next heartbeat beats our status timer
            checkSafety();
            safety_ = msg;
            safety_received_ = std::chrono::steady_clock::now();
            checkSafety();
        });
    roi_pose_.pose.position.x = this->declare_parameter<double>("roi_x", -40.0);
    roi_pose_.pose.position.y = this->declare_parameter<double>("roi_y", 40.0);
    roi_pose_.pose.position.z = this->declare_parameter<double>("roi_z", 0.0);
    roi_pose_.pose.orientation.w = 1.0;

    // Axis-aligned ROI box used by the sampling phase. The entry point above is
    // the first-placed corner; the box is the region the parrot sweeps.
    roi_min_.x = this->declare_parameter<double>("roi_x_min", -45.0);
    roi_min_.y = this->declare_parameter<double>("roi_y_min", 35.0);
    roi_min_.z = 0.0;
    roi_max_.x = this->declare_parameter<double>("roi_x_max", -35.0);
    roi_max_.y = this->declare_parameter<double>("roi_y_max", 45.0);
    roi_max_.z = 0.0;
    search_clearance_ = this->declare_parameter<double>("search_clearance", 2.0);
    transit_clearance_ = this->declare_parameter<double>("transit_clearance", 2.0);
    swath_spacing_ = this->declare_parameter<double>("swath_spacing", 0.0);

    // Dashboard launches supply spawn poses, standalone launches remember the first mission start
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    declare_parameter<bool>("home_from_start", true, fixed);
    home_tolerance_ = declare_parameter<double>("home_tolerance_m", 0.5, fixed);
    if (!std::isfinite(home_tolerance_) || home_tolerance_ <= 0)
        throw std::invalid_argument("home_tolerance_m must be positive and finite");
    for (const auto &robot : {"parrot", "husky"})
        declare_parameter<std::vector<double>>(std::string(robot) + "_home", {0.0, 0.0, 0.0, 0.0}, fixed);
    home_returns_[0].client = rclcpp_action::create_client<ReturnHome>(this, "/parrot1/return_home");
    home_returns_[1].client = rclcpp_action::create_client<ReturnHome>(this, "/husky1/return_home");

    // Keep the latest ROI marker for RViz instances opened later
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mission/roi_marker", rclcpp::QoS(1).reliable().transient_local());

    // Latched tree map the Husky samples from, populated from the parrot's
    // SampleROI action result.
    mission_tree_map_pub_ = this->create_publisher<rs1_interfaces::msg::TreeMap>(
        "/mission/tree_map", rclcpp::QoS(1).reliable().transient_local());

    // Show the configured ROI before starting the mission
    publishROIMarker();

    // Parameter services and mission callbacks share the default mutually
    // exclusive callback group. A target update cannot overlap mission start.
    roi_parameters_callback_ = this->add_on_set_parameters_callback(
        std::bind(&MissionCoordinatorNode::validateROIParameters, this, std::placeholders::_1));


    // Action clients and callbacks for the parrot and husky mission executors
    traverse_to_roi_client_parrot_ = rclcpp_action::create_client<rs1_interfaces::action::TraverseToROI>(
        this,
        "/parrot1/traverse_to_roi"
    );

    parrot_goal_options_.goal_response_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIGoalResponseCb,
        this,
        Robot::Parrot,
        std::placeholders::_1
    );

    parrot_goal_options_.feedback_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIFeedbackCb,
        this,
        Robot::Parrot,
        std::placeholders::_1,
        std::placeholders::_2
    );

    parrot_goal_options_.result_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIResultCb,
        this,
        Robot::Parrot,
        std::placeholders::_1
    );

    traverse_to_roi_client_husky_ = rclcpp_action::create_client<rs1_interfaces::action::TraverseToROI>(
        this,
        "/husky1/traverse_to_roi"
    );

    husky_goal_options_.goal_response_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIGoalResponseCb,
        this,
        Robot::Husky,
        std::placeholders::_1
    );

    husky_goal_options_.feedback_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIFeedbackCb,
        this,
        Robot::Husky,
        std::placeholders::_1,
        std::placeholders::_2
    );

    husky_goal_options_.result_callback = std::bind(
        &MissionCoordinatorNode::traverseToROIResultCb,
        this,
        Robot::Husky,
        std::placeholders::_1
    );

    measure_client_ = rclcpp_action::create_client<MeasureTrees>(this, "/husky1/measure_trees");
    survey_pub_ = create_publisher<rs1_interfaces::msg::TreeSurvey>(
        "/mission/tree_survey", rclcpp::QoS(1).reliable().transient_local());
    // A new mission starts with no readings, including for late dashboard subscribers
    survey_.mission_id = instance_id_;
    survey_.tree_map.header.frame_id = "world";
    survey_.tree_map.header.stamp = now();
    mission_tree_map_pub_->publish(survey_.tree_map);
    survey_pub_->publish(survey_);

    // Parrot handles the sweep, then Husky measures the trees it found
    sample_roi_client_parrot_ = rclcpp_action::create_client<rs1_interfaces::action::SampleROI>(
        this,
        "/parrot1/sample_roi"
    );

    parrot_sample_goal_options_.goal_response_callback = std::bind(
        &MissionCoordinatorNode::sampleROIGoalResponseCb,
        this,
        std::placeholders::_1
    );

    parrot_sample_goal_options_.feedback_callback = std::bind(
        &MissionCoordinatorNode::sampleROIFeedbackCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2
    );

    parrot_sample_goal_options_.result_callback = std::bind(
        &MissionCoordinatorNode::sampleROIResultCb,
        this,
        std::placeholders::_1
    );



    // Entrypoint into starting the mission is a service called StartMission
    // TODO (james): Currently it is just a service of type trigger, but in future make it an action as well.
    start_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/start",
        std::bind(
            &MissionCoordinatorNode::startMissionCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // Request cancellation of the current mission
    cancel_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/cancel",
        std::bind(
            &MissionCoordinatorNode::cancelMissionCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // Manual selection only changes command ownership - the robot stacks stay running
    manual_husky_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/manual_husky", std::bind(&MissionCoordinatorNode::manualMissionCb,
            this, Robot::Husky, std::placeholders::_1, std::placeholders::_2));
    manual_parrot_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/manual_parrot", std::bind(&MissionCoordinatorNode::manualMissionCb,
            this, Robot::Parrot, std::placeholders::_1, std::placeholders::_2));
    resume_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/resume", std::bind(&MissionCoordinatorNode::resumeMissionCb,
            this, std::placeholders::_1, std::placeholders::_2));

    // Keep input ownership in ROS so keyboard and controller publishers cannot compete
    keyboard_input_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/input_keyboard", std::bind(&MissionCoordinatorNode::manualInputCb,
            this, "keyboard", std::placeholders::_1, std::placeholders::_2));
    controller_input_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/input_controller", std::bind(&MissionCoordinatorNode::manualInputCb,
            this, "controller", std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Mission coordinator initialised to IDLE");

}

rcl_interfaces::msg::SetParametersResult MissionCoordinatorNode::validateROIParameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto candidate = configuredROI();
    auto candidate_min = configuredROIMin();
    auto candidate_max = configuredROIMax();
    bool updates_roi = false;
    for (const auto &parameter : parameters)
    {
        const auto &name = parameter.get_name();
        if (name != "roi_x" && name != "roi_y" && name != "roi_z" && name != "roi_frame" &&
            name != "roi_x_min" && name != "roi_x_max" && name != "roi_y_min" && name != "roi_y_max") continue;
        updates_roi = true;
        if (mission_state_ != MissionState::IDLE)
        {
            result.successful = false;
            result.reason = "ROI can only be updated while the mission coordinator is IDLE; restart the mission launch after a terminal outcome";
            return result;
        }
        if (name == "roi_frame")
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = false;
                result.reason = "roi_frame must be a string";
                return result;
            }
            candidate.header.frame_id = parameter.as_string();
        }
        else
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                result.successful = false;
                result.reason = name + " must be a double";
                return result;
            }
            if (name == "roi_x") candidate.pose.position.x = parameter.as_double();
            if (name == "roi_y") candidate.pose.position.y = parameter.as_double();
            if (name == "roi_z") candidate.pose.position.z = parameter.as_double();
            if (name == "roi_x_min") candidate_min.x = parameter.as_double();
            if (name == "roi_x_max") candidate_max.x = parameter.as_double();
            if (name == "roi_y_min") candidate_min.y = parameter.as_double();
            if (name == "roi_y_max") candidate_max.y = parameter.as_double();
        }
    }
    if (!updates_roi) return result;
    if (candidate.header.frame_id.empty())
    {
        result.successful = false;
        result.reason = "roi_frame must not be empty";
    }
    else if (!std::isfinite(candidate.pose.position.x) ||
        !std::isfinite(candidate.pose.position.y) ||
        !std::isfinite(candidate.pose.position.z) ||
        !std::isfinite(candidate_min.x) || !std::isfinite(candidate_min.y) ||
        !std::isfinite(candidate_max.x) || !std::isfinite(candidate_max.y))
    {
        result.successful = false;
        result.reason = "ROI coordinates must be finite";
    }
    else if (candidate_min.x >= candidate_max.x || candidate_min.y >= candidate_max.y)
    {
        result.successful = false;
        result.reason = "ROI box must have a positive extent on both axes";
    }
    // This is a pre-commit validation callback. Read committed values in
    // refreshROI(), so a rejected atomic parameter batch has no side effects.
    return result;
}

geometry_msgs::msg::PoseStamped MissionCoordinatorNode::configuredROI() const
{
    geometry_msgs::msg::PoseStamped roi;
    roi.header.frame_id = this->get_parameter("roi_frame").as_string();
    roi.pose.position.x = this->get_parameter("roi_x").as_double();
    roi.pose.position.y = this->get_parameter("roi_y").as_double();
    roi.pose.position.z = this->get_parameter("roi_z").as_double();
    roi.pose.orientation.w = 1.0;
    return roi;
}

geometry_msgs::msg::Point MissionCoordinatorNode::configuredROIMin() const
{
    geometry_msgs::msg::Point point;
    point.x = this->get_parameter("roi_x_min").as_double();
    point.y = this->get_parameter("roi_y_min").as_double();
    point.z = 0.0;
    return point;
}

geometry_msgs::msg::Point MissionCoordinatorNode::configuredROIMax() const
{
    geometry_msgs::msg::Point point;
    point.x = this->get_parameter("roi_x_max").as_double();
    point.y = this->get_parameter("roi_y_max").as_double();
    point.z = 0.0;
    return point;
}

void MissionCoordinatorNode::refreshROI()
{
    const auto configured = configuredROI();
    const auto configured_min = configuredROIMin();
    const auto configured_max = configuredROIMax();
    if (configured.header.frame_id == roi_pose_.header.frame_id &&
        configured.pose.position.x == roi_pose_.pose.position.x &&
        configured.pose.position.y == roi_pose_.pose.position.y &&
        configured.pose.position.z == roi_pose_.pose.position.z &&
        configured_min.x == roi_min_.x && configured_min.y == roi_min_.y &&
        configured_max.x == roi_max_.x && configured_max.y == roi_max_.y) return;
    roi_pose_ = configured;
    roi_min_ = configured_min;
    roi_max_ = configured_max;
    roi_progress_ = 0.0f;
    publishROIMarker();
}

void MissionCoordinatorNode::requireFreshPose(const std::string &robot) const
{
    const auto tf = tf_buffer_->lookupTransform("world", robot + "_base_link", tf2::TimePointZero);
    const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (age < -0.1 || age > 2.0) throw std::runtime_error(robot + " localization TF is stale");
}

void MissionCoordinatorNode::startMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!safetyClear())
    {
        response->success = false;
        response->message = "Software stop active or safety heartbeat missing; reset the stop before enabling motion";
        return;
    }

    response->success = false;

    // Reuse the start checks when retrying the saved ROI after manual assistance
    if (mission_state_ != MissionState::IDLE && mission_state_ != MissionState::MANUAL_ASSISTANCE)
    {
        response->message = "Mission can only start from idle or resume from manual assistance";
        return;
    }
    if (traversalPending())
    {
        response->message = "Previous navigation is still stopping; continue manual assistance and try Resume again shortly";
        return;
    }
    if (mission_state_ == MissionState::MANUAL_ASSISTANCE && returning_home_)
    {
        response->success = startTraverseHome();
        response->message = message_;
        return;
    }
    if (mission_state_ == MissionState::MANUAL_ASSISTANCE && sampling_stage_ != SamplingStage::None)
    {
        resumeSampling(*response);
        return;
    }
    // Synchronize immediately even when Start follows a parameter update before
    // the next status timer. Both operations execute serially in this node.
    refreshROI();
    // Validate supplied ROI
    if (roi_pose_.header.frame_id.empty())
    {
        response->message = "ROI frame_id is empty";
        return;
    }
    if (!std::isfinite(roi_pose_.pose.position.x) ||
        !std::isfinite(roi_pose_.pose.position.y) ||
        !std::isfinite(roi_pose_.pose.position.z))
    {
        response->message = "ROI position contains invalid numbers";
        return;
    }
    const double norm_squared =
        roi_pose_.pose.orientation.x * roi_pose_.pose.orientation.x +
        roi_pose_.pose.orientation.y * roi_pose_.pose.orientation.y +
        roi_pose_.pose.orientation.z * roi_pose_.pose.orientation.z +
        roi_pose_.pose.orientation.w * roi_pose_.pose.orientation.w;

    if (!std::isfinite(norm_squared) ||
        std::abs(norm_squared - 1.0) > 1e-3)
    {
        response->message = "ROI orientation must be a unit quaternion";
        return;
    }

    // Check both action clients are ready
    const bool parrot_ready = traverse_to_roi_client_parrot_->action_server_is_ready();
    const bool husky_ready = traverse_to_roi_client_husky_->action_server_is_ready();

    if (!parrot_ready || !husky_ready)
    {
        response->message = "TraverseToROI action server unavailable:";
        if (!parrot_ready) response->message += " Parrot";
        if (!husky_ready) response->message += " Husky";
        return;
    }

    // A frame label is not a coordinate conversion. Both existing executors
    // accept parrot1_map; Husky transforms the generated path to its own map.
    try
    {
        requireFreshPose("husky1");
        requireFreshPose("parrot1");
        const auto transform = tf_buffer_->lookupTransform("parrot1_map", roi_pose_.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(roi_pose_, executor_roi_, transform);
        saveHomePoses();
    }
    catch (const std::exception &e)
    {
        response->message = std::string("Waiting for world alignment/localization: ") + e.what();
        return;
    }

    // Start with husky so it is ready before parrot generates the path
    response->success = startTraverseToROI();
    response->message = response->success ?
        "Mission started; waiting for Husky to accept the ROI" :
        "Could not send Husky's TraverseToROI goal";
}

void MissionCoordinatorNode::cancelMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = false;
    if (mission_state_ == MissionState::IDLE)
    {
        response->message = "No active mission to cancel";
        return;
    }

    if (mission_state_ == MissionState::COMPLETE || mission_state_ == MissionState::CANCELLED)
    {
        response->message = "Mission already finished; create a new session";
        return;
    }

    // Repeated clicks must not cancel the home journey or resume it behind the operator's back
    if (returning_home_)
    {
        response->success = true;
        response->message = mission_state_ == MissionState::MANUAL_ASSISTANCE ?
            "Return home is paused; select Resume when ready" : "Return home is already requested";
        return;
    }
    survey_cancelled_ = true;
    returning_home_ = true;
    mission_state_ = MissionState::CANCELLING;
    control_mode_ = "hold";
    ++input_revision_;
    message_ = "Stopping the survey before returning both robots home";
    publishStatus();

    // A pending goal will be canceled once its goal handle arrives
    cancelTraversalToROI(Robot::Parrot);
    cancelTraversalToROI(Robot::Husky);
    cancelSampleROI();
    cancelMeasureTrees();
    evaluateTransitions();

    response->success = true;
    response->message = "Cancel and return home requested; keeping confirmed measurements";
}

void MissionCoordinatorNode::traverseToROIGoalResponseCb(Robot robot,
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle)
{
    auto &goal_pending = robot == Robot::Parrot ? parrot_goal_pending_ : husky_goal_pending_;
    auto &current_goal = robot == Robot::Parrot ? parrot_goal_handle_ : husky_goal_handle_;
    const std::string robot_name = robot == Robot::Parrot ? "Parrot" : "Husky";

    // Resolve the pending request, even if the mission is already stopping
    goal_pending = false;
    if (!goal_handle)
    {
        if (mission_state_ == MissionState::TRAVERSINGTOROI || mission_state_ == MissionState::SAMPLINGROI)
        {
            enterManualAssistance(robot, robot_name + " executor rejected the TraverseToROI goal");
        }
        else
        {
            evaluateTransitions();
        }
        return;
    }

    current_goal = goal_handle;
    (robot == Robot::Parrot ? parrot_stage_ : husky_stage_) = "Accepted";

    // Cancellation may have arrived before the executor accepted its goal
    if (mission_state_ == MissionState::MANUAL_ASSISTANCE || mission_state_ == MissionState::CANCELLING)
    {
        cancelTraversalToROI(robot);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "%s accepted the TraverseToROI goal", robot_name.c_str());

    if (robot == Robot::Husky && mission_state_ == MissionState::SAMPLINGROI)
    {
        // The route is back under Husky's control so Parrot can restart its interrupted sweep
        if (sampling_stage_ == SamplingStage::Sweeping) startSampleROI();
        return;
    }

    // Husky is now waiting for a path, so parrot can start its mission
    if (robot == Robot::Husky && mission_state_ == MissionState::TRAVERSINGTOROI)
    {
        if (!traverse_to_roi_client_parrot_->action_server_is_ready())
        {
            enterManualAssistance(Robot::Parrot, "Parrot TraverseToROI server became unavailable");
            return;
        }

        try
        {
            parrot_goal_pending_ = true;
            traverse_to_roi_client_parrot_->async_send_goal(createTraverseToROIGoal(executor_roi_), parrot_goal_options_);
        }
        catch (const std::exception &e)
        {
            parrot_goal_pending_ = false;
            enterManualAssistance(Robot::Parrot, std::string("Could not send Parrot's TraverseToROI goal: ") + e.what());
        }
    }
}

void MissionCoordinatorNode::traverseToROIFeedbackCb(Robot robot,
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle,
    const std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Feedback> feedback)
{
    const auto &current_goal = robot == Robot::Parrot ? parrot_goal_handle_ : husky_goal_handle_;

    // Ignore feedback from old goals or while the mission is stopping
    if (!feedback || !goal_handle || !current_goal ||
        (mission_state_ != MissionState::TRAVERSINGTOROI && mission_state_ != MissionState::SAMPLINGROI) ||
        goal_handle->get_goal_id() != current_goal->get_goal_id()) return;

    (robot == Robot::Parrot ? parrot_stage_ : husky_stage_) = feedback->stage;
    if (robot == Robot::Parrot)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Parrot traversal stage: %s", feedback->stage.c_str());
    }
    else if (robot == Robot::Husky)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Husky traversal stage: %s", feedback->stage.c_str());
    }
}

void MissionCoordinatorNode::traverseToROIResultCb(Robot robot,
    const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::WrappedResult &result)
{
    auto &current_goal = robot == Robot::Parrot ? parrot_goal_handle_ : husky_goal_handle_;
    auto &cancel_requested = robot == Robot::Parrot ? parrot_cancel_requested_ : husky_cancel_requested_;
    const std::string robot_name = robot == Robot::Parrot ? "Parrot" : "Husky";

    // Only accept a result for the goal this mission owns
    if (!current_goal || result.goal_id != current_goal->get_goal_id()) return;
    (robot == Robot::Parrot ? parrot_stage_ : husky_stage_) =
        result.code == rclcpp_action::ResultCode::SUCCEEDED ? "Reached ROI" :
        result.code == rclcpp_action::ResultCode::CANCELED ? "Cancelled" : "Failed";
    current_goal.reset();
    cancel_requested = false;

    if (result.result && !result.result->message.empty())
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED || result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s executor reported: %s", robot_name.c_str(), result.result->message.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "%s executor reported: %s", robot_name.c_str(), result.result->message.c_str());
        }
    }

    // A success received while stopping must not advance the mission
    if (mission_state_ == MissionState::MANUAL_ASSISTANCE || mission_state_ == MissionState::CANCELLING)
    {
        if (mission_state_ == MissionState::MANUAL_ASSISTANCE && sampling_stage_ != SamplingStage::None &&
            robot == Robot::Husky && result.code == rclcpp_action::ResultCode::SUCCEEDED)
            husky_succeded_ = true;
        evaluateTransitions();
        return;
    }

    // Keep the executor's reason visible so the operator knows why assistance was requested
    const std::string detail = result.result && !result.result->message.empty() ?
        ": " + result.result->message : "";

    // Handle each result case for the action: SUCCEEDED, ABORTED, CANCELLED
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (robot == Robot::Parrot)
            {
                parrot_succeded_ = true;
                RCLCPP_INFO(this->get_logger(), "Parrot reached the ROI and handed the ground path to Husky");
            }
            else if (robot == Robot::Husky)
            {
                husky_succeded_ = true;
                RCLCPP_INFO(this->get_logger(), "Husky has reached the ROI");
            }
            evaluateTransitions();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            enterManualAssistance(robot, robot_name + " traversal to ROI aborted" + detail);
            break;

        case rclcpp_action::ResultCode::CANCELED:
            enterManualAssistance(robot, robot_name + " traversal to ROI was canceled" + detail);
            break;

        default:
            enterManualAssistance(robot, robot_name + " TraverseToROI returned an unexpected result code" + detail);
            break;
    }
}

void MissionCoordinatorNode::traverseToROICancelResponseCb(Robot robot,
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle,
    rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::CancelResponse::SharedPtr response)
{
    const auto &current_goal = robot == Robot::Parrot ? parrot_goal_handle_ : husky_goal_handle_;
    auto &cancel_requested = robot == Robot::Parrot ? parrot_cancel_requested_ : husky_cancel_requested_;

    // The executor may have finished before its cancellation response arrived
    if (!goal_handle || !current_goal || goal_handle->get_goal_id() != current_goal->get_goal_id()) return;

    // Keep ownership until the final result, even if cancellation was rejected
    if (!response || response->goals_canceling.empty())
    {
        cancel_requested = false;
        RCLCPP_WARN(this->get_logger(), "%s did not accept cancellation; waiting for its final result",
            robot == Robot::Parrot ? "Parrot" : "Husky");
    }
}

bool MissionCoordinatorNode::startTraverseToROI()
{
    parrot_succeded_ = false;
    husky_succeded_ = false;
    sampling_stage_ = SamplingStage::None;
    mission_state_ = MissionState::TRAVERSINGTOROI;
    message_ = "Parrot will start sweeping once Husky accepts its ground path";
    control_mode_ = "auto";
    ++input_revision_;
    parrot_stage_ = "Waiting";
    husky_stage_ = "Waiting";
    publishStatus();

    return sendHuskyTraversal(false);
}

bool MissionCoordinatorNode::sendHuskyTraversal(bool resume_route)
{
    try
    {
        husky_goal_pending_ = true;
        auto goal = createTraverseToROIGoal(executor_roi_);
        goal.route_id = instance_id_;
        goal.resume_route = resume_route;
        traverse_to_roi_client_husky_->async_send_goal(goal, husky_goal_options_);
    }
    catch (const std::exception &e)
    {
        husky_goal_pending_ = false;
        enterManualAssistance(Robot::Husky, std::string("Could not send Husky's TraverseToROI goal: ") + e.what());
        return false;
    }

    return true;
}

void MissionCoordinatorNode::startSampleROI()
{
    sampling_stage_ = SamplingStage::Sweeping;
    if (!sample_roi_client_parrot_->action_server_is_ready())
    {
        enterManualAssistance(Robot::Parrot, "Parrot SampleROI action server unavailable");
        return;
    }

    rs1_interfaces::action::SampleROI::Goal goal;
    try
    {
        goal = createSampleROIGoal();
    }
    catch (const std::exception &e)
    {
        enterManualAssistance(Robot::Parrot, std::string("Could not build the sampling goal: ") + e.what());
        return;
    }

    mission_state_ = MissionState::SAMPLINGROI;
    roi_progress_ = 0.0f;
    message_ = "Sampling ROI: sweeping the box for trees";
    // Restore autonomous control before dispatching the sweep
    control_mode_ = "auto";
    ++input_revision_;
    publishStatus();

    try
    {
        parrot_sample_goal_pending_ = true;
        sample_roi_client_parrot_->async_send_goal(goal, parrot_sample_goal_options_);
    }
    catch (const std::exception &e)
    {
        parrot_sample_goal_pending_ = false;
        enterManualAssistance(Robot::Parrot, std::string("Could not send Parrot's SampleROI goal: ") + e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting phase 2: sweeping the ROI for trees");
}

rs1_interfaces::action::SampleROI::Goal MissionCoordinatorNode::createSampleROIGoal()
{
    // Transform the entry point and the box into the executor's planning frame.
    const std::string source_frame = roi_pose_.header.frame_id;
    const auto transform = tf_buffer_->lookupTransform(
        "parrot1_map", source_frame, tf2::TimePointZero);

    rs1_interfaces::action::SampleROI::Goal goal;

    geometry_msgs::msg::PoseStamped entry = roi_pose_;
    entry.header.stamp = this->now();
    tf2::doTransform(entry, goal.roi_origin, transform);
    goal.roi_origin.header.stamp = this->now();

    // Transform all four corners, then re-derive an axis-aligned box so the
    // sweep stays valid even if the map is yawed relative to the source frame.
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    const double corners[4][2] = {
        {roi_min_.x, roi_min_.y}, {roi_max_.x, roi_min_.y},
        {roi_max_.x, roi_max_.y}, {roi_min_.x, roi_max_.y}};
    geometry_msgs::msg::PoseStamped corner;
    corner.header.frame_id = source_frame;
    corner.header.stamp = this->now();
    corner.pose.orientation.w = 1.0;
    for (const auto &xy : corners)
    {
        corner.pose.position.x = xy[0];
        corner.pose.position.y = xy[1];
        corner.pose.position.z = roi_pose_.pose.position.z;
        geometry_msgs::msg::PoseStamped out;
        tf2::doTransform(corner, out, transform);
        min_x = std::min(min_x, out.pose.position.x);
        min_y = std::min(min_y, out.pose.position.y);
        max_x = std::max(max_x, out.pose.position.x);
        max_y = std::max(max_y, out.pose.position.y);
    }
    goal.roi_min.x = min_x;
    goal.roi_min.y = min_y;
    goal.roi_min.z = 0.0;
    goal.roi_max.x = max_x;
    goal.roi_max.y = max_y;
    goal.roi_max.z = 0.0;

    goal.search_clearance = static_cast<float>(search_clearance_);
    goal.transit_clearance = static_cast<float>(transit_clearance_);
    goal.swath_spacing = static_cast<float>(swath_spacing_);
    return goal;
}

void MissionCoordinatorNode::sampleROIGoalResponseCb(
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::SampleROI>::SharedPtr goal_handle)
{
    parrot_sample_goal_pending_ = false;
    if (!goal_handle)
    {
        if (mission_state_ == MissionState::SAMPLINGROI)
        {
            enterManualAssistance(Robot::Parrot, "Parrot executor rejected the SampleROI goal");
        }
        else
        {
            evaluateTransitions();
        }
        return;
    }

    parrot_sample_goal_handle_ = goal_handle;
    parrot_stage_ = "Sampling ROI";

    if (mission_state_ != MissionState::SAMPLINGROI)
    {
        cancelSampleROI();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Parrot accepted the SampleROI goal");
}

void MissionCoordinatorNode::sampleROIFeedbackCb(
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::SampleROI>::SharedPtr goal_handle,
    const std::shared_ptr<const rs1_interfaces::action::SampleROI::Feedback> feedback)
{
    if (!feedback || !goal_handle || !parrot_sample_goal_handle_ ||
        mission_state_ != MissionState::SAMPLINGROI ||
        goal_handle->get_goal_id() != parrot_sample_goal_handle_->get_goal_id()) return;

    if (!feedback->stage.empty()) parrot_stage_ = feedback->stage;
    roi_progress_ = std::clamp(feedback->progress, 0.0f, 1.0f);
}

void MissionCoordinatorNode::sampleROIResultCb(
    const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::SampleROI>::WrappedResult & result)
{
    if (!parrot_sample_goal_handle_ || result.goal_id != parrot_sample_goal_handle_->get_goal_id()) return;
    parrot_sample_goal_handle_.reset();
    parrot_sample_cancel_requested_ = false;

    const std::string detail = result.result ? result.result->message : std::string{};
    parrot_stage_ = result.code == rclcpp_action::ResultCode::SUCCEEDED ? "Sampled ROI" :
        result.code == rclcpp_action::ResultCode::CANCELED ? "Cancelled" : "Failed";
    if (!detail.empty())
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED ||
            result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_DEBUG(this->get_logger(), "Parrot sampling reported: %s", detail.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Parrot sampling reported: %s", detail.c_str());
        }
    }

    // Save a completed map if its result crosses the operator's pause request
    const bool paused = mission_state_ == MissionState::MANUAL_ASSISTANCE;
    if (mission_state_ != MissionState::SAMPLINGROI &&
        !(paused && result.code == rclcpp_action::ResultCode::SUCCEEDED))
    {
        evaluateTransitions();
        return;
    }

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (!result.result)
            {
                if (!paused) enterManualAssistance(Robot::Parrot, "Sweep returned no tree map");
                return;
            }
            try
            {
                survey_.tree_map = treesInROI(result.result->tree_map);
            }
            catch (const std::exception &e)
            {
                if (!paused) enterManualAssistance(Robot::Parrot, e.what());
                return;
            }
            survey_.measurements.clear();
            survey_.pending_views.clear();
            roi_progress_ = 1.0f;
            mission_tree_map_pub_->publish(survey_.tree_map);
            survey_pub_->publish(survey_);
            if (survey_.tree_map.trees.empty())
            {
                if (!paused) enterManualAssistance(Robot::Parrot, "Sweep found no trees in the ROI; check coverage before retrying");
                return;
            }
            sampling_stage_ = SamplingStage::MapReady;
            if (!paused) message_ = "Sweep map ready; waiting for Husky to reach the ROI";
            evaluateTransitions();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            enterManualAssistance(Robot::Parrot, "Parrot ROI sampling aborted");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            enterManualAssistance(Robot::Parrot, "Parrot ROI sampling was canceled");
            break;

        default:
            enterManualAssistance(Robot::Parrot, "Parrot SampleROI returned an unexpected result code");
            break;
    }
}

void MissionCoordinatorNode::sampleROICancelResponseCb(
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::SampleROI>::SharedPtr goal_handle,
    rclcpp_action::Client<rs1_interfaces::action::SampleROI>::CancelResponse::SharedPtr response)
{
    if (!goal_handle || !parrot_sample_goal_handle_ ||
        goal_handle->get_goal_id() != parrot_sample_goal_handle_->get_goal_id()) return;

    if (!response || response->goals_canceling.empty())
    {
        parrot_sample_cancel_requested_ = false;
        RCLCPP_WARN(this->get_logger(), "Parrot did not accept sampling cancellation; waiting for its final result");
    }
}

void MissionCoordinatorNode::cancelSampleROI()
{
    if (!parrot_sample_goal_handle_ || parrot_sample_cancel_requested_) return;

    try
    {
        parrot_sample_cancel_requested_ = true;
        sample_roi_client_parrot_->async_cancel_goal(parrot_sample_goal_handle_, std::bind(
            &MissionCoordinatorNode::sampleROICancelResponseCb,
            this, parrot_sample_goal_handle_, std::placeholders::_1));
    }
    catch (const std::exception &e)
    {
        parrot_sample_cancel_requested_ = false;
        RCLCPP_ERROR(this->get_logger(), "Could not request sampling cancellation: %s; waiting for its final result", e.what());
    }
}

rs1_interfaces::msg::TreeMap MissionCoordinatorNode::treesInROI(const rs1_interfaces::msg::TreeMap &map)
{
    // Check the original box since a rotated sweep can pick up trees just outside it
    rs1_interfaces::msg::TreeMap inside;
    inside.header = map.header;
    if (map.trees.empty()) return inside;
    geometry_msgs::msg::TransformStamped transform;
    const bool needs_tf = map.header.frame_id != roi_pose_.header.frame_id;
    if (needs_tf)
        transform = tf_buffer_->lookupTransform(roi_pose_.header.frame_id, map.header.frame_id, tf2::TimePointZero);
    for (const auto &tree : map.trees)
    {
        auto p = tree.location;
        if (needs_tf) tf2::doTransform(tree.location, p, transform);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::runtime_error("Sweep returned a tree with invalid coordinates");
        if (p.x >= roi_min_.x && p.x <= roi_max_.x && p.y >= roi_min_.y && p.y <= roi_max_.y)
            inside.trees.push_back(tree);
    }
    return inside;
}

void MissionCoordinatorNode::startMeasureTrees(bool resume_from_current_pose)
{
    sampling_stage_ = SamplingStage::Measuring;
    if (!measure_client_->action_server_is_ready())
    {
        enterManualAssistance(Robot::Husky, "Husky MeasureTrees server unavailable");
        return;
    }
    mission_state_ = MissionState::SAMPLINGROI;
    control_mode_ = "auto";
    ++input_revision_;
    husky_stage_ = "Planning tree visits";
    message_ = "Husky is measuring the trees in the ROI";
    publishStatus();

    MeasureTrees::Goal goal;
    goal.survey = survey_;
    goal.resume_from_current_pose = resume_from_current_pose;
    rclcpp_action::Client<MeasureTrees>::SendGoalOptions options;
    options.goal_response_callback = [this](MeasureHandle::SharedPtr handle) {
        measure_pending_ = false;
        measure_goal_ = handle;
        if (mission_state_ != MissionState::SAMPLINGROI)
        {
            cancelMeasureTrees();
            evaluateTransitions();
        }
        else if (!handle) enterManualAssistance(Robot::Husky, "Husky rejected the tree survey");
    };
    options.feedback_callback = [this](MeasureHandle::SharedPtr handle, MeasureTrees::Feedback::ConstSharedPtr fb) {
        if (handle == measure_goal_ && mission_state_ == MissionState::SAMPLINGROI)
            husky_stage_ = fb->stage + " (" + std::to_string(fb->measured_count) + "/" +
                std::to_string(fb->total_count) + " measured)";
    };
    options.result_callback = std::bind(&MissionCoordinatorNode::measureResultCb, this, std::placeholders::_1);
    try
    {
        measure_pending_ = true;
        measure_client_->async_send_goal(goal, options);
    }
    catch (const std::exception &e)
    {
        measure_pending_ = false;
        enterManualAssistance(Robot::Husky, e.what());
    }
}

void MissionCoordinatorNode::resumeSampling(std_srvs::srv::Trigger::Response &response)
{
    if (traversalPending())
    {
        response.success = false;
        response.message = "Previous action is still stopping";
        return;
    }
    try
    {
        if (!husky_succeded_ || sampling_stage_ != SamplingStage::Sweeping) requireFreshPose("husky1");
        if (sampling_stage_ == SamplingStage::Sweeping) requireFreshPose("parrot1");
        if (!husky_succeded_ && !traverse_to_roi_client_husky_->action_server_is_ready())
            throw std::runtime_error("Husky TraverseToROI server unavailable");
    }
    catch (const std::exception &e)
    {
        response.success = false;
        response.message = e.what();
        return;
    }
    mission_state_ = MissionState::SAMPLINGROI;
    control_mode_ = "auto";
    ++input_revision_;
    if (!husky_succeded_)
    {
        message_ = "Resuming Husky's ground path";
        publishStatus();
        sendHuskyTraversal(true);
    }
    else if (sampling_stage_ == SamplingStage::Sweeping) startSampleROI();
    else startMeasureTrees(true);
    response.success = mission_state_ == MissionState::SAMPLINGROI;
    response.message = message_;
}

void MissionCoordinatorNode::cancelMeasureTrees()
{
    if (!measure_goal_ || measure_cancel_requested_) return;
    try
    {
        measure_cancel_requested_ = true;
        measure_client_->async_cancel_goal(measure_goal_, [this, goal = measure_goal_](auto reply) {
            if (goal == measure_goal_ && (!reply || reply->goals_canceling.empty()))
                measure_cancel_requested_ = false;
        });
    }
    catch (const std::exception &e)
    {
        measure_cancel_requested_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting to cancel the tree survey: %s", e.what());
    }
}

void MissionCoordinatorNode::measureResultCb(const MeasureHandle::WrappedResult &result)
{
    if (!measure_goal_ || result.goal_id != measure_goal_->get_goal_id()) return;
    measure_goal_.reset();
    measure_cancel_requested_ = false;

    // Save confirmed readings even if the operator stopped Husky halfway through
    const auto count = survey_.tree_map.trees.size();
    const bool valid = result.result && validSurveyProgress(count, result.result->measurements, result.result->pending_views) &&
        std::all_of(survey_.measurements.begin(), survey_.measurements.end(), [&](const auto &previous) {
            const auto &readings = result.result->measurements;
            return std::find(readings.begin(), readings.end(), previous) != readings.end();
        }) && std::all_of(survey_.pending_views.begin(), survey_.pending_views.end(), [&](const auto &previous) {
            const auto &readings = result.result->measurements;
            const auto &pending = result.result->pending_views;
            return std::any_of(readings.begin(), readings.end(), [&](const auto &reading) {
                return reading.tree_index == previous.tree_index;
            }) || std::find(pending.begin(), pending.end(), previous) != pending.end();
        });
    if (valid)
    {
        survey_.measurements = result.result->measurements;
        survey_.pending_views = result.result->pending_views;
        survey_pub_->publish(survey_);
    }
    if (mission_state_ != MissionState::SAMPLINGROI)
    {
        evaluateTransitions();
        return;
    }
    if (!valid || result.code != rclcpp_action::ResultCode::SUCCEEDED || survey_.measurements.size() != count)
    {
        enterManualAssistance(Robot::Husky, valid && !result.result->message.empty() ?
            result.result->message : "Husky returned an incomplete tree survey");
        return;
    }
    husky_stage_ = "Measured all " + std::to_string(count) + " trees";
    message_ = "Phase 2 complete: " + husky_stage_;
    startTraverseHome();
}

void MissionCoordinatorNode::saveHomePoses()
{
    if (homes_saved_) return;
    std::array<geometry_msgs::msg::PoseStamped, 2> homes;
    for (std::size_t i = 0; i < homes.size(); ++i)
    {
        const std::string robot = i == 0 ? "parrot" : "husky";
        auto &pose = homes[i];
        pose.header.frame_id = "world";
        pose.header.stamp = now();
        double yaw;
        if (get_parameter("home_from_start").as_bool())
        {
            const auto tf = tf_buffer_->lookupTransform("world", robot + "1_base_link", tf2::TimePointZero);
            pose.pose.position.x = tf.transform.translation.x;
            pose.pose.position.y = tf.transform.translation.y;
            pose.pose.position.z = tf.transform.translation.z;
            yaw = tf2::getYaw(tf.transform.rotation);
        }
        else
        {
            const auto values = get_parameter(robot + "_home").as_double_array();
            if (values.size() != 4) throw std::runtime_error(robot + "_home needs [x, y, z, yaw]");
            pose.pose.position.x = values[0];
            pose.pose.position.y = values[1];
            pose.pose.position.z = values[2];
            yaw = values[3];
        }
        if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
            !std::isfinite(pose.pose.position.z) || !std::isfinite(yaw))
            throw std::runtime_error(robot + " home must be finite");
        pose.pose.orientation.z = std::sin(yaw / 2);
        pose.pose.orientation.w = std::cos(yaw / 2);
    }
    for (std::size_t i = 0; i < homes.size(); ++i) home_returns_[i].pose = homes[i];
    homes_saved_ = true;
}

bool MissionCoordinatorNode::startTraverseHome()
{
    // Keep this flag through assistance so Resume cannot restart tree sampling
    returning_home_ = true;
    if (traversalPending())
    {
        message_ = "Previous navigation is still stopping; try Resume again shortly";
        return false;
    }
    if (!safetyClear())
    {
        enterManualAssistance(Robot::Husky, "Return home paused: software stop active or safety heartbeat missing");
        return false;
    }
    for (std::size_t i = 0; i < home_returns_.size(); ++i)
    {
        auto &home = home_returns_[i];
        try
        {
            if (!homes_saved_) throw std::runtime_error("Mission start poses were not saved");
            const std::string robot = i == 0 ? "parrot1" : "husky1";
            requireFreshPose(robot);
            // The operator may have driven an already-arrived robot away during assistance
            const auto tf = tf_buffer_->lookupTransform("world", robot + "_base_link", tf2::TimePointZero);
            if (std::hypot(tf.transform.translation.x - home.pose.pose.position.x,
                           tf.transform.translation.y - home.pose.pose.position.y) > home_tolerance_)
                home.arrived = false;
            if (home.arrived) continue;
            if (!home.client->action_server_is_ready()) throw std::runtime_error("ReturnHome server is unavailable");
        }
        catch (const std::exception &e)
        {
            enterManualAssistance(static_cast<Robot>(i), e.what());
            return false;
        }
    }
    mission_state_ = MissionState::TRAVERSINGTOHOME;
    control_mode_ = "auto";
    ++input_revision_;
    message_ = survey_cancelled_ ? "Survey cancelled; returning both robots to their start poses" :
        "Phase 3: returning both robots to their start poses";
    publishStatus();
    for (std::size_t i = 0; i < home_returns_.size(); ++i)
    {
        auto &home = home_returns_[i];
        if (home.arrived) continue;
        const auto robot = static_cast<Robot>(i);
        rclcpp_action::Client<ReturnHome>::SendGoalOptions options;
        options.goal_response_callback = [this, i, robot](auto goal) {
            auto &current = home_returns_[i];
            current.pending = false;
            current.goal = goal;
            if (mission_state_ != MissionState::TRAVERSINGTOHOME)
            {
                cancelReturnHome();
                evaluateTransitions();
            }
            else if (!goal) enterManualAssistance(robot, "ReturnHome goal was rejected");
        };
        options.feedback_callback = [this, i](auto goal, auto feedback) {
            if (mission_state_ == MissionState::TRAVERSINGTOHOME && home_returns_[i].goal == goal)
                (i == 0 ? parrot_stage_ : husky_stage_) = feedback->stage;
        };
        options.result_callback = [this, i, robot](const auto &result) {
            auto &current = home_returns_[i];
            if (!current.goal || result.goal_id != current.goal->get_goal_id()) return;
            current.goal.reset();
            current.cancel_requested = false;
            // Preserve an arrival even when it crosses the operator's pause request
            current.arrived = result.code == rclcpp_action::ResultCode::SUCCEEDED;
            (i == 0 ? parrot_stage_ : husky_stage_) = current.arrived ? "Reached home" : "Return interrupted";
            if (mission_state_ == MissionState::TRAVERSINGTOHOME && !current.arrived)
                enterManualAssistance(robot, result.result ? result.result->message : "Return home failed");
            else evaluateTransitions();
        };
        try
        {
            home.pending = true;
            (i == 0 ? parrot_stage_ : husky_stage_) = "Returning home";
            ReturnHome::Goal goal;
            goal.home = home.pose;
            goal.home.header.stamp = now();
            home.client->async_send_goal(goal, options);
        }
        catch (const std::exception &e)
        {
            home.pending = false;
            enterManualAssistance(robot, e.what());
            return false;
        }
    }
    evaluateTransitions();
    return true;
}

void MissionCoordinatorNode::cancelReturnHome()
{
    for (std::size_t i = 0; i < home_returns_.size(); ++i)
    {
        auto &home = home_returns_[i];
        if (!home.goal || home.cancel_requested) continue;
        auto goal = home.goal;
        home.cancel_requested = true;
        try
        {
            home.client->async_cancel_goal(goal, [this, i, goal](auto response) {
                auto &current = home_returns_[i];
                if (current.goal == goal && (!response || response->goals_canceling.empty()))
                    current.cancel_requested = false;
            });
        }
        catch (const std::exception &) { home.cancel_requested = false; }
    }
}

rs1_interfaces::action::TraverseToROI::Goal MissionCoordinatorNode::createTraverseToROIGoal(const geometry_msgs::msg::PoseStamped &roi_pose_)
{
    // Create the TraverseToROI goal which just needs the roi pose
    rs1_interfaces::action::TraverseToROI::Goal goal;
    goal.target_roi = roi_pose_;
    goal.target_roi.header.stamp = this->now(); // Update timestamp to moment of goal creation

    return goal;
}

void MissionCoordinatorNode::evaluateTransitions()
{
    if (mission_state_ == MissionState::TRAVERSINGTOHOME && !traversalPending() &&
        home_returns_[0].arrived && home_returns_[1].arrived)
    {
        mission_state_ = survey_cancelled_ ? MissionState::CANCELLED : MissionState::COMPLETE;
        message_ = survey_cancelled_ ? "Mission cancelled: both robots returned home; confirmed measurements preserved" :
            "Mission complete: both robots returned home";
        finalizeMission();
        return;
    }
    if (mission_state_ == MissionState::TRAVERSINGTOROI && parrot_succeded_)
    {
        startSampleROI();
        return;
    }
    if (mission_state_ == MissionState::SAMPLINGROI && sampling_stage_ == SamplingStage::MapReady && husky_succeded_)
    {
        startMeasureTrees();
        return;
    }

    // Manual assistance keeps the mission alive even after both old attempts have stopped
    if (mission_state_ == MissionState::CANCELLING && !traversalPending())
    {
        // Final action results have saved any completed DBHs and released navigation ownership
        startTraverseHome();
    }
}

void MissionCoordinatorNode::publishROIMarker()
{
    // Do not send invalid coordinates to RViz
    if (!std::isfinite(roi_pose_.pose.position.x) ||
        !std::isfinite(roi_pose_.pose.position.y) ||
        !std::isfinite(roi_pose_.pose.position.z))
    {
        RCLCPP_WARN(this->get_logger(), "ROI marker not published: position contains invalid numbers");
        return;
    }

    // Mark the target point in the same frame used by both mission goals
    visualization_msgs::msg::Marker roi_marker;
    roi_marker.header = roi_pose_.header;
    roi_marker.header.stamp = rclcpp::Time(0); // Use the latest transform
    roi_marker.ns = "mission_roi";
    roi_marker.id = 0;
    roi_marker.type = visualization_msgs::msg::Marker::SPHERE;
    roi_marker.action = visualization_msgs::msg::Marker::ADD;
    roi_marker.pose = roi_pose_.pose;
    roi_marker.scale.x = 1.5;
    roi_marker.scale.y = 1.5;
    roi_marker.scale.z = 1.5;
    roi_marker.color.r = 0.0;
    roi_marker.color.g = 1.0;
    roi_marker.color.b = 1.0;
    roi_marker.color.a = 1.0;
    roi_marker.lifetime = rclcpp::Duration::from_seconds(0.0); // Keep it until replaced or deleted
    roi_marker.frame_locked = true; // Stay attached to the ROI frame as TF updates

    // Put a label above the target so it is easy to identify
    visualization_msgs::msg::Marker roi_label = roi_marker;
    roi_label.id = 1;
    roi_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    roi_label.pose.position.z += 2.0;
    roi_label.scale.z = 0.5;
    roi_label.text = "ROI";

    // Outline the ROI box as a rectangle in the entry point's plane
    visualization_msgs::msg::Marker roi_box;
    roi_box.header = roi_pose_.header;
    roi_box.header.stamp = rclcpp::Time(0);
    roi_box.ns = "mission_roi";
    roi_box.id = 2;
    roi_box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    roi_box.action = visualization_msgs::msg::Marker::ADD;
    roi_box.pose.orientation.w = 1.0;
    roi_box.scale.x = 0.4;
    roi_box.color.r = 0.0;
    roi_box.color.g = 1.0;
    roi_box.color.b = 1.0;
    roi_box.color.a = 0.8;
    roi_box.lifetime = rclcpp::Duration::from_seconds(0.0);
    roi_box.frame_locked = true;
    geometry_msgs::msg::Point corner;
    corner.z = roi_pose_.pose.position.z;
    corner.x = roi_min_.x; corner.y = roi_min_.y; roi_box.points.push_back(corner);
    corner.x = roi_max_.x; corner.y = roi_min_.y; roi_box.points.push_back(corner);
    corner.x = roi_max_.x; corner.y = roi_max_.y; roi_box.points.push_back(corner);
    corner.x = roi_min_.x; corner.y = roi_max_.y; roi_box.points.push_back(corner);
    corner.x = roi_min_.x; corner.y = roi_min_.y; roi_box.points.push_back(corner);

    // Store the point, label and box together so late subscribers receive all
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(roi_marker);
    markers.markers.push_back(roi_label);
    markers.markers.push_back(roi_box);
    roi_marker_pub_->publish(markers);
}

bool MissionCoordinatorNode::traversalPending() const
{
    for (const auto &home : home_returns_) if (home.pending || home.goal) return true;
    return parrot_goal_pending_ || husky_goal_pending_ || parrot_goal_handle_ || husky_goal_handle_ ||
        parrot_sample_goal_pending_ || parrot_sample_goal_handle_ || measure_pending_ || measure_goal_;
}

void MissionCoordinatorNode::enterManualAssistance(Robot robot, const std::string &message)
{
    mission_state_ = MissionState::MANUAL_ASSISTANCE;
    control_mode_ = safetyClear() ? (robot == Robot::Husky ? "husky1" : "parrot1") : "hold";
    // A quick retry followed by another failure must not reuse a held controller input
    ++input_revision_;
    message_ = control_mode_ == "hold" ? message + "; motion held, reset the software stop then select Resume or manual assistance" :
        message + ". Manual assistance selected for " + control_mode_ + "; reposition the robot, then select Resume";
    RCLCPP_WARN(this->get_logger(), "%s", message_.c_str());

    // Gate autonomous velocities immediately, then cancel the old attempts without losing the ROI
    publishStatus();

    cancelTraversalToROI(Robot::Parrot);
    cancelTraversalToROI(Robot::Husky);
    cancelSampleROI();
    cancelMeasureTrees();
    cancelReturnHome();
    evaluateTransitions();
}

void MissionCoordinatorNode::manualMissionCb(Robot robot,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!safetyClear())
    {
        response->success = false;
        response->message = "Software stop active or safety heartbeat missing; reset the stop before enabling motion";
        return;
    }

    response->success = false;
    if (mission_state_ != MissionState::TRAVERSINGTOROI &&
        mission_state_ != MissionState::TRAVERSINGTOHOME &&
        mission_state_ != MissionState::SAMPLINGROI &&
        mission_state_ != MissionState::MANUAL_ASSISTANCE)
    {
        response->message = "No active mission to assist";
        return;
    }
    enterManualAssistance(robot, "Manual control requested by operator");
    response->success = true;
    response->message = message_;
}

void MissionCoordinatorNode::resumeMissionCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!safetyClear())
    {
        response->success = false;
        response->message = "Software stop active or safety heartbeat missing; reset the stop before enabling motion";
        return;
    }

    if (mission_state_ != MissionState::MANUAL_ASSISTANCE)
    {
        response->success = false;
        response->message = "Mission is not in manual assistance";
        return;
    }

    // Pick up the same part of phase 2 after the old goal has stopped
    if (!returning_home_ && sampling_stage_ != SamplingStage::None)
    {
        resumeSampling(*response);
        return;
    }

    // Failed readiness checks leave manual mode selected and explain why Resume was rejected
    startMissionCb(request, response);
}

void MissionCoordinatorNode::manualInputCb(const std::string &input,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!safetyClear())
    {
        response->success = false;
        response->message = "Software stop active or safety heartbeat missing; reset the stop before enabling motion";
        return;
    }

    response->success = false;
    if (mission_state_ != MissionState::MANUAL_ASSISTANCE)
    {
        response->message = "Select manual assistance before choosing an input";
        return;
    }

    manual_input_ = input;
    ++input_revision_;
    publishStatus();
    response->success = true;
    response->message = input + " selected; release the enable button before driving";
}

void MissionCoordinatorNode::cancelTraversalToROI(Robot robot)
{
    const auto &goal_handle = robot == Robot::Parrot ? parrot_goal_handle_ : husky_goal_handle_;
    const auto &client = robot == Robot::Parrot ? traverse_to_roi_client_parrot_ : traverse_to_roi_client_husky_;
    auto &cancel_requested = robot == Robot::Parrot ? parrot_cancel_requested_ : husky_cancel_requested_;

    // A pending goal has no handle yet; its response callback will cancel it
    if (!goal_handle || cancel_requested) return;

    try
    {
        cancel_requested = true;
        client->async_cancel_goal(goal_handle, std::bind(
            &MissionCoordinatorNode::traverseToROICancelResponseCb,
            this, robot, goal_handle, std::placeholders::_1));
    }
    catch (const std::exception &e)
    {
        cancel_requested = false;
        RCLCPP_ERROR(this->get_logger(), "Could not request %s cancellation: %s; waiting for its final result",
            robot == Robot::Parrot ? "Parrot" : "Husky", e.what());
    }
}

void MissionCoordinatorNode::finalizeMission()
{
    // Clear goal ownership once both executors have finished; keep the outcome.
    parrot_goal_handle_.reset();
    husky_goal_handle_.reset();
    parrot_goal_pending_ = false;
    husky_goal_pending_ = false;
    parrot_cancel_requested_ = false;
    husky_cancel_requested_ = false;
    parrot_succeded_ = false;
    husky_succeded_ = false;
    parrot_sample_goal_handle_.reset();
    parrot_sample_goal_pending_ = false;
    parrot_sample_cancel_requested_ = false;
    sampling_stage_ = SamplingStage::None;
    // Preserve the terminal outcome and stop all command sources until a new session
    control_mode_ = "hold";
    publishStatus();
}

bool MissionCoordinatorNode::safetyClear() const
{
    return !safety_.instance_id.empty() && !safety_.husky_stopped && !safety_.parrot_stopped &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - safety_received_).count() < 0.5;
}

void MissionCoordinatorNode::checkSafety()
{
    if (!safetyClear() && (mission_state_ == MissionState::TRAVERSINGTOROI ||
        mission_state_ == MissionState::SAMPLINGROI || mission_state_ == MissionState::TRAVERSINGTOHOME ||
        (mission_state_ == MissionState::MANUAL_ASSISTANCE && control_mode_ != "hold")))
    {
        // Keep phase progress and samples, but do not grant teleop authority during a stop
        enterManualAssistance(Robot::Husky, "Software stop active or safety heartbeat missing");
    }
}

void MissionCoordinatorNode::publishStatus()
{
    checkSafety();
    if (mission_state_ == MissionState::CANCELLING || mission_state_ == MissionState::MANUAL_ASSISTANCE)
    {
        cancelTraversalToROI(Robot::Husky);
        cancelTraversalToROI(Robot::Parrot);
        cancelSampleROI();
        cancelMeasureTrees();
        cancelReturnHome();
    }
    refreshROI();
    rs1_interfaces::msg::MissionStatus status;
    status.header.stamp = this->now();
    status.header.frame_id = "world";
    status.session_id = session_id_;
    status.instance_id = instance_id_;
    status.roi = roi_pose_;
    status.roi.header.stamp = status.header.stamp;
    status.roi_min = roi_min_;
    status.roi_max = roi_max_;
    status.roi_progress = roi_progress_;
    switch (mission_state_)
    {
        case MissionState::IDLE: status.state = "ready"; break;
        case MissionState::TRAVERSINGTOROI: status.state = "executing"; break;
        case MissionState::SAMPLINGROI: status.state = "sampling"; break;
        case MissionState::TRAVERSINGTOHOME: status.state = "returning"; break;
        case MissionState::CANCELLING: status.state = "cancellation_requested"; break;
        case MissionState::CANCELLED: status.state = "cancelled"; break;
        case MissionState::COMPLETE: status.state = "complete"; break;
        case MissionState::MANUAL_ASSISTANCE: status.state = "manual_assistance"; break;
        default: status.state = "unsupported_phase";
    }
    status.parrot_stage = parrot_stage_;
    status.husky_stage = husky_stage_;
    status.message = message_;
    status.control_mode = control_mode_;
    status.manual_input = manual_input_;
    status.input_revision = input_revision_;
    status.safety_instance_id = safety_.instance_id;
    status.safety_revision = safety_.revision;
    status.cleanup_complete = !traversalPending();
    status.returning_home = returning_home_;
    status_pub_->publish(status);
}

}  // namespace rs1_mission
