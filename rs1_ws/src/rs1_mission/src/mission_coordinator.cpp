// This node handles the overall mission state machine.
// We call a service or action to start the mission.
// Starting the mission causes the mission state to go from IDLE to phase 1. Get to ROI
// When in phase 1, it calls action servers in both the parrot and husky mission executors to execute the TraverseToROI action.
// It receives updates on the status of both robots and their current task statuses, and updates the overall mission status
// It should go something like from:
// 1. Get to ROI
// 2. Sample trees
// 3. Get to home
// Once both robots are at home base (ground station), we finish the mission

#include "mission_coordinator.hpp"

#include <exception>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>

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
    roi_pose_.pose.position.x = this->declare_parameter<double>("roi_x", -40.0);
    roi_pose_.pose.position.y = this->declare_parameter<double>("roi_y", 40.0);
    roi_pose_.pose.position.z = this->declare_parameter<double>("roi_z", 0.0);
    roi_pose_.pose.orientation.w = 1.0;

    // Keep the latest ROI marker for RViz instances opened later
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mission/roi_marker", rclcpp::QoS(1).reliable().transient_local());

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

    RCLCPP_INFO(this->get_logger(), "Mission coordinator initialised to IDLE");

}

rcl_interfaces::msg::SetParametersResult MissionCoordinatorNode::validateROIParameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto candidate = configuredROI();
    bool updates_roi = false;
    for (const auto &parameter : parameters)
    {
        const auto &name = parameter.get_name();
        if (name != "roi_x" && name != "roi_y" && name != "roi_z" && name != "roi_frame") continue;
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
        !std::isfinite(candidate.pose.position.z))
    {
        result.successful = false;
        result.reason = "ROI coordinates must be finite";
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

void MissionCoordinatorNode::refreshROI()
{
    const auto configured = configuredROI();
    if (configured.header.frame_id == roi_pose_.header.frame_id &&
        configured.pose.position.x == roi_pose_.pose.position.x &&
        configured.pose.position.y == roi_pose_.pose.position.y &&
        configured.pose.position.z == roi_pose_.pose.position.z) return;
    roi_pose_ = configured;
    publishROIMarker();
}

void MissionCoordinatorNode::startMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
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
        for (const auto &robot : {"husky1", "parrot1"})
        {
            const auto t = tf_buffer_->lookupTransform("world", std::string(robot) + "_base_link", tf2::TimePointZero);
            const double age = (this->now() - rclcpp::Time(t.header.stamp)).seconds();
            if (age < -0.1 || age > 2.0) throw std::runtime_error("Robot localization TF is stale");
        }
        const auto transform = tf_buffer_->lookupTransform("parrot1_map", roi_pose_.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(roi_pose_, executor_roi_, transform);
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
    mission_state_ = MissionState::CANCELLING;
    control_mode_ = "hold";
    message_ = "Cancellation requested; waiting for both executors";
    publishStatus();

    // A pending goal will be canceled once its goal handle arrives
    cancelTraversalToROI(Robot::Parrot);
    cancelTraversalToROI(Robot::Husky);
    evaluateTransitions();

    response->success = true;
    response->message = "Cancellation requested; waiting for both executors to finish";
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
        if (mission_state_ == MissionState::TRAVERSINGTOROI)
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
    if (!feedback || !goal_handle || !current_goal || mission_state_ != MissionState::TRAVERSINGTOROI ||
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
    mission_state_ = MissionState::TRAVERSINGTOROI;
    message_ = "Traversal to ROI; sampling and return-home are not implemented";
    control_mode_ = "auto";
    parrot_stage_ = "Waiting";
    husky_stage_ = "Waiting";
    publishStatus();

    // Send husky its goal first; its acceptance callback starts parrot
    try
    {
        husky_goal_pending_ = true;
        traverse_to_roi_client_husky_->async_send_goal(createTraverseToROIGoal(executor_roi_), husky_goal_options_);
    }
    catch (const std::exception &e)
    {
        husky_goal_pending_ = false;
        enterManualAssistance(Robot::Husky, std::string("Could not send Husky's TraverseToROI goal: ") + e.what());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting phase 1; waiting for Husky to accept the ROI");
    return true;
}

void MissionCoordinatorNode::startSampleROI()
{
    return; // TODO (james): Implement phase 2
}

void MissionCoordinatorNode::startTraverseHome()
{
    return; // TODO (james): Implement phase 3
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
    // Parrot finishes after handoff, but husky must also reach the ROI
    if (mission_state_ == MissionState::TRAVERSINGTOROI && parrot_succeded_ && husky_succeded_)
    {
        mission_state_ = MissionState::COMPLETE; // TODO (james): Change this to the next phase, and then call startSampleROI()
        RCLCPP_INFO(this->get_logger(), "Phase 1 complete: both robots reached the ROI");
        message_ = "Traversal complete: both robots reached ROI";
        finalizeMission();
        return;
    }

    // Manual assistance keeps the mission alive even after both old attempts have stopped
    if (mission_state_ == MissionState::CANCELLING && !traversalPending())
    {
        mission_state_ = MissionState::CANCELLED;
        message_ = "Cancellation completed; both executors finished";
        finalizeMission();
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

    // Store the point and label together so late subscribers receive both
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(roi_marker);
    markers.markers.push_back(roi_label);
    roi_marker_pub_->publish(markers);
}

bool MissionCoordinatorNode::traversalPending() const
{
    return parrot_goal_pending_ || husky_goal_pending_ || parrot_goal_handle_ || husky_goal_handle_;
}

void MissionCoordinatorNode::enterManualAssistance(Robot robot, const std::string &message)
{
    mission_state_ = MissionState::MANUAL_ASSISTANCE;
    control_mode_ = robot == Robot::Husky ? "husky1" : "parrot1";
    message_ = message + ". Manual assistance selected for " + control_mode_ +
        "; reposition the robot, then select Resume";
    RCLCPP_WARN(this->get_logger(), "%s", message_.c_str());

    // Gate autonomous velocities immediately, then cancel the old attempts without losing the ROI
    publishStatus();

    cancelTraversalToROI(Robot::Parrot);
    cancelTraversalToROI(Robot::Husky);
}

void MissionCoordinatorNode::manualMissionCb(Robot robot,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = false;
    if (mission_state_ != MissionState::TRAVERSINGTOROI && mission_state_ != MissionState::MANUAL_ASSISTANCE)
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
    if (mission_state_ != MissionState::MANUAL_ASSISTANCE)
    {
        response->success = false;
        response->message = "Mission is not in manual assistance";
        return;
    }

    // Failed readiness checks leave manual mode selected and explain why Resume was rejected
    startMissionCb(request, response);
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
    // Preserve the terminal outcome and stop all command sources until a new session
    control_mode_ = "hold";
    publishStatus();
}

void MissionCoordinatorNode::publishStatus()
{
    refreshROI();
    rs1_interfaces::msg::MissionStatus status;
    status.header.stamp = this->now();
    status.header.frame_id = "world";
    status.session_id = session_id_;
    status.instance_id = instance_id_;
    status.roi = roi_pose_;
    status.roi.header.stamp = status.header.stamp;
    switch (mission_state_)
    {
        case MissionState::IDLE: status.state = "ready"; break;
        case MissionState::TRAVERSINGTOROI: status.state = "executing"; break;
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
    status.cleanup_complete = !traversalPending();
    status_pub_->publish(status);
}

}  // namespace rs1_mission
