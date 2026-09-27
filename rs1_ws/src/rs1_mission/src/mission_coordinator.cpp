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

    // Define ROI - both robots receive the same pose in parrot frame
    roi_pose_.header.frame_id = "parrot1_map";
    roi_pose_.pose.position.x = this->declare_parameter<double>("roi_x", -40.0);
    roi_pose_.pose.position.y = this->declare_parameter<double>("roi_y", 40.0);
    roi_pose_.pose.position.z = this->declare_parameter<double>("roi_z", 0.0);
    roi_pose_.pose.orientation.w = 1.0;

    // Keep the latest ROI marker for RViz instances opened later
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mission/roi_marker", rclcpp::QoS(1).reliable().transient_local());

    // Show the configured ROI before starting the mission
    publishROIMarker();


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

    RCLCPP_INFO(this->get_logger(), "Mission coordinator initialised to IDLE");

}

void MissionCoordinatorNode::startMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = false;

    // Check if the current mission state is IDLE. Reject service request if not.
    if (mission_state_ != MissionState::IDLE)
    {
        response->message = "Mission can only start from IDLE state";
        return;
    }
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

    // Keep a failed mission marked as failed while retrying its cleanup
    if (mission_state_ != MissionState::FAILED)
    {
        mission_state_ = MissionState::CANCELLING;
    }

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
            failMission(robot_name + " executor rejected the TraverseToROI goal");
        }
        else
        {
            evaluateTransitions();
        }
        return;
    }

    current_goal = goal_handle;

    // Cancellation may have arrived before the executor accepted its goal
    if (mission_state_ == MissionState::FAILED || mission_state_ == MissionState::CANCELLING)
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
            failMission("Parrot TraverseToROI server became unavailable");
            return;
        }

        try
        {
            parrot_goal_pending_ = true;
            traverse_to_roi_client_parrot_->async_send_goal(createTraverseToROIGoal(roi_pose_), parrot_goal_options_);
        }
        catch (const std::exception &e)
        {
            parrot_goal_pending_ = false;
            failMission(std::string("Could not send Parrot's TraverseToROI goal: ") + e.what());
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
    if (mission_state_ == MissionState::FAILED || mission_state_ == MissionState::CANCELLING)
    {
        evaluateTransitions();
        return;
    }

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
            failMission(robot_name + " traversal to ROI aborted");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            failMission(robot_name + " traversal to ROI was canceled");
            break;

        default:
            failMission(robot_name + " TraverseToROI returned an unexpected result code");
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

    // Send husky its goal first; its acceptance callback starts parrot
    try
    {
        husky_goal_pending_ = true;
        traverse_to_roi_client_husky_->async_send_goal(createTraverseToROIGoal(roi_pose_), husky_goal_options_);
    }
    catch (const std::exception &e)
    {
        husky_goal_pending_ = false;
        failMission(std::string("Could not send Husky's TraverseToROI goal: ") + e.what());
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
        returnToIdle();
        return;
    }

    // Do not allow a new mission while a goal or cancellation is still pending
    if ((mission_state_ == MissionState::FAILED || mission_state_ == MissionState::CANCELLING) &&
        !parrot_goal_pending_ && !husky_goal_pending_ && !parrot_goal_handle_ && !husky_goal_handle_)
    {
        RCLCPP_INFO(this->get_logger(), "%s; both executors have finished",
            mission_state_ == MissionState::FAILED ? "Mission failed" : "Mission canceled");
        returnToIdle();
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

void MissionCoordinatorNode::failMission(const std::string &message)
{
    mission_state_ = MissionState::FAILED;
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());

    // Stop the other robot too, then wait for both final results
    cancelTraversalToROI(Robot::Parrot);
    cancelTraversalToROI(Robot::Husky);
    evaluateTransitions();
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

void MissionCoordinatorNode::returnToIdle()
{
    // Reset goal handles, flags and mission state once both executors have finished
    parrot_goal_handle_.reset();
    husky_goal_handle_.reset();
    parrot_goal_pending_ = false;
    husky_goal_pending_ = false;
    parrot_cancel_requested_ = false;
    husky_cancel_requested_ = false;
    parrot_succeded_ = false;
    husky_succeded_ = false;
    mission_state_ = MissionState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Mission coordinator returned to IDLE");
}

}  // namespace rs1_mission
