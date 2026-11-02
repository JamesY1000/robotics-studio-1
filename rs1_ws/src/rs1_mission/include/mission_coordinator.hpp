#ifndef RS1_MISSION__MISSION_COORDINATOR_HPP_
#define RS1_MISSION__MISSION_COORDINATOR_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "rs1_interfaces/msg/mission_status.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rs1_interfaces/action/traverse_to_roi.hpp"


namespace rs1_mission
{

class MissionCoordinatorNode : public rclcpp::Node
{
public:
  MissionCoordinatorNode();

private:

    // State machine
    enum class MissionState
    {
        IDLE,
        TRAVERSINGTOROI, // Phase 1
        SAMPLINGROI, // Phase 2
        TRAVERSINGTOHOME, // Phase 3
        CANCELLING,
        COMPLETE,
        CANCELLED,
        MANUAL_ASSISTANCE
    };

    enum class Robot
    {
        Parrot,
        Husky
    };

    // Start mission callback
    void startMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Cancel mission callback
    void cancelMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Select the robot to assist, then retry the same ROI when the operator resumes
    void manualMissionCb(Robot robot,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void resumeMissionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Action callbacks
    void traverseToROIGoalResponseCb(Robot robot,
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle);

    void traverseToROIFeedbackCb(Robot robot,
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle,
        const std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Feedback> feedback);

    void traverseToROIResultCb(Robot robot,
        const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::WrappedResult & result);


    void traverseToROICancelResponseCb(Robot robot,
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr goal_handle,
        rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::CancelResponse::SharedPtr response);

    // Entrypoints to starting each phase
    bool startTraverseToROI();
    void startSampleROI();
    void startTraverseHome();

    rs1_interfaces::action::TraverseToROI::Goal createTraverseToROIGoal(
        const geometry_msgs::msg::PoseStamped &roi_pose_);

    // Decide whether current phase has finished to progress to next phase
    void evaluateTransitions();

    // Helpers
    rcl_interfaces::msg::SetParametersResult validateROIParameters(
        const std::vector<rclcpp::Parameter> &parameters);
    geometry_msgs::msg::PoseStamped configuredROI() const;
    void refreshROI();
    void publishROIMarker();
    void publishStatus();
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<rs1_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr roi_parameters_callback_;
    std::string session_id_, parrot_stage_{"Waiting"}, husky_stage_{"Waiting"}, message_;
    std::string instance_id_;
    geometry_msgs::msg::PoseStamped executor_roi_;
    std::string control_mode_{"hold"};
    bool traversalPending() const;
    void enterManualAssistance(Robot robot, const std::string &message);
    void cancelTraversalToROI(Robot robot);
    void finalizeMission();

    // Actions
    rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::SharedPtr traverse_to_roi_client_parrot_;
    rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::SharedPtr traverse_to_roi_client_husky_;

    // TODO (james): Add a sampleTree action client for both parrot and husky

    rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::SendGoalOptions parrot_goal_options_;
    rclcpp_action::Client<rs1_interfaces::action::TraverseToROI>::SendGoalOptions husky_goal_options_;

    // Keep ownership until each executor has returned its final result
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr parrot_goal_handle_;
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::TraverseToROI>::SharedPtr husky_goal_handle_;
    bool parrot_goal_pending_{false};
    bool husky_goal_pending_{false};
    bool parrot_cancel_requested_{false};
    bool husky_cancel_requested_{false};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mission_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_husky_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_parrot_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_mission_service_;

    // ROI visualisation
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr roi_marker_pub_;
    geometry_msgs::msg::PoseStamped roi_pose_;



    MissionState mission_state_;
    bool parrot_succeded_;
    bool husky_succeded_;

};

}  // namespace rs1_mission

#endif  // RS1_MISSION__MISSION_COORDINATOR_HPP_
