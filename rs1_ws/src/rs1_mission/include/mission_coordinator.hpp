#ifndef RS1_MISSION__MISSION_COORDINATOR_HPP_
#define RS1_MISSION__MISSION_COORDINATOR_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "std_srvs/srv/trigger.hpp"

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
        FAILED
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
    void publishROIMarker();
    void failMission(const std::string &message);
    void cancelTraversalToROI(Robot robot);
    void returnToIdle();

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

    // ROI visualisation
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr roi_marker_pub_;
    geometry_msgs::msg::PoseStamped roi_pose_;



    MissionState mission_state_;
    bool parrot_succeded_;
    bool husky_succeded_;

};

}  // namespace rs1_mission

#endif  // RS1_MISSION__MISSION_COORDINATOR_HPP_