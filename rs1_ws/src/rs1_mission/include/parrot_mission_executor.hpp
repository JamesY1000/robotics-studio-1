#ifndef RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_
#define RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "rs1_interfaces/action/traverse_to_roi.hpp"
#include "rs1_interfaces/action/explore_to_pose.hpp"
#include "rs1_interfaces/srv/set_ground_path.hpp"
#include "rs1_interfaces/srv/get_current_pose.hpp"

namespace rs1_mission
{

class ParrotMissionExecutorNode : public rclcpp::Node
{

public:
    ParrotMissionExecutorNode();

private:

    enum class ParrotState
    {
        IDLE,
        TRAVERSING_TO_ROI,
        GENERATING_GROUND_PATH,
        TRAVERSE_INSIDE_ROI,
        GENERATE_TREE_MAP, // Finalise tree map -> send map to husky
    };

    // Goal response and cancel response for TraverseToROI action received from the mission coordinator
    rclcpp_action::GoalResponse traverseToROIGoalRequestCb(const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Goal> goal);

    rclcpp_action::CancelResponse traverseToROICancelRequestCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle);


    // TraverseToROI acceptance callback - call main logic entrypoint here
    void traverseToROIGoalAcceptedCb(std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle);

    // Responses received from nav2pose action
    void navigateToPoseResponseCb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void navigateToPoseResultCb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    // Responses received from parrot's explore_to_pose action calls
    void exploreToPoseResponseCb(rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr goal_handle);
    void exploreToPoseResultCb(const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::WrappedResult &result);

    // GetCurrentPose service response
    void getCurrentPoseResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        rclcpp::Client<rs1_interfaces::srv::GetCurrentPose>::SharedFuture future
    );

    // Responses for ComputePathToPose action
    void computePathToPoseResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr planner_goal
    );

    void computePathToPoseResultCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult &result
    );

    // Response for SetGroundPath service
    void setGroundPathResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        rclcpp::Client<rs1_interfaces::srv::SetGroundPath>::SharedFuture future
    );

    // Response when canceling the current navigation, exploration or planning action
    void cancelActionResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response
    );

    // Main logic - navigating to ROI and generating global path
    void traverseToROI(); // Checks map bounds before choosing navigation or exploration
    void tryExplorationFallback(); // Checks whether failed navigation needs exploration
    void exploreToROI(); // Sends a goal to the ExploreToPose action server
    void generateGroundPath(); // From husky pose to ROI pose - we can use ComputePathToPose
    void sendGroundPath(const nav_msgs::msg::Path &ground_path); // Sends ground path to husky via SetGroundPath service

    // Helpers
    bool targetOutsideGlobalCostmap() const; // Checks map data and ROI bounds
    bool validTargetPose(const geometry_msgs::msg::PoseStamped &target_pose) const;
    void abortTraversalToROI(const std::string &message); // Aborts the TraverseToROI action
    void cancelTraversalToROI();
    void finishCancelledTraversal();
    void returnToIdle(); // Resets goal handles and parrot state to IDLE
    void computeGroundPath(const geometry_msgs::msg::PoseStamped &start_pose);

    // Clients for nav2pose action
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions navigate_to_pose_goal_options_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_goal_handle_;

    // TF for checking Parrot's arrival at the ROI
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double roi_tolerance_;
    double ground_path_goal_tolerance_;

    // Latest global costmap and when we received it
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_subscription_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_costmap_;
    std::chrono::steady_clock::time_point global_costmap_received_at_;

    // Server for TraverseToROI action
    rclcpp_action::Server<rs1_interfaces::action::TraverseToROI>::SharedPtr traverse_to_roi_server_;

    // Goal handle for the TraverseToROI action received from the mission coordinator node
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> traverse_to_roi_goal_handle_;

    // Clients for ExploreToPose action
    rclcpp_action::Client<rs1_interfaces::action::ExploreToPose>::SharedPtr explore_to_pose_client_;
    rclcpp_action::Client<rs1_interfaces::action::ExploreToPose>::SendGoalOptions explore_to_pose_goal_options_;
    rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr explore_to_pose_goal_handle_;

    // Clients for GetCurrentPose service, ComputePathToPose action and SetGroundPath service
    rclcpp::Client<rs1_interfaces::srv::GetCurrentPose>::SharedPtr get_current_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_goal_handle_;
    rclcpp::Client<rs1_interfaces::srv::SetGroundPath>::SharedPtr set_ground_path_client_;

    // Keep the mission active until any pending action has finished
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    bool action_goal_pending_{false};
    bool ground_path_handoff_pending_{false};
    int64_t get_current_pose_request_id_{-1};

    ParrotState parrot_state_;

    geometry_msgs::msg::PoseStamped husky_current_pose_;


};

}  // namespace rs1_mission

#endif  // RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_
