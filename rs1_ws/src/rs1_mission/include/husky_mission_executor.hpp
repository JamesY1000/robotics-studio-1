#ifndef RS1_MISSION__HUSKY_MISSION_EXECUTOR_HPP_
#define RS1_MISSION__HUSKY_MISSION_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rs1_interfaces/action/traverse_to_roi.hpp"
#include "rs1_interfaces/srv/set_ground_path.hpp"
#include "rs1_interfaces/srv/get_current_pose.hpp"

namespace rs1_mission
{

class HuskyMissionExecutorNode : public rclcpp::Node
{
public:
    HuskyMissionExecutorNode();

private:
    enum class HuskyState
    {
        IDLE,
        WAITING_FOR_GROUND_PATH,
        GROUND_PATH_RECEIVED,
        TRAVERSING_TO_ROI,
    };

    // Goal request, cancel and accepted callbacks for TraverseToROI action
    rclcpp_action::GoalResponse traverseToROIGoalRequestCb(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const rs1_interfaces::action::TraverseToROI::Goal> goal
    );
    rclcpp_action::CancelResponse traverseToROICancelRequestCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle
    );
    void traverseToROIGoalAcceptedCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> goal_handle
    );

    // Request callbacks for pose lookup and the path supplied by parrot
    void getCurrentPoseRequestCb(
        const std::shared_ptr<rs1_interfaces::srv::GetCurrentPose::Request> request,
        std::shared_ptr<rs1_interfaces::srv::GetCurrentPose::Response> response
    );
    void setGroundPathRequestCb(
        const std::shared_ptr<rs1_interfaces::srv::SetGroundPath::Request> request,
        std::shared_ptr<rs1_interfaces::srv::SetGroundPath::Response> response
    );

    // Navigate short chunks of parrot's route
    void traverseToROI();
    void createGuidancePoses();
    void navigateNextChunk();
    void globalCostmapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
    void navigateThroughPosesResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle);
    void navigateThroughPosesFeedbackCb(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
    void navigateThroughPosesResultCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> mission_goal,
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result);
    void retryChunk(const std::string &reason);
    void advanceRouteProgress(std::size_t next_index);
    void waitForWaypoint(const std::string &reason);

    // Helpers
    geometry_msgs::msg::PoseStamped getCurrentPose(const std::string &frame_id, double max_age = 2.0);
    int costmapValue(const geometry_msgs::msg::PoseStamped &pose) const;
    bool validTargetPose(const geometry_msgs::msg::PoseStamped &pose) const;
    bool validGroundPath(const nav_msgs::msg::Path &path) const;
    bool pathTargetsROI(const nav_msgs::msg::Path &path);
    nav_msgs::msg::Path transformGroundPath(const nav_msgs::msg::Path &path);
    void publishTraversalFeedback(const std::string &stage);
    void abortTraversalToROI(const std::string &message);
    void cancelTraversalToROI();

    // Reset goal handles, stored path and husky state
    void returnToIdle();

    // TF buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TraverseToROI action server and current mission goal
    rclcpp_action::Server<rs1_interfaces::action::TraverseToROI>::SharedPtr traverse_to_roi_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::TraverseToROI>> traverse_to_roi_goal_handle_;

    // Defer cancellation until ROS marks the goal CANCELING
    rclcpp::TimerBase::SharedPtr cancel_timer_;

    // GetCurrentPose and SetGroundPath service servers
    rclcpp::Service<rs1_interfaces::srv::GetCurrentPose>::SharedPtr get_current_pose_service_;
    rclcpp::Service<rs1_interfaces::srv::SetGroundPath>::SharedPtr set_ground_path_service_;

    // Keep parrot's path in husky frame ready for waypoint generation
    nav_msgs::msg::Path ground_path_;

    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigation_goal_handle_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_subscription_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_costmap_;
    rclcpp::TimerBase::SharedPtr navigation_timer_;

    std::vector<geometry_msgs::msg::PoseStamped> guidance_poses_;
    std::vector<bool> discarded_guidance_;
    std::vector<std::size_t> chunk_indices_;
    geometry_msgs::msg::PoseStamped target_roi_;
    std::size_t next_guidance_index_ = 0;
    int chunk_failures_ = 0;
    bool navigation_goal_pending_ = false;
    bool navigation_cancel_requested_ = false;
    std::chrono::steady_clock::time_point waypoint_wait_started_;
    std::string last_stage_;

    double waypoint_spacing_;
    double waypoint_wait_timeout_;
    // The local costmap's goal transform allows 0.3 seconds, while DWB allows
    // 0.5 seconds. Leave some margin below the stricter of these two checks.
    static constexpr double navigation_pose_max_age_ = 0.2;
    double roi_tolerance_;
    int max_chunk_waypoints_;
    int max_chunk_retries_;
    HuskyState husky_state_;
    double ground_path_goal_tolerance_;
};

}  // namespace rs1_mission

#endif  // RS1_MISSION__HUSKY_MISSION_EXECUTOR_HPP_
