#ifndef RS1_MISSION__HUSKY_MISSION_EXECUTOR_HPP_
#define RS1_MISSION__HUSKY_MISSION_EXECUTOR_HPP_

#include <chrono>
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "rs1_interfaces/action/measure_trees.hpp"
#include "rs1_interfaces/srv/measure_dbh.hpp"
#include "rs1_interfaces/srv/locate_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "return_home.hpp"

#include "rs1_interfaces/action/traverse_to_roi.hpp"
#include "rs1_interfaces/msg/tree_map.hpp"
#include "rs1_interfaces/msg/tree_sampling_target.hpp"
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
        MEASURING_TREES,
        RETURNING_HOME,
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

    // Store the tree map published by the parrot into the database file
    void treeMapCb(const rs1_interfaces::msg::TreeMap::SharedPtr msg);

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
    std::unique_ptr<ReturnHomeExecution> home_execution_;
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

    // Tree map database received from the parrot
    rclcpp::Subscription<rs1_interfaces::msg::TreeMap>::SharedPtr tree_map_subscription_;
    std::string tree_database_file_;

    using MeasureTrees = rs1_interfaces::action::MeasureTrees;
    using SurveyHandle = rclcpp_action::ServerGoalHandle<MeasureTrees>;
    using Navigate = nav2_msgs::action::NavigateToPose;
    using PlanPath = nav2_msgs::action::ComputePathToPose;
    using BackUp = nav2_msgs::action::BackUp;
    using MeasureDBH = rs1_interfaces::srv::MeasureDBH;
    using LocateTree = rs1_interfaces::srv::LocateTree;
    enum class SurveyStep { Plan, Approach, AssistedSelection, Recover, Moving, TryCurrentPose, CheckPose, Locating, Reading };

    void setupTreeSurvey();
    void surveyTick();
    void planTreeVisits();
    void nextAssistedTree(const std::string &reason);
    void chooseAssistedTree();
    void publishSamplingTarget(bool active = true);
    void approachTree();
    void resetApproachSearch();
    void finishApproachSearch();
    void checkMeasurementPose(bool try_current_pose = false);
    void locateTrunk();
    void requestDBH(double bearing);
    void resetTreeApproach();
    void restoreTreeView();
    void handleBlockedDrive();
    void recoverTreeApproach();
    void deferTree(const std::string &reason);
    void footprintCb(geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint);
    bool surveyFootprintClear(const geometry_msgs::msg::PoseStamped &pose, bool allow_unknown = false) const;
    bool footprintClearOnMap(const geometry_msgs::msg::PoseStamped &pose,
        const nav_msgs::msg::OccupancyGrid &map, bool allow_unknown) const;
    bool surveyRobotClear(const geometry_msgs::msg::PoseStamped &pose) const;
    bool surveyMapsFresh() const;
    bool surveyPathClear(const nav_msgs::msg::Path &path);
    void nextApproach(const std::string &reason = "No reachable stand-off pose");
    void waitForSurvey(const std::string &reason);
    void surveyFeedback(const std::string &stage);
    void finishSurvey(const std::string &message, bool success = false);
    geometry_msgs::msg::Point currentTrunk() const;
    template<typename Action>
    void sendSurveyMotion(const typename rclcpp_action::Client<Action>::SharedPtr &client,
                          const typename Action::Goal &goal);

    rclcpp_action::Server<MeasureTrees>::SharedPtr survey_server_;
    std::shared_ptr<SurveyHandle> survey_goal_;
    rclcpp_action::Client<Navigate>::SharedPtr tree_nav_;
    rclcpp_action::Client<PlanPath>::SharedPtr tree_planner_;
    rclcpp_action::Client<BackUp>::SharedPtr tree_backup_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr survey_local_map_subscription_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr survey_local_map_;
    std::vector<geometry_msgs::msg::Point> survey_footprint_;
    rclcpp::Time footprint_stamp_{0, 0, RCL_ROS_TIME};
    std::vector<bool> tried_approaches_;
    std::vector<double> measurement_radii_;
    std::array<unsigned int, 8> sector_plans_{};
    std::vector<unsigned int> sector_radius_plans_;
    std::array<unsigned int, 8> failed_sectors_{};
    unsigned int planned_approaches_{0};
    bool departure_blocked_{false};
    std::optional<geometry_msgs::msg::PoseStamped> best_approach_;
    unsigned int best_approach_index_{0};
    double best_approach_score_{0};
    unsigned int rejected_paths_{0};
    bool choosing_viewpoint_{false};
    bool moved_viewpoint_{false};
    double viewpoint_bearing_{0};
    std::optional<rclcpp::Time> view_arrival_stamp_;
    unsigned int retreat_count_{0};
    bool drive_failed_{false};
    std::optional<geometry_msgs::msg::Point> retreat_start_;
    std::vector<geometry_msgs::msg::Point> failed_measurement_views_;
    std::optional<geometry_msgs::msg::PoseStamped> settled_pose_;
    bool navigating_tree_{false};
    std::optional<geometry_msgs::msg::Point> progress_position_;
    std::chrono::steady_clock::time_point progress_at_;
    double motion_limit_{0};
    double tree_progress_timeout_, tree_planning_timeout_;
    std::string tree_navigation_bt_;
    rclcpp::Client<MeasureDBH>::SharedPtr dbh_client_;
    rclcpp::Client<LocateTree>::SharedPtr locate_client_;
    std::optional<geometry_msgs::msg::Point> refined_trunk_;
    std::optional<double> first_diameter_;
    std::vector<rs1_interfaces::msg::PendingTreeMeasurement> pending_views_;
    bool assisted_measurement_{false};
    std::vector<uint32_t> assisted_tried_;
    std::string assisted_failures_;
    rclcpp::Publisher<rs1_interfaces::msg::TreeSamplingTarget>::SharedPtr sampling_target_publisher_;
    double first_view_bearing_{0};
    std::string unresolved_;
    std::optional<geometry_msgs::msg::Point> blocked_at_;
    uint32_t blocked_tree_{0};
    unsigned int navigation_failures_{0};
    unsigned int localisation_failures_{0};
    unsigned int measurement_failures_{0};
    rclcpp::TimerBase::SharedPtr survey_timer_;
    SurveyStep survey_step_{SurveyStep::Plan};
    std::vector<uint32_t> visit_order_;
    std::vector<rs1_interfaces::msg::TreeMeasurement> readings_;
    std::size_t visit_{0};
    unsigned int approach_{0};
    double first_bearing_{0};
    geometry_msgs::msg::PoseStamped approach_pose_;
    std::function<void()> cancel_motion_;
    bool stopping_motion_{false};
    std::optional<int64_t> dbh_request_id_;
    std::chrono::steady_clock::time_point step_started_;
    double tree_standoff_, dbh_timeout_, tree_motion_timeout_;
    double measurement_min_range_, measurement_max_range_;

    // Keep parrot's path in husky frame ready for waypoint generation
    nav_msgs::msg::Path ground_path_;
    std::string saved_route_id_;
    geometry_msgs::msg::PoseStamped saved_route_roi_;

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
