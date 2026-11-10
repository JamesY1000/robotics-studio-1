#ifndef RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_
#define RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "return_home.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "rs1_interfaces/action/traverse_to_roi.hpp"
#include "rs1_interfaces/action/explore_to_pose.hpp"
#include "rs1_interfaces/action/sample_roi.hpp"
#include "rs1_interfaces/msg/tree_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rs1_interfaces/msg/tree_location.hpp"
#include "rs1_interfaces/srv/set_ground_path.hpp"
#include "rs1_interfaces/srv/get_current_pose.hpp"
#include "rs1_interfaces/srv/detect_yolo.hpp"
#include "rs1_interfaces/srv/process_tree_locations.hpp"
#include "rs1_interfaces/srv/set_clearance.hpp"

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
        RETURNING_HOME,
    };

    // Goal response and cancel response for TraverseToROI action received from the mission coordinator
    std::unique_ptr<ReturnHomeExecution> home_execution_;
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
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response);

    // ---- Phase 2: ROI sampling (lawnmower sweep + tree detection) ----
    rclcpp_action::GoalResponse sampleROIGoalRequestCb(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const rs1_interfaces::action::SampleROI::Goal> goal);
    rclcpp_action::CancelResponse sampleROICancelRequestCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> goal_handle);
    void sampleROIGoalAcceptedCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> goal_handle);

    void sampleROI();
    void setSearchClearance();
    void setSearchClearanceResponseCb(
        rclcpp::Client<rs1_interfaces::srv::SetClearance>::SharedFuture future);
    void beginSweep();
    void computeSwaths(std::vector<geometry_msgs::msg::PoseStamped> & poses,
                       double from_x, double from_y) const;
    void detectionTimerCb();
    void detectResponseCb(rclcpp::Client<rs1_interfaces::srv::DetectYolo>::SharedFuture future);
    void processResponseCb(
        rclcpp::Client<rs1_interfaces::srv::ProcessTreeLocations>::SharedFuture future);
    void mergeTree(const rs1_interfaces::msg::TreeLocation & tree);
    std::size_t publishTreeMap();
    void publishTreeMarkers(const std::vector<rs1_interfaces::msg::TreeLocation> & trees);
    void finishSampling();
    void abortSampling(const std::string & message);
    void cancelSampling();
    void finishCancelledSampling();
    void sampleROICancelResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> mission_goal,
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response);
    // Per-waypoint Nav2 sweep: fly the lanes one NavigateToPose goal at a time
    // so the costmap only needs to be valid one leg ahead of the drone (a single
    // NavigateThroughPoses across the whole box fails on an incomplete map).
    void navigateToNextWaypoint();
    void sweepWaypointResponseCb(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void sweepWaypointResultCb(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
    void stopDetection();
    void restoreTransitClearance();
    // Called from the sweep waypoint feedback/result callbacks: descends from the
    // transit clearance to the search clearance once the drone is inside the ROI
    // box.
    void maybeDescendOnEntry();
    // Lidar watchdog: the sweep relies on the lidar for tree anchoring and on
    // Nav2's costmap for obstacle avoidance, so a dead lidar stalls the drone
    // silently.  Abort sampling with a clear message instead.
    void scanCb(const sensor_msgs::msg::LaserScan & msg);
    void lidarWatchdogCb();
    // Per-waypoint stall watchdog.  Nav2 runs planner_server and the global
    // costmap in ONE process with a single-threaded executor, so a very large
    // SLAM-grown costmap can block a single compute_path_to_pose.  The
    // NavigateToPose goal then never returns and the sweep hangs forever (a
    // 640 s hang at waypoint 9/14 on the 80x100 m ROI).  bt_navigator keeps
    // publishing feedback while that happens, so "feedback arrived" proves
    // nothing - only a shrinking distance_remaining does.
    void sweepStallWatchdogCb();
    void resetSweepProgress();
    // Abandon the waypoint the sweep is currently on and continue with the next
    // one.  Used both by the stall watchdog (no progress) and when the
    // exploration fallback has been exhausted, so a single unreachable waypoint
    // (a tree in the lane, a goal inside lethal space) costs one leg instead of
    // the whole run.  Aborts sampling only if this happens
    // `max_consecutive_skips_` times in a row.
    void skipCurrentWaypoint(const std::string & reason);
    void publishSamplingFeedback(const std::string & stage);
    bool currentPose(geometry_msgs::msg::PoseStamped & pose) const;

    // Cancels the in-flight per-waypoint sweep goal and any exploration fallback.
    void stopSweep();

    bool validROIBox(const geometry_msgs::msg::Point & min, const geometry_msgs::msg::Point & max) const;
    bool prepareSweepBox();

    // Exploration fallback for the sweep: the Nav2 costmap is not complete
    // enough to plan across the ROI, so drive the weighted frontier explorer
    // (as in phase 1) toward the ROI and resume the sweep afterwards.
    void exploreToSweepTarget();
    void retrySweepWithExploration(const std::string & reason);
    void exploreForSweepResponseCb(
        rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::SharedPtr goal_handle);
    void exploreForSweepResultCb(
        const rclcpp_action::ClientGoalHandle<rs1_interfaces::action::ExploreToPose>::WrappedResult & result);
    void sampleROIExploreCancelResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> mission_goal,
        rclcpp_action::Client<rs1_interfaces::action::ExploreToPose>::CancelResponse::SharedPtr response);

    struct MergedTree
    {
        std::string tree_type;
        double x{0.0};
        double y{0.0};
        double z{0.0};
        uint32_t observations{1};
        std::map<std::string, double> type_votes;   // score-weighted votes
        std::map<std::string, double> type_x_sum;   // score-weighted sums
        std::map<std::string, double> type_y_sum;
        std::map<std::string, double> type_z_sum;

        rs1_interfaces::msg::TreeLocation locationMessage() const
        {
            rs1_interfaces::msg::TreeLocation tree;
            tree.tree_type = tree_type;
            tree.location.x = x;
            tree.location.y = y;
            tree.location.z = z;
            tree.observations = observations;
            return tree;
        }

        void addObservation(const std::string & type, double tx, double ty, double tz, double score)
        {
            // Weight confident detections more than weak ones so the merged
            // centroid and the winning type are pulled toward good observations.
            const double weight = (std::isfinite(score) && score > 0.0) ? score : 1.0;
            type_votes[type] += weight;
            type_x_sum[type] += tx * weight;
            type_y_sum[type] += ty * weight;
            type_z_sum[type] += tz * weight;
            ++observations;  // raw detection count, independent of weighting

            std::string winner_type = tree_type;
            double winner_votes = 0.0;
            for (const auto & [candidate_type, count] : type_votes)
            {
                if (count > winner_votes)
                {
                    winner_type = candidate_type;
                    winner_votes = count;
                }
            }

            if (!winner_type.empty() && type_votes.count(winner_type) > 0)
            {
                tree_type = winner_type;
                x = type_x_sum[winner_type] / type_votes[winner_type];
                y = type_y_sum[winner_type] / type_votes[winner_type];
                z = type_z_sum[winner_type] / type_votes[winner_type];
            }
        }

        void absorb(const MergedTree & other)
        {
            // Merge another entry's whole history into this one and recompute
            // the winning type + centroid (used by the final dedup pass).
            for (const auto & [type, weight] : other.type_votes)
            {
                type_votes[type] += weight;
                type_x_sum[type] += other.type_x_sum.at(type);
                type_y_sum[type] += other.type_y_sum.at(type);
                type_z_sum[type] += other.type_z_sum.at(type);
            }
            observations += other.observations;

            std::string winner_type;
            double winner_votes = -1.0;
            for (const auto & [candidate_type, count] : type_votes)
            {
                if (count > winner_votes)
                {
                    winner_type = candidate_type;
                    winner_votes = count;
                }
            }
            if (!winner_type.empty() && type_votes.count(winner_type) > 0)
            {
                tree_type = winner_type;
                x = type_x_sum[winner_type] / type_votes[winner_type];
                y = type_y_sum[winner_type] / type_votes[winner_type];
                z = type_z_sum[winner_type] / type_votes[winner_type];
            }
        }
    };

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
    // Goal handle for the per-waypoint sweep navigation (see navigateToNextWaypoint).
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr sweep_nav_goal_handle_;

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

    // Phase 2: ROI sampling server/clients and detected tree map
    rclcpp_action::Server<rs1_interfaces::action::SampleROI>::SharedPtr sample_roi_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> sample_roi_goal_handle_;
    rclcpp::Client<rs1_interfaces::srv::DetectYolo>::SharedPtr detect_client_;
    rclcpp::Client<rs1_interfaces::srv::ProcessTreeLocations>::SharedPtr process_client_;
    rclcpp::Client<rs1_interfaces::srv::SetClearance>::SharedPtr set_clearance_client_;
    rclcpp::Publisher<rs1_interfaces::msg::TreeMap>::SharedPtr tree_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_marker_pub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;
    // Lidar outage watchdog
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Time latest_scan_time_;
    rclcpp::TimerBase::SharedPtr lidar_watchdog_timer_;
    double scan_outage_timeout_{30.0};
    // Waypoint stall watchdog state
    rclcpp::TimerBase::SharedPtr sweep_stall_timer_;
    rclcpp::Time last_sweep_progress_time_;
    geometry_msgs::msg::Point sweep_progress_anchor_;
    bool sweep_progress_anchor_valid_{false};
    double waypoint_stall_timeout_{60.0};
    double waypoint_progress_epsilon_{1.0};
    int consecutive_skips_{0};
    int max_consecutive_skips_{3};
    // For waypoints that the drone physically can't reach
    double waypoint_close_enough_m_{3.0};
    double waypoint_close_enough_dwell_{3.0};
    bool withinCloseEnough(double & distance) const;
    bool acceptWaypointIfCloseEnough(const std::string & reason);
    // Where the drone was at the previous skipped waypoint.  Skips from the same
    // spot are what "wedged" looks like; skips while moving down a lane are not.
    geometry_msgs::msg::Point skip_anchor_;
    bool skip_anchor_valid_{false};
    bool detection_request_pending_{false};
    // Stationary-inference limiter: a parked drone sees the same frame again and
    // again, and YOLO is the heaviest CPU consumer in the stack, so inference is
    // paused after `stationary_detection_limit_` requests at one spot until the
    // drone has moved `detection_motion_threshold_m_.
    geometry_msgs::msg::Point detection_motion_anchor_;
    bool detection_motion_anchor_valid_{false};
    int stationary_detections_{0};
    int stationary_detection_limit_{3};
    double detection_motion_threshold_m_{0.5};
    std::vector<MergedTree> tree_map_;
    // Finalized map built by publishTreeMap(); attached to the SampleROI result
    // so the coordinator receives the tree map from the action call.
    rs1_interfaces::msg::TreeMap last_tree_map_;
    std::size_t total_waypoints_{0};
    int16_t waypoints_remaining_{0};

    // ROI box normalised into the executor's planning frame (parrot1_odom)
    geometry_msgs::msg::Point sweep_min_;
    geometry_msgs::msg::Point sweep_max_;
    // The same ROI box expressed in `world`, which is the frame perception
    // reports tree positions in.  parrot1_odom only equals `world` while the EKF
    // fuses the odometry POSE, so the tree filter cannot reuse sweep_min_/max_.
    geometry_msgs::msg::Point tree_min_;
    geometry_msgs::msg::Point tree_max_;
    std::string sweep_frame_;
    double sweep_z_{0.0};

    // Exploration fallback state
    bool exploring_for_sweep_{false};
    int fallback_hops_{0};
    int max_fallback_hops_{1};
    bool explore_before_sweep_{false};

    // Sweep lane state (per-waypoint Nav2 follower)
    std::vector<geometry_msgs::msg::PoseStamped> sweep_waypoints_;
    std::size_t sweep_index_{0};

    // Inbound transit: with no teleport the drone starts outside the ROI and
    // flies in.  Drives the descend-on-entry transition (search clearance only
    // once the drone is inside the box).
    double roi_enter_margin_{5.0};
    // Set when the drone has to fly in: hold the transit clearance on the way in
    // and descend to the search clearance once the ROI is reached.
    bool descend_on_entry_{false};

    // U-turn geometry for the lane-end turnaround (turn_lead must match the
    // test's hardcoded value for the --teleport path).
    double turn_lead_m_{3.0};
    double turn_offset_m_{3.0};
    // Long lanes are split into legs of at most this length.  A "lane" on the
    // 80x100 m ROI is ~106 m: a single NavigateToPose across it needs the global
    // costmap to be usable over the whole strip, and if the planner wedges there
    // is no intermediate goal to fall back to.  Shorter legs also give the stall
    // watchdog somewhere to resume from mid-lane.
    double max_leg_length_m_{25.0};

    // Sampling parameters
    double detection_rate_hz_{1.0};
    double detection_range_{20.0};
    double camera_hfov_{2.0944};
    double swath_overlap_{0.2};
    double min_swath_spacing_{2.0};
    double merge_distance_{4.0};
    double final_merge_distance_{5.5};
    double roi_tree_margin_{0.0};
    int min_observations_{2};

    ParrotState parrot_state_;

    geometry_msgs::msg::PoseStamped husky_current_pose_;


};

}  // namespace rs1_mission

#endif  // RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_
