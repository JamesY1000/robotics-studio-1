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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "rs1_interfaces/action/traverse_to_roi.hpp"
#include "rs1_interfaces/action/explore_to_pose.hpp"
#include "rs1_interfaces/action/sample_roi.hpp"
#include "rs1_interfaces/msg/tree_map.hpp"
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
    void finishSampling();
    void maybePopUpRecovery();
    void maybeDuckRecovery();
    void abortSampling(const std::string & message);
    void cancelSampling();
    void finishCancelledSampling();
    void sampleROICancelResponseCb(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::SampleROI>> mission_goal,
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::CancelResponse::SharedPtr response);
    void navigateThroughPosesResponseCb(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle);
    void navigateThroughPosesResultCb(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
    void navigateThroughPosesFeedbackCb(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
    void stopDetection();
    void restoreTransitClearance();
    void publishSamplingFeedback(const std::string & stage);
    bool currentPose(geometry_msgs::msg::PoseStamped & pose) const;
    // Ground distance from a pose to the current sweep waypoint (infinity when
    // there is no active waypoint).  Used as the anchor for altitude recoveries
    // so they only end once the drone has genuinely closed on the target.
    double distanceToCurrentWaypoint(const geometry_msgs::msg::PoseStamped & pose) const;

    // Direct velocity lane follower: flies the swath lines without the costmap.
    void beginVelocitySweep();
    void sweepControlCb();
    void stopSweep();
    void scanCb(const sensor_msgs::msg::LaserScan & msg);
    bool forwardClear(double & min_range) const;
    bool scanMinRange(double centre, double half_width, double & out) const;
    bool scanClearances(double & left, double & front, double & right) const;
    bool widestGap(double & centre, double & width) const;
    bool scanFresh() const;

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
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions navigate_through_poses_goal_options_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_goal_handle_;
    rclcpp::Client<rs1_interfaces::srv::DetectYolo>::SharedPtr detect_client_;
    rclcpp::Client<rs1_interfaces::srv::ProcessTreeLocations>::SharedPtr process_client_;
    rclcpp::Client<rs1_interfaces::srv::SetClearance>::SharedPtr set_clearance_client_;
    rclcpp::Publisher<rs1_interfaces::msg::TreeMap>::SharedPtr tree_map_pub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;
    bool detection_request_pending_{false};
    std::vector<MergedTree> tree_map_;
    std::size_t total_waypoints_{0};
    int16_t waypoints_remaining_{0};

    // ROI box normalised into the executor's planning frame (parrot1_map)
    geometry_msgs::msg::Point sweep_min_;
    geometry_msgs::msg::Point sweep_max_;
    std::string sweep_frame_;
    double sweep_z_{0.0};

    // Exploration fallback state
    bool exploring_for_sweep_{false};
    int fallback_hops_{0};
    int max_fallback_hops_{3};
    bool explore_before_sweep_{false};

    // Velocity sweep follower state
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
    rclcpp::Time latest_scan_time_;
    rclcpp::TimerBase::SharedPtr sweep_timer_;
    std::vector<geometry_msgs::msg::PoseStamped> sweep_waypoints_;
    std::size_t sweep_index_{0};
    bool velocity_sweep_active_{false};
    int avoid_side_{0};
    bool avoid_engaged_{false};
    int avoid_clear_ticks_{0};
    int avoid_stuck_ticks_{0};
    int reverse_ticks_{0};
    bool reversing_{false};
    std::string sweep_controller_{"velocity"};
    double sweep_control_hz_{10.0};
    double max_sweep_speed_{1.0};
    double sweep_speed_gain_{0.6};
    double max_yaw_rate_{1.0};
    double sweep_yaw_gain_{1.5};
    double heading_gate_{0.7};
    double waypoint_tolerance_{1.0};
    double waypoint_skip_distance_{2.5};
    double waypoint_skip_time_{4.0};
    int skip_ticks_{0};
    int skip_ticks_threshold_{40};
    // No-progress watchdog: if the drone stops closing on a waypoint (wedged
    // against a tree mid-lane), skip that leg instead of oscillating forever.
    double no_progress_timeout_{12.0};
    int no_progress_ticks_threshold_{120};
    int no_progress_ticks_{0};
    double leg_best_along_{-1e18};

    // Pop-up recovery: when the drone skips several legs in a row, or is
    // hard-wedged near a leg end (can't drive forward and the reverse/avoidance
    // oscillation suppresses the no-progress watchdog), ascend above the canopy
    // to clear the wedge, then descend again once a waypoint is reached freely.
    int consecutive_skips_{0};
    int max_consecutive_skips_{3};
    double hard_wedge_timeout_{8.0};
    int hard_wedge_ticks_{0};
    int hard_wedge_ticks_threshold_{80};
    bool popup_active_{false};
    bool descend_resume_sweep_{false};
    int popup_grace_ticks_{0};
    int popup_grace_ticks_threshold_{400};
    // The oak canopy in v2_forest occupies ~1.2-6.5 m above the ground (mesh
    // bbox 1.24-6.53 m), so a pop-up only clears the branches when it ends up
    // ABOVE that band.  A 4 m pop-up sits in the middle of the canopy and
    // wedges the drone again, which is why the recovery appeared to do nothing.
    double popup_clearance_{8.0};
    // Each successive pop-up within the same wedge climbs one step higher
    // (bounded) so an unexpectedly deep canopy is still cleared instead of
    // looping at a height that is inside the branches.
    double popup_step_m_{2.0};
    double popup_max_clearance_{12.0};
    int wedge_attempts_{0};
    // How many rungs of the duck/pop-up ladder to try before the leg is
    // abandoned.  Without this the ladder could loop forever and never sweep.
    int max_recovery_attempts_{4};
    double popup_grace_time_{10.0};

    // Duck recovery: at branch height the forward lidar sees oak branches in
    // every direction, but a metre off the deck only the widely spaced trunks
    // remain, so descending gets the drone unstuck while tree detection stays
    // active.  Falls back to a pop-up if the duck does not clear the wedge.
    bool duck_active_{false};
    bool ascend_resume_sweep_{false};
    int duck_grace_ticks_{0};
    int duck_grace_ticks_threshold_{200};
    // The oak branches start ~1.2 m above the ground and the scan ring sits
    // 0.2 m above base_link, so ducking to 1.0 m leaves the lidar inside the
    // lowest branches.  0.8 m puts the ring clearly below the branch band.
    double duck_clearance_{0.8};
    // Grace window before the duck is declared useless (seconds).  Kept short so
    // the ladder moves on quickly instead of parking the drone in the branches.
    double duck_grace_time_{5.0};
    // Clearance (metres above the ground) that was last requested from the
    // terrain-following node.  The duck guard is judged from THIS, not from a
    // world-z difference against the last waypoint: the terrain in v2_forest
    // rises by up to ~7 m across a lane, so on the uphill half of the ROI a
    // world-z test always thought the drone was "up in the canopy" and silently
    // turned every duck into a no-op descent to the search clearance.
    double commanded_clearance_m_{0.0};

    // Altitude recovery: after a duck / pop-up / canopy descent, resume the
    // search clearance once the drone has physically moved a short distance
    // past the obstacle (not on waypoint arrival, which can be a whole 100 m
    // lane away and leaves it at the altered height for too long).
    bool alt_recover_active_{false};
    double alt_recover_origin_x_{0.0};
    double alt_recover_origin_y_{0.0};
    // Distance to the current waypoint when the recovery started.  The resume
    // is anchored on CLOSING that distance, not on raw displacement, so backing
    // out of a wedge (reverse) or strafing in place cannot end the recovery
    // early and drop the drone straight back into the obstacle.
    double alt_recover_target_dist_{1e18};
    double alt_recover_travel_m_{2.5};

    // Safety net: a broken world<->map registration (or a localisation jump)
    // makes the drone chase waypoints that are not where it thinks they are, so
    // it leaves the ROI entirely.  Abort with a clear reason instead.
    double roi_escape_margin_{20.0};
    double roi_escape_timeout_{8.0};
    int roi_escape_ticks_threshold_{80};
    int roi_escape_ticks_{0};

    // Transit leg: with no teleport the drone starts outside the ROI and flies
    // in, so it is expected to be far from the box at first.  Track whether it
    // has arrived yet, and bound the inbound leg instead of aborting it.
    double roi_enter_margin_{5.0};
    double roi_transit_timeout_{120.0};
    int roi_transit_ticks_threshold_{1200};
    int roi_transit_ticks_{0};
    bool entered_roi_{false};
    // Set when the drone has to fly in: transit at the higher clearance and only
    // descend to the search clearance once the ROI is reached.
    bool descend_on_entry_{false};
    double forward_cone_{0.6};
    double obstacle_stop_distance_{1.2};
    double min_gap_width_{0.25};

    // Lidar watchdog: a GPU lidar can die mid-flight, leaving only a stale
    // scan.  Stale scans must not drive avoidance, and a long outage aborts the
    // sweep because tree anchoring needs the lidar.
    double scan_stale_timeout_{2.0};
    double scan_outage_timeout_{30.0};
    int scan_outage_warn_ticks_threshold_{50};
    int scan_outage_ticks_threshold_{300};
    int scan_outage_ticks_{0};
    bool scan_outage_warned_{false};

    // Detour / line-rejoin state
    geometry_msgs::msg::Point seg_start_;
    bool segment_ready_{false};
    double avoid_offset_{0.0};
    double lookahead_distance_{3.0};
    double avoid_distance_{2.5};
    double avoid_offset_m_{2.5};
    double avoid_rate_{0.5};
    double avoid_disengage_margin_{1.5};
    double avoid_min_slowdown_{0.5};
    double side_cone_{1.2};
    double steering_cone_{0.5};
    double turn_lead_m_{3.0};
    double turn_offset_m_{3.0};

    // Sampling parameters
    double detection_rate_hz_{1.0};
    double detection_range_{20.0};
    double camera_hfov_{2.0944};
    double swath_overlap_{0.2};
    double min_swath_spacing_{2.0};
    bool start_double_pass_{true};
    double merge_distance_{4.0};
    double final_merge_distance_{5.5};
    double roi_tree_margin_{0.0};
    int min_observations_{2};

    ParrotState parrot_state_;

    geometry_msgs::msg::PoseStamped husky_current_pose_;


};

}  // namespace rs1_mission

#endif  // RS1_MISSION__PARROT_MISSION_EXECUTOR_HPP_
