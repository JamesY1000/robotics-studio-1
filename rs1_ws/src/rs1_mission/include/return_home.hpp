#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rs1_interfaces/action/return_home.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace rs1_mission
{
// Both executors use the same Nav2 return, but still own their own busy/idle state
class ReturnHomeExecution
{
    using Home = rs1_interfaces::action::ReturnHome;
    using Handle = rclcpp_action::ServerGoalHandle<Home>;
    using Navigate = nav2_msgs::action::NavigateToPose;
    using NavHandle = rclcpp_action::ClientGoalHandle<Navigate>;
    using Clock = std::chrono::steady_clock;

public:
    ReturnHomeExecution(rclcpp::Node &node, std::shared_ptr<tf2_ros::Buffer> tf,
                        std::string robot, std::function<bool()> idle,
                        std::function<void(bool)> busy)
        : node_(node), tf_(std::move(tf)), robot_(std::move(robot)), busy_(std::move(busy))
    {
        tolerance_ = node_.declare_parameter<double>("home_tolerance_m", 0.5);
        progress_timeout_ = node_.declare_parameter<double>("home_progress_timeout_s", 30.0);
        timeout_ = node_.declare_parameter<double>("home_timeout_s", 600.0);
        for (double value : {tolerance_, progress_timeout_, timeout_})
            if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid home return limits");
        nav_ = rclcpp_action::create_client<Navigate>(&node_, "/" + robot_ + "/navigate_to_pose");
        server_ = rclcpp_action::create_server<Home>(&node_, "/" + robot_ + "/return_home",
            [this, idle](const auto &, auto goal) {
                const auto &p = goal->home.pose.position;
                const auto &q = goal->home.pose.orientation;
                const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                if (!idle() || goal_ || goal->home.header.frame_id != "world" ||
                    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                    !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
                    return rclcpp_action::GoalResponse::REJECT;
                busy_(true);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](auto goal) {
                return goal == goal_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
            },
            [this](auto goal) {
                goal_ = goal;
                started_ = progress_at_ = Clock::now();
                progress_pose_.reset();
                stop_reason_.clear();
                sent_ = pending_ = cancel_requested_ = false;
            });
        timer_ = node_.create_wall_timer(std::chrono::milliseconds(200), [this]() { tick(); });
    }

private:
    geometry_msgs::msg::Point position() const
    {
        const auto transform = tf_->lookupTransform("world", robot_ + "_base_link", tf2::TimePointZero);
        const auto age = (node_.now() - rclcpp::Time(transform.header.stamp)).seconds();
        const auto &p = transform.transform.translation;
        if (age < -0.1 || age > 2.0 || !std::isfinite(p.x) || !std::isfinite(p.y))
            throw std::runtime_error("Waiting for fresh " + robot_ + " localisation");
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        return point;
    }

    double distance(const geometry_msgs::msg::Point &p) const
    {
        const auto &home = goal_->get_goal()->home.pose.position;
        return std::hypot(home.x-p.x, home.y-p.y);
    }

    void finish(const std::string &message, bool success = false)
    {
        // A cancel acknowledgement is not proof Nav2 has stopped, wait for its result
        if (!goal_ || pending_ || nav_goal_) return;
        auto result = std::make_shared<Home::Result>();
        result->message = message;
        if (goal_->is_canceling()) goal_->canceled(result);
        else if (success) goal_->succeed(result);
        else goal_->abort(result);
        goal_.reset();
        busy_(false);
    }

    void stop()
    {
        if (nav_goal_ && !cancel_requested_)
        {
            auto owner = goal_;
            cancel_requested_ = true;
            try
            {
                nav_->async_cancel_goal(nav_goal_, [this, owner](auto response) {
                    if (goal_ == owner && (!response || response->goals_canceling.empty()))
                        cancel_requested_ = false;
                });
            }
            catch (const std::exception &) { cancel_requested_ = false; }
        }
        finish(stop_reason_.empty() ? "Return home cancelled" : stop_reason_);
    }

    void tick()
    {
        if (!goal_) return;
        if (goal_->is_canceling() || !stop_reason_.empty()) { stop(); return; }
        const auto elapsed = std::chrono::duration<double>(Clock::now()-started_).count();
        if (elapsed > timeout_)
        {
            stop_reason_ = "Return home timed out, please assist then Resume";
            stop();
            return;
        }
        try
        {
            const auto p = position();
            const double remaining = distance(p);
            if (!progress_pose_ || std::hypot(p.x-progress_pose_->x, p.y-progress_pose_->y) >= 0.2)
            {
                progress_pose_ = p;
                progress_at_ = Clock::now();
            }
            if (!sent_ && remaining <= tolerance_) { finish("Already at home", true); return; }
            if (std::chrono::duration<double>(Clock::now()-progress_at_).count() > progress_timeout_)
            {
                stop_reason_ = "No progress returning home, please assist then Resume";
                stop();
                return;
            }
            auto feedback = std::make_shared<Home::Feedback>();
            feedback->stage = sent_ ? "Navigating home" : "Waiting for home navigation";
            feedback->distance_remaining = remaining;
            goal_->publish_feedback(feedback);
            if (sent_) return;
            if (!nav_->action_server_is_ready()) { finish("Nav2 is unavailable for return home"); return; }

            // World home stays fixed, even if the robot's SLAM map frame has moved
            auto home = goal_->get_goal()->home;
            const auto transform = tf_->lookupTransform(robot_ + "_map", "world", tf2::TimePointZero);
            Navigate::Goal navigation;
            tf2::doTransform(home, navigation.pose, transform);
            navigation.pose.header.stamp = node_.now();
            // Nav2 handles XY, terrain following keeps Parrot at its transit clearance
            navigation.pose.pose.position.z = 0.0;
            auto owner = goal_;
            typename rclcpp_action::Client<Navigate>::SendGoalOptions options;
            options.goal_response_callback = [this, owner](auto handle) {
                if (owner != goal_) return;
                pending_ = false;
                nav_goal_ = handle;
                if (!handle) finish("Nav2 rejected the return-home goal");
                else if (owner->is_canceling() || !stop_reason_.empty()) stop();
            };
            options.result_callback = [this, owner](const auto &result) {
                if (owner != goal_ || !nav_goal_ || result.goal_id != nav_goal_->get_goal_id()) return;
                nav_goal_.reset();
                cancel_requested_ = false;
                if (owner->is_canceling() || !stop_reason_.empty()) { stop(); return; }
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
                {
                    finish("Nav2 failed to return home, please assist then Resume");
                    return;
                }
                try
                {
                    const bool arrived = distance(position()) <= tolerance_;
                    finish(arrived ? "Reached home" : "Nav2 finished outside the home tolerance", arrived);
                }
                catch (const std::exception &e) { finish(e.what()); }
            };
            sent_ = pending_ = true;
            try { nav_->async_send_goal(navigation, options); }
            catch (const std::exception &e) { pending_ = false; finish(e.what()); }
        }
        catch (const std::exception &e)
        {
            stop_reason_ = e.what();
            stop();
        }
    }

    rclcpp::Node &node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string robot_;
    std::function<void(bool)> busy_;
    rclcpp_action::Server<Home>::SharedPtr server_;
    rclcpp_action::Client<Navigate>::SharedPtr nav_;
    std::shared_ptr<Handle> goal_;
    NavHandle::SharedPtr nav_goal_;
    rclcpp::TimerBase::SharedPtr timer_;
    Clock::time_point started_, progress_at_;
    std::optional<geometry_msgs::msg::Point> progress_pose_;
    std::string stop_reason_;
    double tolerance_, progress_timeout_, timeout_;
    bool sent_{false}, pending_{false}, cancel_requested_{false};
};
}
