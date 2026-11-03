#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rs1_interfaces/msg/mission_status.hpp"
#include "rs1_interfaces/msg/safety_state.hpp"
#include "rs1_interfaces/srv/safety_stop.hpp"

using Clock = std::chrono::steady_clock;
using SafetyState = rs1_interfaces::msg::SafetyState;
using SafetyStop = rs1_interfaces::srv::SafetyStop;
using MissionStatus = rs1_interfaces::msg::MissionStatus;

class EmergencyStop : public rclcpp::Node
{
public:
    EmergencyStop() : Node("emergency_stop")
    {
        // Restarting this node must never silently release a previous stop
        state_.instance_id = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        state_.husky_stopped = true;
        state_.parrot_stopped = true;
        const auto qos = rclcpp::QoS(1).reliable().transient_local();
        publisher_ = create_publisher<SafetyState>("/safety/state", qos);
        mission_sub_ = create_subscription<MissionStatus>("/mission/status", qos,
            [this](const MissionStatus &msg) { mission_ = msg; mission_at_ = Clock::now(); });
        service_ = create_service<SafetyStop>("/safety/stop",
            [this](const std::shared_ptr<SafetyStop::Request> request,
                   std::shared_ptr<SafetyStop::Response> response) { change(*request, *response); });
        // The selectors use a wall-clock watchdog, including while Gazebo is paused
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publish(); });
        publish();
    }

private:
    void publish()
    {
        state_.header.stamp = now();
        publisher_->publish(state_);
    }

    void change(const SafetyStop::Request &request, SafetyStop::Response &response)
    {
        response.success = false;
        if (request.robot != "husky1" && request.robot != "parrot1" && request.robot != "all")
        {
            response.message = "Choose husky1, parrot1 or all";
            return;
        }
        if (!request.stopped)
        {
            if (request.expected_instance_id != state_.instance_id || request.expected_revision != state_.revision)
            {
                response.message = "Safety state changed; review the latest stop state before resetting";
                return;
            }
            // Reset cannot release old Nav2 commands while cancellation is still in flight
            const bool acknowledged = !mission_.instance_id.empty() &&
                std::chrono::duration<double>(Clock::now() - mission_at_).count() < 0.75 &&
                mission_.safety_instance_id == state_.instance_id && mission_.safety_revision == state_.revision;
            if (!acknowledged || mission_.control_mode != "hold" || !mission_.cleanup_complete)
            {
                response.message = "Wait for the mission coordinator to acknowledge the stop and finish stopping its actions";
                return;
            }
        }
        if (request.robot == "husky1" || request.robot == "all") state_.husky_stopped = request.stopped;
        if (request.robot == "parrot1" || request.robot == "all") state_.parrot_stopped = request.stopped;
        // Even a repeated Stop invalidates any Reset already in flight from an older UI state
        ++state_.revision;
        publish();
        response.success = true;
        response.message = request.stopped ? "Software stop latched" : "Stop reset; select Resume or manual assistance to move";
    }

    SafetyState state_;
    MissionStatus mission_;
    Clock::time_point mission_at_{};
    rclcpp::Publisher<SafetyState>::SharedPtr publisher_;
    rclcpp::Subscription<MissionStatus>::SharedPtr mission_sub_;
    rclcpp::Service<SafetyStop>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyStop>());
    rclcpp::shutdown();
    return 0;
}
