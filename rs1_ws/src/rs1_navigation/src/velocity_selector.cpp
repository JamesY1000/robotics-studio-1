#include "rs1_navigation/velocity_selector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

const std::string HUSKY_NAME = "husky1";
const std::string PARROT_NAME = "parrot1";

namespace rs1_navigation
{

VelocitySelectorNode::VelocitySelectorNode() : Node("velocity_selector_node")
{
    // Declare params
    robot_ = declare_parameter<std::string>("robot", "husky1"); // Default is husky1 but could also be parrot1
    command_timeout_ = declare_parameter<double>("command_timeout", 0.3); // Default to 0.3 for now
    status_timeout_ = declare_parameter<double>("status_timeout", 1.5);
    controller_enable_button_ = declare_parameter<int>("controller_enable_button", 9); // L1 in SDL's controller mapping
    if (controller_enable_button_ < 0)
    {
        throw std::invalid_argument("Controller enable button must be non-negative");
    }

    // Check valid robot names
    if (robot_ != HUSKY_NAME && robot_ != PARROT_NAME)
    {
        throw std::invalid_argument("Robot supplied must be " + HUSKY_NAME + " or " + PARROT_NAME);
    }

    // Check valid command and statues received
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
        !std::isfinite(status_timeout_) || status_timeout_ <= 0.0)
        {
            throw std::invalid_argument("Command and status timeout must be finite and positive");
        }

    // Initialise publisher
    output_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_controlled", rclcpp::QoS(1)); // Depth 1 to keep only latest cmd

    // Initialise subscribers
    const auto command_qos = rclcpp::QoS(1).best_effort(); // Same for subscriber - depth 1 and best effort for cmd vel/twist msgs

    const std::string autonomous_topic = robot_ == PARROT_NAME ? "cmd_vel_combined" : "cmd_vel"; // parrot topic is cmd_vel_combined, husky is just "cmd_vel"

    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        autonomous_topic,
        command_qos,
        std::bind(&VelocitySelectorNode::autoCb, this, std::placeholders::_1)
    );

    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_manual", // TODO (james): Add topic to rs1_interfaces
        command_qos,
        std::bind(&VelocitySelectorNode::manualCb, this, std::placeholders::_1)
    );

    // Both selectors share controller input, but only the selected robot can use it
    controller_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/teleop/cmd_vel_controller", command_qos,
        std::bind(&VelocitySelectorNode::controllerCb, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/teleop/joy", command_qos,
        std::bind(&VelocitySelectorNode::joyCb, this, std::placeholders::_1));

    status_sub_ = create_subscription<rs1_interfaces::msg::MissionStatus>(
        "/mission/status",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&VelocitySelectorNode::statusCb, this, std::placeholders::_1)
    );

    // Create independent wall timer for this node
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), // Hardcode to 20 Hz for now
        [this]()
        {
            publishCommand();
        }
    );

    RCLCPP_INFO(this->get_logger(), "Velocity selector node initialised");

}

void VelocitySelectorNode::autoCb(const geometry_msgs::msg::Twist &msg)
{
    receiveCommand(msg, "auto");
}

void VelocitySelectorNode::manualCb(const geometry_msgs::msg::Twist &msg)
{
    receiveCommand(msg, "keyboard");

}

void VelocitySelectorNode::controllerCb(const geometry_msgs::msg::Twist &msg)
{
    receiveCommand(msg, "controller");
}

void VelocitySelectorNode::resetController()
{
    controller_released_ = false;
    controller_enabled_ = false;
}

void VelocitySelectorNode::joyCb(const sensor_msgs::msg::Joy &msg)
{
    if (age(joy_received_) >= command_timeout_) resetController();
    joy_received_ = std::chrono::steady_clock::now();
    if (!statusFresh() || mode_ != robot_ || manual_input_ != "controller") return;

    if (msg.buttons.size() <= static_cast<size_t>(controller_enable_button_))
    {
        resetController();
        clearCommand();
        return;
    }

    // A held L1 cannot start motion after switching inputs, robots or reconnecting
    if (msg.buttons[controller_enable_button_] == 0)
    {
        controller_released_ = true;
        controller_enabled_ = false;
        clearCommand();
    }
    else
    {
        controller_enabled_ = controller_released_;
    }
}

void VelocitySelectorNode::statusCb(const rs1_interfaces::msg::MissionStatus &msg)
{
    std::string next_mode = "hold"; // If current status control mode is unknown, we want to stop the commands

    // Get current mode from status.state and status.control_mode
    if (!msg.instance_id.empty())
    {
        if (msg.state == "executing" && msg.control_mode == "auto")
        {
            next_mode = "auto"; // Set velocity selector to autonomous (nav2) if mission status is "auto"
        }
        else if (
            msg.state == "manual_assistance" &&
            (msg.control_mode == HUSKY_NAME || msg.control_mode == PARROT_NAME) &&
            (msg.manual_input == "keyboard" || msg.manual_input == "controller")
        )
        {
            next_mode = msg.control_mode; // Set velocity selector to select husky or parrot teleop if the mission status is "manual assistance" and which robot it's for
        }
    }

    // First check freshness of mission status - reject if stale or invalid
    const bool reset_command = !status_seen_ || !statusFresh() || msg.instance_id != instance_id_ ||
        next_mode != mode_ || msg.manual_input != manual_input_ || msg.input_revision != input_revision_;

    // Update status receipt time and coordinator instance
    status_seen_ = true;
    status_received_ = std::chrono::steady_clock::now();
    instance_id_ = msg.instance_id;
    mode_ = next_mode;
    manual_input_ = msg.manual_input;
    input_revision_ = msg.input_revision;

    if (reset_command)
    {
        resetController();
        clearCommand();
    }


}

void VelocitySelectorNode::receiveCommand(const geometry_msgs::msg::Twist &input, const std::string &source)
{
    // Commands cannot select their own command mode
    // Only the chosen input can drive the selected robot, even if both keep publishing
    const std::string selected_source = mode_ == "auto" ? "auto" : mode_ == robot_ ? manual_input_ : "";
    if (!statusFresh() || source != selected_source)
    {
        return;
    }
    if (source == "controller" && (!controller_enabled_ || age(joy_received_) >= command_timeout_)) return;

    // Check received velocity - clear command and return if invalid
    for (const double value : {
        input.linear.x,
        input.linear.y,
        input.linear.z,
        input.angular.x,
        input.angular.y,
        input.angular.z
    })
    {
        if (!std::isfinite(value))
        {
            clearCommand();
            return;
        }
    }

    geometry_msgs::msg::Twist next = input;
    if (source != "auto")
    {
        // TODO (james): Configure these hardcoded cmd_vel limit values
        next.linear.x = std::clamp(input.linear.x, -0.5, 0.5); // Manual linear cmd_vels in m/s
        next.linear.y = robot_ == PARROT_NAME ? std::clamp(input.linear.y, -0.5, 0.5) : 0.0; // Restrict y vels (sidways) to only parrot
        next.linear.z = robot_ == PARROT_NAME ? std::clamp(input.linear.z, -0.3, 0.3) : 0.0; // Restrict z vels (up/down) to only parrot

        // Restrict rotation to yaw only
        next.angular.x = 0.0;
        next.angular.y = 0.0;
        next.angular.z = std::clamp(input.angular.z, -0.7, 0.7);
    }

    // Update the filtered command and timestamps
    command_ = next;
    command_received_ = true;
    command_time_ = std::chrono::steady_clock::now();
}

double VelocitySelectorNode::age(std::chrono::steady_clock::time_point timestamp)
{
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - timestamp).count();
    return age;
}

bool VelocitySelectorNode::statusFresh() const
{
    // Status is fresh if it has not yet been seen and is new (hasn't exceeded status_timeout)
    auto fresh_status = !status_seen_ || age(status_received_) < status_timeout_;
    return fresh_status;
}

void VelocitySelectorNode::clearCommand()
{
    // Create and publish an empty twist msg
    command_received_ = false;
    command_ = geometry_msgs::msg::Twist();

    output_cmd_pub_->publish(command_);
}

void VelocitySelectorNode::publishCommand()
{
    // Stop on driver or converter loss and require a fresh L1 press before moving again
    if (!statusFresh() || age(joy_received_) >= command_timeout_ ||
        (manual_input_ == "controller" && command_received_ && age(command_time_) >= command_timeout_))
    {
        resetController();
    }
    // Velocity selector will publish velocities only if current mode is in "auto" or a robot (manual assistance/override)
    const bool selected =
        mode_ == "auto" || mode_ == robot_;

    // if mode is neither, or commands are not being received or are stale, empty commands will be sent
    if (!selected || !statusFresh() || !command_received_ || age(command_time_) >= command_timeout_ ||
        (mode_ == robot_ && manual_input_ == "controller" && !controller_enabled_))
    {
        clearCommand();
        return;
    }

    output_cmd_pub_->publish(command_);
}

}
