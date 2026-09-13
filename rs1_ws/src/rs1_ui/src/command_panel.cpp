#include "rs1_ui/command_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>

#include <rviz_common/display_context.hpp>

namespace command_panel
{

CommandPanel::CommandPanel(QWidget * parent)
: rviz_common::Panel(parent),
  linear_velocity_scale_(1.0),
  velocity_value_(0.0),
  cmd_vel_topic_("/husky1/cmd_vel")
{
  auto * layout = new QVBoxLayout;

  // --- Robot selection ---
  auto * name_layout = new QHBoxLayout;
  name_layout->addWidget(new QLabel("Robot:"));
  robot_combo_ = new QComboBox;
  robot_combo_->addItems({"Husky", "Parrot"});
  name_layout->addWidget(robot_combo_);
  layout->addLayout(name_layout);

  // --- Forward / Backwards / Stop row ---
  auto * robot_control_layout = new QHBoxLayout;
  forward_button_ = new QPushButton("Forward");
  reverse_button_ = new QPushButton("Reverse");
  e_stop_button_ = new QPushButton("E-Stop");
  robot_control_layout->addWidget(forward_button_);
  robot_control_layout->addWidget(reverse_button_);
  robot_control_layout->addWidget(e_stop_button_);
  layout->addLayout(robot_control_layout);

  // --- Velocity info row ---
  auto * velocity_layout = new QHBoxLayout;
  velocity_layout->addWidget(new QLabel("Linear vel:"));
  velocity_label_ = new QLabel("0.00 m/s");
  velocity_layout->addWidget(velocity_label_);
  layout->addLayout(velocity_layout);

  // Set the layout for the panel
  setLayout(layout);

  // Connect button signals to their respective slots
  connect(forward_button_, &QPushButton::clicked, this, &CommandPanel::onForwardClicked);
  connect(reverse_button_, &QPushButton::clicked, this, &CommandPanel::onReverseClicked);
  connect(e_stop_button_, &QPushButton::clicked, this, &CommandPanel::onEStopClicked);
  connect(robot_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &CommandPanel::onRobotChanged);

  // Republish cmd_vel continuously at 10 Hz, matching common ROS 2 driver expectations.
  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &CommandPanel::publishVelocity);
}

CommandPanel::~CommandPanel() = default;

void CommandPanel::onInitialize()
{
  // RViz2 panels get their ROS node from the DisplayContext, not from
  // rclcpp::init() directly - the node already belongs to the RViz process.
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  publish_timer_->start(100);
}

void CommandPanel::onRobotChanged(int index)
{
  velocity_value_ = 0.0;
  velocity_label_->setText("0.00 m/s");

  cmd_vel_topic_ = index == 0 ? "/husky1/cmd_vel" : "/parrot1/cmd_vel";
  if (node_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }
}

void CommandPanel::onForwardClicked()
{
  // Set the velocity to a positive value to indicate forward motion
  velocity_value_ = 25.0;
  velocity_label_->setText("0.25 m/s");
}

void CommandPanel::onReverseClicked()
{
  // Set the velocity to a negative value to indicate reverse motion
  velocity_value_ = -25.0;
  velocity_label_->setText("-0.25 m/s");
}

void CommandPanel::onEStopClicked()
{
  // Stop the robot by setting velocity to zero
  velocity_value_ = 0.0;
  velocity_label_->setText("0.00 m/s");
}

void CommandPanel::publishVelocity()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist msg;
  msg.linear.x = (velocity_value_ / 100.0) * linear_velocity_scale_;
  cmd_vel_pub_->publish(msg);
}

}  // namespace command_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(command_panel::CommandPanel, rviz_common::Panel)