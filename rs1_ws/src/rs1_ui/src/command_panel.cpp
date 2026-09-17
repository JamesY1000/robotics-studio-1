#include "rs1_ui/command_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>

#include <pluginlib/class_list_macros.hpp>

namespace command_panel
{

CommandPanel::CommandPanel()
: rqt_gui_cpp::Plugin(),
  widget_(nullptr),
  linear_velocity_scale_(1.0),
  velocity_value_(0.0),
  cmd_vel_topic_("/husky1/cmd_vel")
{
  setObjectName("CommandPanel");
}

void CommandPanel::buildUI()
{
  widget_ = new QWidget;
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
  forward_button_->setCheckable(true);
  reverse_button_->setCheckable(true);
  e_stop_button_->setCheckable(true);
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

  widget_->setLayout(layout); // Set the layout for the main widget
  widget_->setWindowTitle("Command Panel"); // Set the window title for the main widget

  // Connect button signals to their respective slots
  connect(forward_button_, &QPushButton::clicked, this, &CommandPanel::onForwardClicked);
  connect(reverse_button_, &QPushButton::clicked, this, &CommandPanel::onReverseClicked);
  connect(e_stop_button_, &QPushButton::clicked, this, &CommandPanel::onEStopClicked);
  connect(robot_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &CommandPanel::onRobotChanged);
}

void CommandPanel::initPlugin(qt_gui_cpp::PluginContext & context)
{
  buildUI();

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // Add the widget to the user interface
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Republish cmd_vel continuously at 10 Hz, matching common ROS 2 driver expectations.
  publish_timer_ = new QTimer(widget_);
  connect(publish_timer_, &QTimer::timeout, this, &CommandPanel::publishVelocity);
  publish_timer_->start(100);
}

void CommandPanel::shutdownPlugin()
{
  // Stop the timer when shutting down the plugin
  if (publish_timer_) {
    publish_timer_->stop();
  }
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
  // Check if E-Stop is active; if so, ignore forward command
  if (e_stop_button_->isChecked()) {
    return;
  }

  // Check if the forward button is checked (activated)
  if (!forward_button_->isChecked()) {
    // If the forward button is unchecked, reset velocity to zero
    forward_button_->setStyleSheet("");  // Reset forward button style
    velocity_value_ = 0.0;
    velocity_label_->setText("0.00 m/s");
    return;
  }

  // Set the forward button style to indicate it's active and deactivate the reverse button
  forward_button_->setStyleSheet("background-color: green; color: white;");
  reverse_button_->setChecked(false);
  reverse_button_->setStyleSheet("");  // Reset reverse button style
  // Set the velocity to a positive value to indicate forward motion
  velocity_value_ = 25.0;
  velocity_label_->setText("0.25 m/s");
}

void CommandPanel::onReverseClicked()
{
  // Check if E-Stop is active; if so, ignore reverse command
  if (e_stop_button_->isChecked()) {
    return;
  }

  // Check if the reverse button is checked (activated)
  if (!reverse_button_->isChecked()) {
    // If the reverse button is unchecked, reset velocity to zero
    reverse_button_->setStyleSheet("");  // Reset reverse button style
    velocity_value_ = 0.0;
    velocity_label_->setText("0.00 m/s");
    return;
  }

  // Set the reverse button style to indicate it's active and deactivate the forward button
  reverse_button_->setStyleSheet("background-color: blue; color: white;");
  forward_button_->setChecked(false);
  forward_button_->setStyleSheet("");  // Reset forward button style
  // Set the velocity to a negative value to indicate reverse motion
  velocity_value_ = -25.0;
  velocity_label_->setText("-0.25 m/s");
}

void CommandPanel::onEStopClicked()
{
  // Check if the E-Stop button is checked (activated)
  if (e_stop_button_->isChecked()) {
    // Change the E-Stop button style to indicate it's active
    e_stop_button_->setStyleSheet("background-color: red; color: white;");
    e_stop_button_->setText("E-Stop Activated");
    // deactivate the forward and reverse buttons
    forward_button_->setChecked(false);
    forward_button_->setStyleSheet("");  // Reset forward button style
    reverse_button_->setChecked(false);
    reverse_button_->setStyleSheet("");  // Reset reverse button style
    // Reset the velocity to zero when E-Stop is activated
    velocity_value_ = 0.0;
    velocity_label_->setText("0.00 m/s");
  } else {
    e_stop_button_->setStyleSheet("");
    e_stop_button_->setText("E-Stop");
  }
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
PLUGINLIB_EXPORT_CLASS(command_panel::CommandPanel, rqt_gui_cpp::Plugin)