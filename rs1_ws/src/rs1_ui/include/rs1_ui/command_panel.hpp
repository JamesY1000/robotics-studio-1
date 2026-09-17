#ifndef RS1_UI__COMMAND_PANEL_HPP_
#define RS1_UI__COMMAND_PANEL_HPP_

#include <string>

#include <rqt_gui_cpp/plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QTimer>

namespace command_panel
{

// An RViz2 panel with Forward, Reverse, and E-Stop controls for husky and parrot.
class CommandPanel : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  CommandPanel();

  // rqt_gui_cpp calls initPlugin() once the plugin's node_ (inherited
  // from rqt_gui_cpp::Plugin) is ready - publishers must be created
  // here, not in the constructor.
  void initPlugin(qt_gui_cpp::PluginContext & context) override;
  void shutdownPlugin() override;

private Q_SLOTS:

  /* Handles changes to the robot selection */
  void onRobotChanged(int index);

  /* Handles the Forward button click */
  void onForwardClicked();

  /* Handles the Reverse button click */
  void onReverseClicked();

  /* Handles the E-Stop button click */
  void onEStopClicked();

  /* Publishes the current velocity to the cmd_vel topic */
  void publishVelocity();

private:
  void buildUI();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // Publisher to send velocity commands

  QWidget * widget_; // The main widget for the panel
  QPushButton * forward_button_; // Button to move the robot forward
  QPushButton * reverse_button_; // Button to move the robot backward
  QPushButton * e_stop_button_; // Button to stop the robot immediately
  QComboBox * robot_combo_; // Dropdown to select the robot (Husky or Parrot)
  QLabel * velocity_label_; // Label to display the current linear velocity in m/s

  QTimer * publish_timer_; // Used to publish velocity commands at a fixed rate (10hz)

  double linear_velocity_scale_; // Scale factor for linear velocity (can be adjusted based on robot capabilities)
  double velocity_value_; // Current linear velocity value to be published (in m/s)
  std::string cmd_vel_topic_; // The topic to publish velocity commands to
};

}  // namespace command_panel

#endif  // RS1_UI__COMMAND_PANEL_HPP_