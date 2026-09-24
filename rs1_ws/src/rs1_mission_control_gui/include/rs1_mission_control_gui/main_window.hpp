#ifndef RS1_MISSION_CONTROL_GUI__MAIN_WINDOW_HPP_
#define RS1_MISSION_CONTROL_GUI__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QLayout>
#include <QTableWidget>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rs1_mission_interfaces/msg/tree_sample.hpp>
#include <rs1_mission_interfaces/msg/tree_sample_array.hpp>

#include "rs1_mission_control_gui/map_widget.hpp"
#include "rs1_mission_control_gui/robot_card_widget.hpp"

namespace rs1_mission_control_gui
{

// Per-robot widgets + ROS interfaces.
struct RobotEntry
{
  QString name;           // ROS namespace, e.g. "husky1"
  QString display_name;   // e.g. "Husky UGV"
  bool is_uav = false;
  QColor color;
  bool connected = false;

  RobotCardWidget * card = nullptr;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr return_home_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~MainWindow() override;

private Q_SLOTS:
  void onEstopAllClicked();
  void onReturnHomeAllClicked();
  void onStartMissionClicked();
  void spinRos();
  void updateClock();

private:
  QWidget * buildSidebar();
  QWidget * buildTopBar();
  QWidget * buildMapCard();
  QWidget * buildMissionProgressCard();
  QWidget * buildSelectedTreesCard();
  QWidget * buildAgbEstimateCard();
  QWidget * buildRobotColumn();

  void setupRosInterfaces();
  void updateReadyBadge();
  void updateMissionProgressUi();

  rclcpp::Node::SharedPtr node_;
  QTimer * ros_spin_timer_;
  QTimer * clock_timer_;

  MapWidget * map_widget_;
  std::vector<RobotEntry> robots_;

  // Top bar
  QLabel * clock_label_;
  QLabel * ready_badge_;

  // Sidebar system status
  QLabel * comms_status_dot_;
  QLabel * uav_status_dot_;
  QLabel * ugv_status_dot_;

  // Mission progress card
  QProgressBar * mission_progress_bar_;
  QLabel * mission_progress_percent_;
  QLabel * step_uav_survey_;
  QLabel * step_traversability_;
  QLabel * step_sampling_;
  QLabel * step_returning_;
  QLabel * step_complete_;

  // Selected trees table
  QTableWidget * selected_trees_table_;

  // AGB estimate card
  QLabel * agb_estimate_value_;
  QLabel * agb_estimate_delta_;
  QLabel * agb_estimate_subtext_;

  // ROS-driven state
  bool comms_ok_ = false;
  QString current_stage_ = "uav_survey";  // uav_survey|traversability|sampling|returning|complete
  int trees_sampled_ = 0;
  int trees_total_ = 0;
  float mission_progress_percent_value_ = 0.0f;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr agb_map_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr other_trees_sub_;
  rclcpp::Subscription<rs1_mission_interfaces::msg::TreeSampleArray>::SharedPtr selected_trees_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr roi_boundary_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr comms_ok_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_stage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mission_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trees_sampled_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trees_total_sub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr agb_current_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr agb_delta_sub_;
};

}  // namespace rs1_mission_control_gui

#endif  // RS1_MISSION_CONTROL_GUI__MAIN_WINDOW_HPP_
