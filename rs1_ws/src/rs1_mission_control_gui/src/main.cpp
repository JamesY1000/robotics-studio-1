#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "rs1_mission_control_gui/main_window.hpp"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rs1_mission_control_gui");

  rs1_mission_control_gui::MainWindow window(node);
  window.show();

  int result = app.exec();

  rclcpp::shutdown();
  return result;
}
