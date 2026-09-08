#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mission_coordinator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rs1_mission::MissionCoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}