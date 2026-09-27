#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "parrot_mission_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rs1_mission::ParrotMissionExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}