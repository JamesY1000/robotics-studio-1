#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rs1_navigation/velocity_selector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rs1_navigation::VelocitySelectorNode>());
  rclcpp::shutdown();
  return 0;
}
