#include "rs1_safety/emergency_stop.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStop>());
  rclcpp::shutdown();
  return 0;
}