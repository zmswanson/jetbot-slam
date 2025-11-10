#include "jetbot_base/wheel_odom_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetbot_base::WheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}
