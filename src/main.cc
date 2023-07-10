#include "fusion_engine_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionEngineNode>());
  rclcpp::shutdown();
  return 0;
}