#include "simpleMapping.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto SM = std::make_shared<SimpleMapping>("simple_mapping");
  SM->tfInitial();

  rclcpp::spin(SM);
  rclcpp::shutdown();

  return 0;
}


