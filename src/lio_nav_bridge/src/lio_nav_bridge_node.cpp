#include "simpleMapping.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto SM = std::make_shared<SimpleMapping>("lio_nav_bridge");
  SM->tfInitial();

  rclcpp::spin(SM);
  rclcpp::shutdown();

  return 0;
}


