cmake_minimum_required(VERSION 3.8)
project(bag_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(bag_converter src/bag_converter.cpp)

ament_target_dependencies(bag_converter
  rclcpp
  rosbag2_cpp
  sensor_msgs
  livox_ros_driver2
)

install(TARGETS bag_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
