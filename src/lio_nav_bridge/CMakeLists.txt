cmake_minimum_required(VERSION 3.5)
project(lio_nav_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#Check yolo build flag
if(NOT TRT_ENABLED)
    set(TRT_ENABLED OFF CACHE BOOL "Enable or disable TensorRT")
endif()

if(TRT_ENABLED)
  add_definitions(-DTRT_ENABLED)
  find_package(CUDAToolkit REQUIRED)
  find_package(dddmr_trt REQUIRED)

  message(STATUS "TensorRT is enabled. Compiling TRT and YOLO features")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake")

  include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${dddmr_trt_INCLUDE_DIRS}
  )
else()
    message(STATUS "TensorRT is disabled.")
    include_directories(
      include
      ${PCL_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
endif()

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pcl_conversions
  cv_bridge
)

#simple mapping node
add_executable(lio_nav_bridge_node
  src/lio_nav_bridge_node.cpp
  src/simpleMapping.cpp
)
target_link_libraries(lio_nav_bridge_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(lio_nav_bridge_node
  ${dependencies}
)

install(TARGETS
  lio_nav_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()