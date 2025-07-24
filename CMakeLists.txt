cmake_minimum_required(VERSION 3.5)
project(pcd_image_publisher)

# 避免常见 policy 警告
if(POLICY CMP0012)
  cmake_policy(SET CMP0012 NEW)
endif()
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pcd_image_publisher src/pcd_image_publisher.cpp)
target_link_libraries(pcd_image_publisher
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcd_image_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  OpenCV
  cv_bridge
)

install(TARGETS pcd_image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
