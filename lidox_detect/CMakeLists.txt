cmake_minimum_required(VERSION 3.8)
project(lidox_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# cloud_filter_node
add_executable(cloud_filter_node src/cloud_filter_node.cpp)
ament_target_dependencies(cloud_filter_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
  pcl_ros
  pcl_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(cloud_filter_node ${PCL_LIBRARIES})

# shape_detect_node (merged ring + pillar, single clustering pass)
add_executable(shape_detect_node src/shape_detect_node.cpp)
ament_target_dependencies(shape_detect_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(shape_detect_node ${PCL_LIBRARIES})

# 安装
install(TARGETS
  cloud_filter_node
  shape_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()