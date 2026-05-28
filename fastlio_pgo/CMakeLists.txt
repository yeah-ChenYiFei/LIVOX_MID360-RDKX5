cmake_minimum_required(VERSION 3.8)
project(fastlio_pgo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(std_srvs REQUIRED)

find_package(OpenCV REQUIRED)
# 可选调试输出
message(STATUS "GTSAM_LIBRARIES: ${GTSAM_LIBRARIES}")

# 如果 find_package 没有设置 GTSAM_INCLUDE_DIRS，手动设置（一般不需要）
if(NOT GTSAM_INCLUDE_DIRS)
    set(GTSAM_INCLUDE_DIRS /opt/ros/humble/include)
endif()
# 添加自己的头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(sc_pgo_node
  src/sc_pgo_node.cpp
  src/Scancontext.cpp
)

# 使用 ament_target_dependencies 添加所有 ROS2 相关依赖
ament_target_dependencies(sc_pgo_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
)

# 添加第三方库链接（Eigen3, PCL, GTSAM）
target_link_libraries(sc_pgo_node
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam   # 或者 GTSAM::gtsam
)

# 安装规则
install(TARGETS sc_pgo_node
  DESTINATION lib/${PROJECT_NAME})

# 可选：安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()