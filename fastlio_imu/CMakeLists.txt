cmake_minimum_required(VERSION 3.8)
project(fastlio_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 找所有需要的包 (仅保留原有包，删除Python模块相关的ament_cmake_python)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED)        
find_package(rosidl_default_generators REQUIRED)
# 只保留rclpy（运行时依赖），删除ament_cmake_python
find_package(rclpy REQUIRED)

# 2. 先定义消息生成 (原有逻辑不变)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FcuState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 3. 编译C++节点 (原有逻辑不变)
add_executable(fastlio_to_fcu src/fastlio_to_fcu.cpp)

# 4. 添加基础依赖 (原有逻辑不变)
ament_target_dependencies(fastlio_to_fcu
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

# 5. 链接自定义消息生成目标 (警告不影响功能，暂时保留)
rosidl_target_interfaces(fastlio_to_fcu
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ============ 仅保留Python可执行脚本安装（彻底删除ament_python_install_package） ============
install(PROGRAMS
  src/ros_to_serial.py
  src/servo_control.py
  DESTINATION lib/${PROJECT_NAME}
)
# ================================================================

# 6. 原有安装逻辑 (保留)
install(TARGETS fastlio_to_fcu DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_package()