cmake_minimum_required(VERSION 3.8)
project(fastlio_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 找所有需要的包 (新增了 std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED)        # 必须添加，因为你的 msg 用到了 Header
find_package(rosidl_default_generators REQUIRED)

# 2. 先定义消息生成 (必须放在 add_executable 之前)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Newmsg.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 3. 编译节点
add_executable(fastlio_to_fcu src/fastlio_to_fcu.cpp)

# 4. 添加基础依赖
ament_target_dependencies(fastlio_to_fcu
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

# 5. 【关键步骤】链接自定义消息生成目标
# 这行代码解决了 "No such file or directory" 的问题
# 它强制让节点在消息头文件生成之后再进行编译
rosidl_target_interfaces(fastlio_to_fcu
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 6. 安装
install(TARGETS fastlio_to_fcu DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_package()
