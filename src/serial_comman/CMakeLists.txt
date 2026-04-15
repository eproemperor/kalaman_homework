cmake_minimum_required(VERSION 3.8)
project(serial_comman)  # 项目名保持不变

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 生成自定义消息（这会创建一个名为 ${PROJECT_NAME} 的 target）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Serialcom.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# 包含目录
include_directories(
  include
)

# 创建可执行文件 - 注意：这里用了不同的名字 "serial_comman_node"
add_executable(serial_comman_node
  src/serial_comman.cpp
)

# 链接依赖 - 注意：这里用的是 "serial_comman_node"
ament_target_dependencies(serial_comman_node
  rclcpp
  std_msgs
  builtin_interfaces
)

# 链接生成的消息库 - 注意：这里用的是 "serial_comman_node"
target_link_libraries(serial_comman_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 安装 - 注意：这里用的是 "serial_comman_node"
install(TARGETS serial_comman_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
)

ament_package()