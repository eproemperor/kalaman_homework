cmake_minimum_required(VERSION 3.8)
project(serial_comman)  

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Serialcom.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# 包含目录
include_directories(
  include
)

add_executable(serial_comman_node
  src/serial_comman.cpp
)

ament_target_dependencies(serial_comman_node
  rclcpp
  std_msgs
  builtin_interfaces
)

target_link_libraries(serial_comman_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

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