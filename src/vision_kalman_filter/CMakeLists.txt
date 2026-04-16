cmake_minimum_required(VERSION 3.8)
project(vision_kalman_filter)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter -Wno-unused-variable)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)  
find_package(rosidl_default_generators REQUIRED)

# 添加自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Serialcom.msg"
  DEPENDENCIES
    std_msgs  
)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(kalman_filter_node
  src/main.cpp
  src/ros_image_subscriber.cpp
  src/serial_comm.cpp
  src/kalman_filter.cpp
)

# 设置依赖
ament_target_dependencies(kalman_filter_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport  
)

target_link_libraries(kalman_filter_node
  ${OpenCV_LIBRARIES}  # 添加OpenCV库链接
)

# 链接库
rosidl_target_interfaces(kalman_filter_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 安装可执行文件
install(TARGETS kalman_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport  
  OpenCV
  rosidl_default_runtime
)

# ament 打包
ament_package()