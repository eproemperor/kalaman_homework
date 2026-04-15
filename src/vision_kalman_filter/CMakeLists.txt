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
find_package(image_transport REQUIRED)  # 添加 image_transport

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/ros_image_subscriber.cpp
  src/serial_comm.cpp
  src/kalman_filter.cpp
)

# 设置依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport  # 添加 image_transport
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
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
)

# ament 打包
ament_package()