cmake_minimum_required(VERSION 3.8)
project(camera_info_interceptor)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加头文件目录
include_directories(include)

# 创建共享库实现
add_library(${PROJECT_NAME} SHARED
  src/camera_info_interceptor_component.cpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "camera_info_interceptor::CameraInfoInterceptorNode")

# 创建节点可执行文件
add_executable(camera_info_interceptor_node 
  src/camera_info_interceptor_node.cpp
)
target_link_libraries(camera_info_interceptor_node
  ${PROJECT_NAME}
)
ament_target_dependencies(camera_info_interceptor_node
  rclcpp
  sensor_msgs
)

# 安装可执行文件和库
install(TARGETS
  ${PROJECT_NAME}
  camera_info_interceptor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
