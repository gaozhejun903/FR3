cmake_minimum_required(VERSION 3.8)
project(dualarm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(epg50_gripper_ros REQUIRED)
find_package(robo_ctrl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(depth_handler REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# 创建卡尔曼滤波器库
add_library(kalman_filter src/kalman.cpp)
target_link_libraries(kalman_filter Eigen3::Eigen)

add_executable(dualarm_node 
                src/main.cpp
                src/kalman.cpp)
ament_target_dependencies(dualarm_node
  rclcpp
  rclcpp_action
  epg50_gripper_ros
  robo_ctrl
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
  depth_handler
)

# 创建 robot_main 可执行文件
add_executable(robot_main 
                src/main.cpp
                src/kalman.cpp)
ament_target_dependencies(robot_main
  rclcpp
  rclcpp_action
  epg50_gripper_ros
  robo_ctrl
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
  depth_handler
)

# 创建卡尔曼滤波器示例程序
add_executable(kalman_example include/dualarm/kalman_example.cpp)
target_link_libraries(kalman_example kalman_filter Eigen3::Eigen)

# 创建重构后的机器人主控程序
add_executable(robot_main_refactored 
                src/main_refactored.cpp
                src/kalman.cpp)
ament_target_dependencies(robot_main_refactored
  rclcpp
  rclcpp_action
  epg50_gripper_ros
  robo_ctrl
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
  depth_handler
)

install(TARGETS dualarm_node robot_main kalman_example robot_main_refactored
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()