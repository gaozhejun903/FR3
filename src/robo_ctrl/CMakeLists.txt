cmake_minimum_required(VERSION 3.8)
project(robo_ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-g)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 添加tf2相关依赖
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
# 添加可视化消息依赖
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 设置包含目录 - 包含libfairino的头文件路径
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接库目录 - 修改为正确的lib目录
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RobotMove.srv"
  "srv/RobotMoveCart.srv"
  "srv/RobotServo.srv"
  "msg/RobotState.msg"
  "msg/JointPosition.msg"
  "msg/TCPPose.msg"
  "srv/RobotSetSpeed.srv"
  "srv/RobotServoLine.srv"
  "srv/RobotAct.srv"
  "srv/RobotServoJoint.srv"
  "srv/RobotActJ.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 声明可执行组件库
add_library(robo_ctrl_components SHARED
  src/robo_ctrl_node.cpp
)

# 设置库的属性
set_target_properties(robo_ctrl_components PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# 链接库依赖 - 库文件位于lib目录下
target_link_libraries(robo_ctrl_components
  fairino
)

# 链接依赖项
ament_target_dependencies(robo_ctrl_components
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
)

# 使用rosidl_get_typesupport_target()和target_link_libraries()替代rosidl_target_interfaces()
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robo_ctrl_components ${cpp_typesupport_target})

# 注册组件
rclcpp_components_register_nodes(robo_ctrl_components
  "robo_ctrl::RoboCtrlNode"
)

# 声明可执行文件
add_executable(robo_ctrl_node src/main.cpp)


# 链接到组件库
target_link_libraries(robo_ctrl_node
  robo_ctrl_components
)

# 添加客户端示例可执行文件
# 添加圆弧规划测试程序
add_executable(test_circular_planner src/circular_test.cpp)

# 链接依赖项
ament_target_dependencies(test_circular_planner
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

# 链接 cpp 类型支持（消息类型）
target_link_libraries(test_circular_planner ${cpp_typesupport_target})

add_executable(robot_move_client src/robot_move_client.cpp)
add_executable(robot_move_cart_client src/robot_move_cart_client.cpp)

# 链接依赖项
ament_target_dependencies(robot_move_client
  rclcpp
)
ament_target_dependencies(robot_move_cart_client
  rclcpp
)

# 链接消息库
target_link_libraries(robot_move_client ${cpp_typesupport_target})
target_link_libraries(robot_move_cart_client ${cpp_typesupport_target})

# 添加高层规划执行节点
add_executable(high_level_node src/high_level.cpp)
ament_target_dependencies(high_level_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(high_level_node
  ${cpp_typesupport_target}
)

# 安装高层规划执行节点
install(
  TARGETS high_level_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出库的相关信息
ament_export_include_directories(include)
ament_export_libraries(robo_ctrl_components)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库
install(
  TARGETS robo_ctrl_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装libfairino库文件 - 从lib目录安装
install(
  DIRECTORY lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libfairino*"
)

# 安装可执行文件
install(
  TARGETS 
    robo_ctrl_node
    robot_move_client
    robot_move_cart_client
    high_level_node
    test_circular_planner
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

ament_package()
